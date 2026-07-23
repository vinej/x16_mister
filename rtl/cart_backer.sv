// ============================================================================
// cart_backer.sv -- cart image restore/persistence via hps_io virtual disk
// slot 2 ("Mount Cart") (jyv 2026-07-07; dual-mode 2026-07-23).
//
// ROM banks 32-255 live in SDRAM at CART_BASE.  This module backs them with a
// disk image mounted on SC2, in one of two modes chosen by the OSD toggle
// `typed_mode` (latched at mount):
//
//   * typed_mode = 0  -- RAW ALL-RAM ("Mount Cart RAM", the original behavior):
//       every cart bank 32-255 is writable RAM (cart_wmask all-ones); the image
//       restores LINEARLY (file offset 0 = bank 32 offset 0, same as "Load
//       Cart") and CPU writes are persisted by the dirty-sector SAVE-BACK below.
//   * typed_mode = 1  -- TYPED .crt ("Mount Cartridge"): the restore parses the
//       X16 CARTRIDGE header (magic + bank_info[]) to build a PER-BANK writable
//       mask and place each present bank's 16 KB block at its real bank address
//       (sparse -- NONE/uninit banks are skipped).  Read-only on disk; RAM/NVRAM
//       writes stay volatile in SDRAM (matches the emulator).
//
//   * RESTORE on mount: the image streams into SDRAM through the existing
//     loader port (muxed in x16.sv with the F2 ioctl cart loader -- they
//     never run concurrently).  Reads use 16 KB multi-block transfers
//     (sd_blk_cnt=31), so a full 3.5 MB image lands in well under a second.
//   * SAVE-BACK (raw mode only, ro_r=0): ext_ram_sdram pulses wr_snoop for
//     every committed CPU write; writes inside the image window mark a
//     512-byte sector dirty in an 8192-bit map.  SAVE_DELAY after the last
//     write, a sweep saves ONLY dirty sectors: clear bit -> prefetch the
//     sector through the lowest-priority bk_* read port -> sd_wr the block.
//     A CPU write landing mid-save re-dirties its sector and a follow-up
//     sweep gets it (clear-before-read ordering makes that lossless).
//
// Everything is in the hps/SDRAM 100 MHz domain -- no CDC in this module.
// Mounts are latched outside the FSM reset (mnt_pend), because MiSTer Main
// auto-remounts SC images while the ROM download still holds the system
// reset (found the hard way on the NVRAM slot).
//
// Only the first min(img_size, CART_BYTES) bytes are backed: size the
// image to the cart you want persistent (must be a multiple of 512).
// ============================================================================
module cart_backer #(
    parameter integer SAVE_DELAY = 200_000_000,   // 2 s @ 100 MHz
    parameter [24:0]  CART_BASE  = 25'h480000,
    parameter [24:0]  CART_BYTES = 25'h380000     // banks 32-255 = 3.5 MB
) (
    input  logic        clk,             // hps_io clk_sys (sdram_clk, 100 MHz)
    input  logic        reset_n,

    // hps_io virtual disk slot 2
    input  logic        img_mounted,     // pulse; this slot's bit
    input  logic        img_readonly,    // valid during the mount pulse
    input  logic [63:0] img_size,        // valid during the mount pulse
    input  logic        typed_mode,      // OSD "Cart mount": 1 = parse a typed .crt
                                         //   (per-bank ROM/RAM/NVRAM, volatile),
                                         //   0 = raw all-RAM image (every bank
                                         //   writable RAM) + dirty-sector save-back
    output logic [31:0] sd_lba,
    output logic  [5:0] sd_blk_cnt,
    output logic        sd_rd,
    output logic        sd_wr,
    input  logic        sd_ack,
    input  logic [13:0] sd_buff_addr,
    input  logic  [7:0] sd_buff_dout,
    output logic  [7:0] sd_buff_din,
    input  logic        sd_buff_wr,

    // restore stream -> ext_ram_sdram loader port (x16.sv muxes with ioctl)
    output logic        rst_ld_wr,
    output logic [24:0] rst_ld_addr,
    output logic  [7:0] rst_ld_data,    // registered WITH the pulse (a comb
                                        // mux on `state` made the final wipe
                                        // byte pick up stale data)
    input  logic        ld_busy,         // loader FIFO backpressure (wipe pacing)

    // sector prefetch <- ext_ram_sdram backup read port
    output logic        bk_rd,
    output logic [24:0] bk_addr,
    input  logic  [7:0] bk_rdata,
    input  logic        bk_ack,

    // dirty snoop <- ext_ram_sdram (committed CPU writes)
    input  logic        wr_snoop,
    input  logic [24:0] wr_snoop_addr,

    // held high from mount until the restore finishes -- x16.sv puts this
    // into sys_rst_n so mounting a cart reboots into it, exactly like the
    // F2 "Load Cart" download hold.  NOTE: this module must therefore run
    // on the MEMORY-side reset (alive during download/CPU-reset holds),
    // never on a reset derived from sys_rst_n -- that would deadlock.
    output logic        restoring,

    // Per-bank writable mask from the .crt bank_info table (bit N = ROM bank N
    // is a RAM/NVRAM bank -> CPU-writable).  Read in the cpu_clk domain; it is
    // quasi-static -- loaded here while `restoring` holds the CPU in reset, and
    // stable before the CPU runs (same discipline as the ROM/cart loaders).
    // Bits 0-31 are unused (system ROM banks).
    output logic [255:0] cart_wmask
);
    // ---- mount capture, NOT reset-gated (see header) ----
    logic        mnt_pend = 1'b0;
    logic        mnt_ro;
    logic        mnt_typed;
    logic [24:0] mnt_bytes;
    logic        mnt_ack;
    always @(posedge clk) begin
        if (img_mounted) begin
            mnt_pend  <= 1'b1;
            mnt_ro    <= img_readonly;
            mnt_typed <= typed_mode;
            mnt_bytes <= (img_size >= {39'd0, CART_BYTES}) ? CART_BYTES
                                                           : img_size[24:0];
        end else if (mnt_ack) begin
            mnt_pend <= 1'b0;
        end
    end

    // ---- dirty map: 1 bit per 512-byte sector (8192 max) ----
    logic dmap [0:8191];
    initial begin
        integer i;
        for (i = 0; i < 4096; i = i + 1) dmap[i] = 1'b0;
        for (i = 4096; i < 8192; i = i + 1) dmap[i] = 1'b0;
    end

    wire        snoop_hit = wr_snoop
                          && (wr_snoop_addr >= CART_BASE)
                          && (wr_snoop_addr <  CART_BASE + CART_BYTES);
    wire [13:0] snoop_sec14 = wr_snoop_addr[22:9] - CART_BASE[22:9];
    wire [12:0] snoop_sec   = snoop_sec14[12:0];

    logic        clr_req;                 // FSM wants to clear scan_idx
    logic [12:0] scan_idx;
    logic        scan_q;
    wire         clr_done = clr_req & ~snoop_hit;   // snoop set wins the port

    always @(posedge clk) begin
        if (snoop_hit)    dmap[snoop_sec] <= 1'b1;
        else if (clr_req) dmap[scan_idx]  <= 1'b0;
        scan_q <= dmap[scan_idx];
    end

    // ---- sector buffer for save prefetch ----
    logic [9:0] pf_idx;                   // 0..511 (declared before use)
    logic [7:0] secbuf [0:511];
    logic [7:0] din_q;
    always @(posedge clk) begin
        if (bk_ack) secbuf[pf_idx] <= bk_rdata;
        din_q <= secbuf[sd_buff_addr[8:0]];
    end
    assign sd_buff_din = din_q;

    // ---- main FSM ----
    typedef enum logic [3:0] {
        S_IDLE,
        S_RST_REQ, S_RST_WAIT, S_RST_XFER,
        S_WIPE,
        S_SCAN_SET, S_SCAN_CHK, S_CLR,
        S_PF_REQ, S_PF_WAIT,
        S_WR_REQ, S_WR_WAIT, S_WR_XFER
    } state_t;
    state_t state;

    logic        mounted, ro_r, typed_r;
    logic [15:0] nsec;                    // sectors in the image window
    logic [15:0] rst_sec, rem_sec;        // restore progress
    logic [24:0] wipe_addr;               // eject wipe progress
    logic  [5:0] chunk_blk;
    logic [12:0] save_sec;
    logic        save_arm, sweep_again;
    logic [31:0] save_cnt;

    // ---- typed .crt parse state (X16 CARTRIDGE header) ----
    //   header layout: magic[16] version[16] desc[32] author[32] copyright[32]
    //   prg_version[32] reserved[96] bank_info[224]  = 480 bytes, then 16 KB
    //   data blocks in bank order ONLY for banks with bank_info[t] odd (has
    //   data: ROM/INIT_RAM/INIT_NVRAM).  writable = bank_info[t] > 1.
    logic [7:0]  plist [0:223];            // ordered list of present (has-data) banks
    logic [7:0]  pcnt;                     // number of present banks
    logic [13:0] dp_off;                   // byte within the current data bank (0..16383)
    logic [7:0]  dp_pbi;                   // index into plist for the streaming data
    logic        magic_ok;                 // "CX16 CARTRIDGE\r\n" verified
    // file byte offset of the current restore byte (linear = the old mapping)
    wire  [24:0] foff = {rst_sec[15:0], 9'd0} + {11'd0, sd_buff_addr};

    function [7:0] magic_byte(input [3:0] i);
        case (i)
            4'd0: magic_byte = 8'h43; 4'd1: magic_byte = 8'h58;   // C X
            4'd2: magic_byte = 8'h31; 4'd3: magic_byte = 8'h36;   // 1 6
            4'd4: magic_byte = 8'h20; 4'd5: magic_byte = 8'h43;   // _ C
            4'd6: magic_byte = 8'h41; 4'd7: magic_byte = 8'h52;   // A R
            4'd8: magic_byte = 8'h54; 4'd9: magic_byte = 8'h52;   // T R
            4'd10: magic_byte = 8'h49; 4'd11: magic_byte = 8'h44; // I D
            4'd12: magic_byte = 8'h47; 4'd13: magic_byte = 8'h45; // G E
            4'd14: magic_byte = 8'h0D; 4'd15: magic_byte = 8'h0A; // CR LF
            default: magic_byte = 8'h00;
        endcase
    endfunction

    assign bk_addr     = CART_BASE + {3'd0, save_sec, pf_idx[8:0]};
    // held through restore AND eject-wipe -> the machine reboots into the
    // newly mounted cart, or cartless after an unmount
    assign restoring = mnt_pend
                     || (state == S_RST_REQ) || (state == S_RST_WAIT)
                     || (state == S_RST_XFER) || (state == S_WIPE);

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_IDLE; mounted <= 1'b0; ro_r <= 1'b0;
            nsec <= 16'd0; rst_sec <= 16'd0; rem_sec <= 16'd0;
            chunk_blk <= 6'd0; save_sec <= 13'd0; pf_idx <= 10'd0;
            wipe_addr <= 25'd0;
            save_arm <= 1'b0; sweep_again <= 1'b0; save_cnt <= 32'd0;
            sd_lba <= 32'd0; sd_blk_cnt <= 6'd0; sd_rd <= 1'b0; sd_wr <= 1'b0;
            rst_ld_wr <= 1'b0; rst_ld_addr <= 25'd0; rst_ld_data <= 8'd0;
            bk_rd <= 1'b0; clr_req <= 1'b0; mnt_ack <= 1'b0; scan_idx <= 13'd0;
            cart_wmask <= 256'd0; pcnt <= 8'd0; dp_off <= 14'd0;
            dp_pbi <= 8'd0; magic_ok <= 1'b1; typed_r <= 1'b0;
        end else begin
            rst_ld_wr <= 1'b0;            // 1-cycle pulses
            bk_rd     <= 1'b0;
            mnt_ack   <= 1'b0;

            // any cart write (re)arms the debounced save
            if (snoop_hit) begin
                save_arm <= 1'b1;
                save_cnt <= SAVE_DELAY;
                if (state != S_IDLE && state != S_SCAN_SET
                    && state != S_SCAN_CHK) sweep_again <= 1'b1;
            end else if (save_arm && save_cnt != 0) begin
                save_cnt <= save_cnt - 32'd1;
            end

            if (mnt_pend && !mnt_ack && state == S_IDLE) begin
                mnt_ack  <= 1'b1;
                mounted  <= (mnt_bytes >= 25'd512);
                // typed .crt: read-only on disk (RAM/NVRAM writes stay volatile
                // in SDRAM, like the emulator).  raw all-RAM: honor img_readonly
                // so the dirty-sector save-back persists writes to the image.
                ro_r     <= mnt_typed ? 1'b1 : mnt_ro;
                typed_r  <= mnt_typed;
                nsec     <= mnt_bytes[24:9];
                save_arm <= 1'b0;
                if (mnt_bytes >= 25'd512) begin
                    rst_sec    <= 16'd0;
                    rem_sec    <= mnt_bytes[24:9];
                    // typed: header parse fills the mask; raw: every cart bank
                    // (32-255) is writable RAM
                    cart_wmask <= mnt_typed ? 256'd0 : {{224{1'b1}}, 32'd0};
                    pcnt       <= 8'd0;
                    dp_off     <= 14'd0;
                    dp_pbi     <= 8'd0;
                    magic_ok   <= 1'b1;
                    state      <= S_RST_REQ;
                end else begin
                    // unmount (OSD Backspace) = EJECT: zero the whole cart
                    // region so the ROM's boot-time cart detect finds
                    // nothing, then `restoring` reboots the machine cartless
                    cart_wmask <= 256'd0;     // no cart -> no writable banks
                    wipe_addr  <= 25'd0;
                    state      <= S_WIPE;
                end
            end

            case (state)
                S_IDLE: begin
                    if (save_arm && save_cnt == 0 && mounted && !ro_r
                        && nsec != 0) begin
                        save_arm <= 1'b0;
                        scan_idx <= 13'd0;
                        state    <= S_SCAN_SET;
                    end
                end

                // ============ restore: 16 KB chunks into the ld port ======
                S_RST_REQ: begin
                    chunk_blk  <= (rem_sec > 16'd32) ? 6'd31 : (rem_sec[5:0] - 6'd1);
                    sd_blk_cnt <= (rem_sec > 16'd32) ? 6'd31 : (rem_sec[5:0] - 6'd1);
                    sd_lba     <= {16'd0, rst_sec};
                    sd_rd      <= 1'b1;
                    state      <= S_RST_WAIT;
                end
                S_RST_WAIT: if (sd_ack) begin
                    sd_rd <= 1'b0;
                    state <= S_RST_XFER;
                end
                S_RST_XFER: begin
                    if (sd_buff_wr && typed_r) begin
                        // ============ typed .crt: parse header, place sparse =====
                        // magic "CX16 CARTRIDGE\r\n" at bytes 0..15
                        if (foff < 25'd16
                            && sd_buff_dout != magic_byte(foff[3:0]))
                            magic_ok <= 1'b0;
                        // bank_info[224] at bytes 256..479 (t == foff[7:0]):
                        // build the writable mask + the ordered present-bank list
                        if (foff >= 25'd256 && foff < 25'd480 && magic_ok) begin
                            if (sd_buff_dout > 8'd1)          // RAM/NVRAM -> writable
                                cart_wmask[8'd32 + foff[7:0]] <= 1'b1;
                            if (sd_buff_dout[0]) begin        // has a 16 KB data block
                                plist[pcnt] <= foff[7:0];
                                pcnt        <= pcnt + 8'd1;
                            end
                        end
                        // bank data (bytes 480+) -> its real bank address, walking
                        // the present-bank list (sparse gaps cost nothing)
                        if (foff >= 25'd480 && magic_ok && dp_pbi < pcnt) begin
                            rst_ld_wr   <= 1'b1;
                            rst_ld_data <= sd_buff_dout;
                            rst_ld_addr <= CART_BASE + {3'd0, plist[dp_pbi], dp_off};
                            if (dp_off == 14'd16383) begin
                                dp_off <= 14'd0;
                                dp_pbi <= dp_pbi + 8'd1;
                            end else dp_off <= dp_off + 14'd1;
                        end
                    end else if (sd_buff_wr && !typed_r) begin
                        // ============ raw all-RAM: linear file offset -> cart ====
                        // file offset 0 = bank 32 offset 0 (same as "Load Cart")
                        rst_ld_wr   <= 1'b1;
                        rst_ld_data <= sd_buff_dout;
                        rst_ld_addr <= CART_BASE + foff;
                    end
                    if (!sd_ack) begin
                        rst_sec <= rst_sec + {10'd0, chunk_blk} + 16'd1;
                        rem_sec <= rem_sec - {10'd0, chunk_blk} - 16'd1;
                        state   <= (rem_sec > {10'd0, chunk_blk} + 16'd1)
                                   ? S_RST_REQ : S_IDLE;
                    end
                end

                S_WIPE: if (!ld_busy) begin
                    rst_ld_wr   <= 1'b1;
                    rst_ld_data <= 8'h00;
                    rst_ld_addr <= CART_BASE + wipe_addr;
                    if (wipe_addr == CART_BYTES - 25'd1) state <= S_IDLE;
                    wipe_addr   <= wipe_addr + 25'd1;
                end

                // ============ save sweep: only dirty sectors ==============
                S_SCAN_SET: state <= S_SCAN_CHK;      // scan_q settles
                S_SCAN_CHK: begin
                    if (scan_q) begin
                        clr_req <= 1'b1;
                        state   <= S_CLR;
                    end else if (scan_idx == 13'(nsec - 16'd1)
                                 || scan_idx == 13'd8191) begin
                        if (sweep_again) begin        // writes landed mid-sweep
                            sweep_again <= 1'b0;
                            scan_idx    <= 13'd0;
                            state       <= S_SCAN_SET;
                        end else state <= S_IDLE;
                    end else begin
                        scan_idx <= scan_idx + 13'd1;
                        state    <= S_SCAN_SET;
                    end
                end
                S_CLR: if (clr_done) begin            // clear BEFORE reading
                    clr_req  <= 1'b0;
                    save_sec <= scan_idx;
                    pf_idx   <= 10'd0;
                    state    <= S_PF_REQ;
                end
                S_PF_REQ: begin
                    bk_rd <= 1'b1;
                    state <= S_PF_WAIT;
                end
                S_PF_WAIT: if (bk_ack) begin
                    // secbuf[pf_idx] captured by the buffer block above
                    if (pf_idx == 10'd511) begin
                        pf_idx <= 10'd0;
                        state  <= S_WR_REQ;
                    end else begin
                        pf_idx <= pf_idx + 10'd1;
                        state  <= S_PF_REQ;
                    end
                end
                S_WR_REQ: begin
                    sd_lba     <= {19'd0, save_sec};
                    sd_blk_cnt <= 6'd0;
                    sd_wr      <= 1'b1;
                    state      <= S_WR_WAIT;
                end
                S_WR_WAIT: if (sd_ack) begin
                    sd_wr <= 1'b0;
                    state <= S_WR_XFER;
                end
                S_WR_XFER: if (!sd_ack) begin
                    if (scan_idx != 13'(nsec - 16'd1) && scan_idx != 13'd8191) begin
                        scan_idx <= scan_idx + 13'd1;      // next sector
                        state    <= S_SCAN_SET;
                    end else if (sweep_again) begin        // writes mid-sweep
                        sweep_again <= 1'b0;
                        scan_idx    <= 13'd0;
                        state       <= S_SCAN_SET;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
