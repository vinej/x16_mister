// ============================================================================
// cart_backer.sv -- cart RAM persistence via hps_io virtual disk slot 2
// (jyv 2026-07-07, the last roadmap item).
//
// ROM banks 32-255 live in SDRAM at CART_BASE and are READ/WRITE (cart
// RAM/flash semantics), but were volatile.  This module backs them with a
// disk image on OSD slot SC2 ("Mount Cart RAM"):
//
//   * RESTORE on mount: the image streams into SDRAM through the existing
//     loader port (muxed in x16.sv with the F2 ioctl cart loader -- they
//     never run concurrently).  Reads use 16 KB multi-block transfers
//     (sd_blk_cnt=31), so a full 3.5 MB image lands in well under a second.
//     The image uses the SAME linear mapping as "Load Cart" (file offset 0
//     = bank 32 offset 0), so any cart file works mounted or loaded.
//   * SAVE-BACK: ext_ram_sdram pulses wr_snoop for every committed CPU
//     write; writes inside the image window mark a 512-byte sector dirty
//     in an 8192-bit map.  SAVE_DELAY after the last write, a sweep saves
//     ONLY dirty sectors: clear bit -> prefetch the sector through the
//     lowest-priority bk_* read port -> sd_wr the block.  A CPU write
//     landing mid-save re-dirties its sector and a follow-up sweep gets it
//     (clear-before-read ordering makes that lossless).
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
    output logic        restoring
);
    // ---- mount capture, NOT reset-gated (see header) ----
    logic        mnt_pend = 1'b0;
    logic        mnt_ro;
    logic [24:0] mnt_bytes;
    logic        mnt_ack;
    always @(posedge clk) begin
        if (img_mounted) begin
            mnt_pend  <= 1'b1;
            mnt_ro    <= img_readonly;
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

    logic        mounted, ro_r;
    logic [15:0] nsec;                    // sectors in the image window
    logic [15:0] rst_sec, rem_sec;        // restore progress
    logic [24:0] wipe_addr;               // eject wipe progress
    logic  [5:0] chunk_blk;
    logic [12:0] save_sec;
    logic        save_arm, sweep_again;
    logic [31:0] save_cnt;

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
                ro_r     <= mnt_ro;
                nsec     <= mnt_bytes[24:9];
                save_arm <= 1'b0;
                if (mnt_bytes >= 25'd512) begin
                    rst_sec <= 16'd0;
                    rem_sec <= mnt_bytes[24:9];
                    state   <= S_RST_REQ;
                end else begin
                    // unmount (OSD Backspace) = EJECT: zero the whole cart
                    // region so the ROM's boot-time cart detect finds
                    // nothing, then `restoring` reboots the machine cartless
                    wipe_addr <= 25'd0;
                    state     <= S_WIPE;
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
                    if (sd_buff_wr) begin
                        rst_ld_wr   <= 1'b1;
                        rst_ld_data <= sd_buff_dout;
                        rst_ld_addr <= CART_BASE
                                     + {rst_sec[15:0], 9'd0}
                                     + {11'd0, sd_buff_addr};
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
