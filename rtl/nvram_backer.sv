// ============================================================================
// nvram_backer.sv -- RTC NVRAM persistence via hps_io virtual disk slot 1
// (jyv 2026-07-07; reworked same day after Quartus error 276001: a RAM with
// write ports in two clock domains is not synthesizable).
//
// The X16 KERNAL stores its settings (screen mode etc., checksum at $5F) in
// the RTC's battery-backed SRAM ($07-$5F = 89 bytes in rtc_x16's mem[]).
// Without a battery, they vanished every power cycle.  This module backs
// the store with a 512-byte disk image mounted on OSD slot S1 ("NVRAM"):
// restore on mount, autosave SAVE_DELAY after the last CPU write.
//
// Clocking architecture -- rtc_x16's mem[] stays strictly single-clock
// (cpu_clk, single muxed write port); this module owns ALL cross-domain
// plumbing with two tiny dual-clock simple-DP RAMs (1 write port + 1 read
// port each, the inferable kind):
//
//   rbuf   (restore buffer): written @ clk (100 MHz) from the hps_io byte
//          stream during the block read; read @ cpu_clk by the walker.
//   shadow (save copy):      written @ cpu_clk -- mirrors every I2C-FSM
//          write via the rtc's nv_snoop_* port AND every restore write --
//          read @ clk to serve sd_buff_din during the block write-back.
//
//   walker (cpu_clk FSM):    after a completed block read (toggle
//          handshake), walks rbuf[0..88] into rtc mem through nv_we/addr/
//          wdata.  The rtc's I2C-FSM write wins any collision cycle
//          (nv_snoop_we high): the walker just retries that byte.
//
// hps_io byte-buffer contract ("2-PORT altsyncram"): during a core->HPS
// transfer sd_buff_addr is presented and sd_buff_din is expected registered
// one clk later -- the shadow read port provides exactly that.  Block
// bytes 89-511 read back as $00.
// ============================================================================
module nvram_backer #(
    parameter integer SAVE_DELAY = 100_000_000   // 1 s @ 100 MHz
) (
    input  logic        clk,             // hps_io clk_sys (sdram_clk, 100 MHz)
    input  logic        cpu_clk,         // rtc_x16 domain (8 MHz)
    input  logic        reset_n,         // stretched system reset (quasi-static)

    // hps_io virtual disk slot 1 (clk domain)
    input  logic        img_mounted,     // pulse; this slot's bit
    input  logic        img_readonly,    // valid during the mount pulse
    input  logic [63:0] img_size,        // valid during the mount pulse
    output logic [31:0] sd_lba,
    output logic        sd_rd,
    output logic        sd_wr,
    input  logic        sd_ack,
    input  logic  [8:0] sd_buff_addr,
    input  logic  [7:0] sd_buff_dout,
    output logic  [7:0] sd_buff_din,
    input  logic        sd_buff_wr,

    // rtc_x16 nv_* port (cpu_clk domain)
    output logic        nv_we,
    output logic  [6:0] nv_addr,
    output logic  [7:0] nv_wdata,
    input  logic        nv_snoop_we,
    input  logic  [6:0] nv_snoop_addr,
    input  logic  [7:0] nv_snoop_data,
    input  logic        nv_dirty_toggle
);
    localparam int NV_BYTES = 89;

    assign sd_lba = 32'd0;               // always block 0

    // ================= dual-clock buffers =================

    // restore buffer: write @ clk, read @ cpu_clk
    reg [7:0] rbuf [0:127];
    reg [7:0] rbuf_q;

    // save shadow: write @ cpu_clk, read @ clk.  Initial contents match
    // rtc_x16 mem[] so a pre-mount save baseline is coherent (saves can
    // only happen after a mount+restore anyway).
    reg [7:0] shadow [0:127];
    initial begin
        integer i;
        for (i = 0; i < 128; i = i + 1) shadow[i] = 8'h00;
        shadow[88] = 8'hFF;
    end
    reg [7:0] sh_q;
    reg       din_zero_r;
    assign sd_buff_din = din_zero_r ? 8'h00 : sh_q;

    // ================= 100 MHz side: slot protocol =================
    typedef enum logic [2:0] {
        S_IDLE, S_RD_WAIT, S_RD_XFER, S_WR_WAIT, S_WR_XFER
    } state_t;
    state_t state;

    logic        mounted, ro_r;
    logic [2:0]  dirty_s;
    logic        save_arm;
    logic [31:0] save_cnt;
    logic        go_t;                   // toggles: "block read done, walk it"
    logic        done_t;                 // cpu-side ack toggle
    logic [1:0]  done_s;                 // done_t synced into clk
    wire         restoring = (go_t != done_s[1]);

    // Mount capture, NOT reset-gated (jyv 2026-07-07 evening, HW-found):
    // at core start MiSTer Main auto-remounts the image while the ROM
    // download still holds the system reset -- the FSM below is in reset
    // and missed the img_mounted pulse, so the boot-time restore never ran
    // (the KERNAL then rewrote factory defaults every cold boot).  Latch
    // the mount here and let the FSM consume it after reset release; the
    // restore then completes ~1 ms into the run, long before the KERNAL's
    // cint reads the NVRAM.  (Same idea as x16.sv's vsd_sel for slot 0.)
    logic mnt_pend = 1'b0;               // power-up 0; survives FSM reset
    logic mnt_sz, mnt_ro, mnt_ack;
    always @(posedge clk) begin
        if (img_mounted) begin
            mnt_pend <= 1'b1;
            mnt_sz   <= (img_size != 64'd0);
            mnt_ro   <= img_readonly;
        end else if (mnt_ack) begin
            mnt_pend <= 1'b0;
        end
    end

    always @(posedge clk)
        if (state == S_RD_XFER && sd_buff_wr && sd_buff_addr < NV_BYTES)
            rbuf[sd_buff_addr[6:0]] <= sd_buff_dout;

    always @(posedge clk) begin
        sh_q       <= shadow[sd_buff_addr[6:0]];
        din_zero_r <= (sd_buff_addr >= NV_BYTES);
        done_s     <= {done_s[0], done_t};
    end

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state    <= S_IDLE;
            mounted  <= 1'b0;
            ro_r     <= 1'b0;
            dirty_s  <= 3'b000;
            save_arm <= 1'b0;
            save_cnt <= 32'd0;
            sd_rd    <= 1'b0;
            sd_wr    <= 1'b0;
            go_t     <= 1'b0;
            mnt_ack  <= 1'b0;
        end else begin
            mnt_ack <= 1'b0;
            dirty_s <= {dirty_s[1:0], nv_dirty_toggle};

            // CPU wrote NVRAM: (re)start the settle countdown
            if (dirty_s[2] != dirty_s[1]) begin
                save_arm <= 1'b1;
                save_cnt <= SAVE_DELAY;
            end else if (save_arm && save_cnt != 0) begin
                save_cnt <= save_cnt - 32'd1;
            end

            if (mnt_pend && !mnt_ack && state == S_IDLE) begin
                mnt_ack  <= 1'b1;
                mounted  <= mnt_sz;
                ro_r     <= mnt_ro;
                save_arm <= 1'b0;
                if (mnt_sz) begin
                    sd_rd <= 1'b1;
                    state <= S_RD_WAIT;
                end
            end

            case (state)
                S_IDLE: begin
                    if (save_arm && save_cnt == 0 && mounted && !ro_r
                        && !restoring) begin
                        save_arm <= 1'b0;
                        sd_wr    <= 1'b1;
                        state    <= S_WR_WAIT;
                    end
                end

                S_RD_WAIT: if (sd_ack) begin
                    sd_rd <= 1'b0;
                    state <= S_RD_XFER;
                end

                S_RD_XFER: if (!sd_ack) begin
                    go_t     <= ~go_t;    // hand the block to the walker
                    save_arm <= 1'b0;     // a restore is not a dirty event
                    state    <= S_IDLE;
                end

                S_WR_WAIT: if (sd_ack) begin
                    sd_wr <= 1'b0;
                    state <= S_WR_XFER;
                end

                S_WR_XFER: if (!sd_ack) state <= S_IDLE;

                default: state <= S_IDLE;
            endcase
        end
    end

    // ================= cpu_clk side: restore walker + shadow =================
    logic [1:0] go_s;
    logic       walking, walk_ph;        // ph 0 = rbuf_q settle, 1 = write
    logic [6:0] walk_addr;

    always @(posedge cpu_clk) begin
        rbuf_q <= rbuf[walk_addr];
        go_s   <= {go_s[0], go_t};
    end

    // walker write is accepted only on cycles the rtc's own I2C FSM isn't
    // writing (rtc muxes mem_we first)
    assign nv_we    = walking & walk_ph & ~nv_snoop_we;
    assign nv_addr  = walk_addr;
    assign nv_wdata = rbuf_q;

    always_ff @(posedge cpu_clk or negedge reset_n) begin
        if (!reset_n) begin
            walking   <= 1'b0;
            walk_ph   <= 1'b0;
            walk_addr <= 7'd0;
            done_t    <= 1'b0;
        end else if (!walking) begin
            if (go_s[1] != done_t) begin
                walking   <= 1'b1;
                walk_ph   <= 1'b0;
                walk_addr <= 7'd0;
            end
        end else if (!walk_ph) begin
            walk_ph <= 1'b1;             // rbuf_q now valid for walk_addr
        end else if (~nv_snoop_we) begin // byte written this cycle
            if (walk_addr == 7'(NV_BYTES-1)) begin
                walking <= 1'b0;
                done_t  <= go_s[1];
            end else begin
                walk_addr <= walk_addr + 7'd1;
                walk_ph   <= 1'b0;
            end
        end
    end

    // shadow stays coherent with rtc mem[]: CPU I2C writes (snoop, wins)
    // and accepted walker writes
    wire        sh_we   = nv_snoop_we | nv_we;
    wire  [6:0] sh_addr = nv_snoop_we ? nv_snoop_addr : walk_addr;
    wire  [7:0] sh_data = nv_snoop_we ? nv_snoop_data : rbuf_q;
    always @(posedge cpu_clk) if (sh_we) shadow[sh_addr] <= sh_data;

endmodule
