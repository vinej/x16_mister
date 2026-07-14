// ============================================================================
// spi_sd_master100 -- guest-SD SPI master in the hps/sd_card 100 MHz domain
// (jyv 2026-07-07, third act of the SD-speed saga).
//
// History: the original cpu_clk master ran sck at 1 MHz (~110 KB/s, CPU 90%
// stalled) and starved streaming demos (PCM video stutter).  Raising it to
// 4 MHz did not boot: the miso return is a cpu_clk<->sdram_clk crossing that
// x16.sdc false-paths (asynchronous clock groups), so its routing delay is
// unbounded and the 4 MHz capture window (~85 ns) is unenforceable.  2 MHz
// was safe (~295 ns slack) but at ~235 KB/s still underran the demo.
//
// THIS module removes the problem class entirely: the shifter, write FIFO
// and SPI pins all live in the SAME 100 MHz domain as sys/sd_card.sv, so
// every SPI path is ordinary single-clock logic that Quartus fully times.
// sck = 100 MHz / 8 = 12.5 MHz -- real VERA's fast SPI speed, 640 ns/byte.
// The CPU's fast-read loop (~1.6 us/byte) is now the bottleneck, exactly
// like real hardware: reads never stall, effective MACPTR rate ~600 KB/s.
//
// What crosses clocks instead is the REGISTER interface, where events are
// microseconds apart -- toggle handshakes with stable payloads:
//   cpu -> 100M:  dat_tgl/dat_pay   one $9F3E write  (>= ~1.6 us apart)
//                 ctl_tgl/ctl_pay   one $9F3F write
//                 rd_tgl            one committed $9F3E read (auto-tx)
//   100M -> cpu:  busy_ext 2-FF synced; m_rx/sel quasi-static when idle
// A cpu-side `pend` covers the toggle->busy_sync latency (same idea as the
// VERA tx_pending fix), so the busy bit is never falsely 0 between a CPU
// access and the transfer start.
//
// Register semantics are identical to rtl/spi_sd_master.sv (VERA $9F3E/3F):
// writes enqueue (512-byte FIFO -- the BUG3 edge-per-STA capture is kept on
// the cpu side), reads stall until drained and return the last received
// byte, a read with auto-tx starts the next $FF transfer, $9F3F busy covers
// shifting-or-queued.
// ============================================================================
module spi_sd_master100 #(
    parameter [3:0] M_HALF = 4'd3          // sck half-period-1 @100 MHz (12.5 MHz)
)(
    // ---- cpu_clk domain: raw register bus (same contract as before) ----
    input            clk_cpu,
    input            rst_cpu_n,
    input            rd_data,              // committed CPU read of $9F3E
    input      [7:0] cpu_do,
    input            acc_data,             // CPU address == $9F3E
    input            acc_ctrl,             // CPU address == $9F3F
    input            cpu_rwn,              // 1 = read cycle
    output           stall,                // gate into cpu_rdy ($9F3E reads)
    output     [7:0] data_q,               // $9F3E read data
    output     [7:0] status_q,             // $9F3F {busy,0000,autotx,slow,sel}

    // ---- 100 MHz domain: SPI pins, same clock as sd_card ----
    input            clk,
    input            rst_n,
    output reg       sck,
    output reg       mosi,
    output reg       sel,                  // 1 = selected (integrator inverts)
    input            miso
);

    // ====================================================================
    // cpu_clk side
    // ====================================================================
    reg  wr_data_d, wr_ctrl_d;
    wire wr_data_raw  = acc_data & ~cpu_rwn;
    wire wr_ctrl_raw  = acc_ctrl & ~cpu_rwn;
    wire wr_data_edge = wr_data_raw & ~wr_data_d;   // one STA = one edge
    wire wr_ctrl_edge = wr_ctrl_raw & ~wr_ctrl_d;

    reg        dat_tgl, ctl_tgl, rd_tgl;
    reg  [7:0] dat_pay, ctl_pay;
    reg        autotx_sh, slow_sh, sel_sh;          // cpu-side shadows
    reg        pend;                                // start requested, busy not yet visible
    reg  [1:0] busy_s;                              // busy_ext synced into cpu_clk
    wire       busy_ext;                            // 100M side (declared below)

    always @(posedge clk_cpu or negedge rst_cpu_n) begin
        if (!rst_cpu_n) begin
            wr_data_d <= 1'b0; wr_ctrl_d <= 1'b0;
            dat_tgl <= 1'b0; ctl_tgl <= 1'b0; rd_tgl <= 1'b0;
            dat_pay <= 8'h00; ctl_pay <= 8'h00;
            autotx_sh <= 1'b0; slow_sh <= 1'b0; sel_sh <= 1'b0;
            pend <= 1'b0; busy_s <= 2'b00;
        end else begin
            wr_data_d <= wr_data_raw;
            wr_ctrl_d <= wr_ctrl_raw;
            busy_s    <= {busy_s[0], busy_ext};

            if (wr_data_edge) begin
                dat_pay <= cpu_do;
                dat_tgl <= ~dat_tgl;
                pend    <= 1'b1;
            end else if (rd_data && autotx_sh) begin
                rd_tgl  <= ~rd_tgl;
                pend    <= 1'b1;
            end else if (busy_s[1]) begin
                pend    <= 1'b0;        // 100M side has taken over
            end

            if (wr_ctrl_edge) begin
                ctl_pay   <= cpu_do;
                ctl_tgl   <= ~ctl_tgl;
                autotx_sh <= cpu_do[2];
                slow_sh   <= cpu_do[1];
                sel_sh    <= cpu_do[0];
            end
        end
    end

    wire busy_cpu = pend | busy_s[1];
    // reads stall until everything drains; writes never stall (FIFO is the
    // protection -- the CPU ignores RDY on write cycles anyway)
    assign stall    = acc_data & cpu_rwn & busy_cpu;
    assign status_q = {busy_cpu, 4'b0000, autotx_sh, slow_sh, sel_sh};

    // ====================================================================
    // 100 MHz side
    // ====================================================================
    reg [2:0] dat_s, ctl_s, rd_s;
    always @(posedge clk) begin
        dat_s <= {dat_s[1:0], dat_tgl};
        ctl_s <= {ctl_s[1:0], ctl_tgl};
        rd_s  <= {rd_s[1:0],  rd_tgl};
    end
    wire dat_ev = dat_s[2] ^ dat_s[1];   // payloads are stable >1 us around
    wire ctl_ev = ctl_s[2] ^ ctl_s[1];   // their toggle -> safe to sample
    wire rd_ev  = rd_s[2]  ^ rd_s[1];

    // ---- write FIFO: 512 x 8, synchronous read with a settle guard ----
    reg [7:0] wf_mem [0:511];
    reg [8:0] wf_wr, wf_rd;
    reg [7:0] wf_q;
    reg [1:0] wq_wait;
    wire      wf_empty = (wf_wr == wf_rd);
    wire      wf_full  = (wf_wr + 9'd1 == wf_rd);

    // ---- shifter (SPI mode 0, late MISO sampling on falling edges) ----
    reg [7:0] m_tx, m_rx;
    // m_rx only changes while busy; the stall guarantees the CPU reads it
    // idle -> quasi-static across the domain boundary
    assign data_q = m_rx;
    reg [3:0] m_ec;
    reg [3:0] m_div;
    reg       m_busy, m_autotx;

    assign busy_ext = m_busy | ~wf_empty;

    wire pop      = ~m_busy & ~wf_empty & (wq_wait == 2'd0);
    wire start_ff = rd_ev & m_autotx;    // the cpu-side stall guarantees the
                                         // queue is empty and shifter idle

    always @(posedge clk) begin
        if (dat_ev) wf_mem[wf_wr] <= dat_pay;
        wf_q <= wf_mem[wf_rd];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wf_wr <= 9'd0; wf_rd <= 9'd0; wq_wait <= 2'd0;
            m_busy <= 1'b0; sck <= 1'b0; mosi <= 1'b1; sel <= 1'b0;
            m_ec <= 4'd0; m_div <= 4'd0; m_rx <= 8'h00; m_tx <= 8'h00;
            m_autotx <= 1'b0;
        end else begin
            if (ctl_ev) begin
                m_autotx <= ctl_pay[2];
                sel      <= ctl_pay[0];
            end

            if (dat_ev & ~wf_full) wf_wr <= wf_wr + 9'd1;

            if (dat_ev | pop)         wq_wait <= 2'd2;
            else if (wq_wait != 2'd0) wq_wait <= wq_wait - 2'd1;

            if (pop) begin
                m_tx  <= wf_q;
                mosi  <= wf_q[7];
                wf_rd <= wf_rd + 9'd1;
                m_busy <= 1'b1; m_ec <= 4'd0; m_div <= 4'd0; sck <= 1'b0;
            end else if (start_ff) begin
                m_tx  <= 8'hFF;
                mosi  <= 1'b1;
                m_busy <= 1'b1; m_ec <= 4'd0; m_div <= 4'd0; sck <= 1'b0;
            end else if (m_busy) begin
                if (m_div == M_HALF) begin
                    m_div <= 4'd0;
                    sck   <= ~sck;
                    if (sck) begin       // falling edge: late-sample MISO,
                                         // then advance MOSI
                        m_rx  <= {m_rx[6:0], miso};
                        m_tx  <= {m_tx[6:0], 1'b0};
                        mosi  <= m_tx[6];
                    end
                    m_ec <= m_ec + 4'd1;
                    if (m_ec == 4'd15) m_busy <= 1'b0;
                end else m_div <= m_div + 4'd1;
            end
        end
    end

endmodule
