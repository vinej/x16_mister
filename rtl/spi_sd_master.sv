// ============================================================================
// spi_sd_master -- cpu_clk-domain SPI master for the X16 guest SD card,
// presenting VERA's $9F3E (data) / $9F3F (ctrl) register semantics.
//
// WRITE FIFO (the BUG3 fix): writes to $9F3E are QUEUED in a 512-byte FIFO
// instead of stalling the CPU.  Rationale: the r65c02 -- like every classic
// 6502 -- IGNORES RDY during write cycles, so a stall can never protect a
// write.  The R49 ROM's fast-write path (back-to-back STA $9F3E every ~13
// cycles with NO busy polling; legal on real VERA where a byte completes in
// 640 ns) would otherwise lose bytes at this master's 2 us/byte pace and
// shred the sector (reproduced in sim/tb_write.v at the original 8 us/byte).
//
// EDGE-TRIGGERED WRITE CAPTURE (the byte-doubling fix): writes are detected
// on the RISING EDGE of the raw bus condition (addr match & ~rwn), NOT on a
// cpu_rdy-qualified level strobe.  Image forensics (rom/sdcard_corrupted.img)
// showed the synthesized level-strobe sometimes committed one STA twice
// (doubled bytes in FSInfo/FAT/dir writes) -- and a cpu_rdy-gated strobe can
// also MISS a write outright when another stall source coincides with the
// (RDY-ignoring) write cycle.  One STA = one address-match edge = exactly
// one byte, however long or glitchy the strobe: consecutive STAs are always
// separated by opcode fetches, so edges cannot merge.
//
// The $9F3F busy bit (bit 7) covers "shifting OR queue not empty", so the
// ROM's spi_write / wait_ready polls still drain everything before deselect.
// Reads of $9F3E stall (reads DO honor RDY) until fully drained and return
// the last received byte; a read with auto-tx enabled starts the next 0xFF
// transfer, as on VERA.  FIFO depth 512 covers the worst burst (512 data
// bytes; the two CRC bytes go through polled spi_write, which drains first).
//
// $9F3F write: bit2 = auto-tx, bit1 = slow (accepted, ignored -- sck is
// fixed at cpu_clk/(2*(M_HALF+1))), bit0 = select.
//
// SCK SPEED + LATE MISO SAMPLING (jyv 2026-07-07, the "PCM video stutters"
// fix): sck raised from cpu_clk/8 (1 MHz, ~8 us/byte, ~110 KB/s ceiling
// with the CPU ~90% stalled) to cpu_clk/2 (4 MHz, 2 us/byte, ~400 KB/s)
// so streaming demos (8-Bit-Guy video player: continuous MACPTR reads
// feeding the VERA PCM FIFO) stop underrunning.  At 4 MHz the old scheme
// -- sample MISO on the sck RISING edge through a 2-FF sync -- reads the
// bit one edge early (sync latency 125-250 ns >= the 125 ns half-period),
// so MISO is now sampled on the *next falling edge* instead: SPI mode-0
// slaves hold DO stable from one falling edge to the next, and the
// sd_card slave's own falling-edge reaction latency (2-FF sck sync @
// 100 MHz + FSM, ~30-40 ns) guarantees the old bit is still present when
// we sample.  Deterministic at any M_HALF; the integrator keeps a single
// capture FF on MISO (see x16.sv).
// ============================================================================
module spi_sd_master #(
    parameter [2:0] M_HALF = 3'd0          // sck half-period-1 (cpu_clk/2 = 4 MHz)
)(
    input            clk,                  // cpu_clk (same domain as sd_card)
    input            rst_n,

    // CPU register interface (raw bus for writes, committed strobe for reads)
    input            rd_data,              // committed CPU read of $9F3E (auto-tx)
    input      [7:0] cpu_do,
    input            acc_data,             // CPU address == $9F3E
    input            acc_ctrl,             // CPU address == $9F3F
    input            cpu_rwn,              // 1 = read cycle
    output           stall,                // gate into cpu_rdy (holds $9F3E reads)

    output     [7:0] data_q,               // $9F3E read data (last received byte)
    output     [7:0] status_q,             // $9F3F read data {busy,0000,autotx,slow,sel}

    // SPI pins (sd_card side, same clock domain)
    output reg       sck,
    output reg       mosi,
    output reg       sel,                  // 1 = selected (integrator inverts for ss)
    input            miso
);

    // ---- write FIFO: 512 x 8, synchronous read with a settle guard ----
    reg [7:0] wf_mem [0:511];
    reg [8:0] wf_wr, wf_rd;
    reg [7:0] wf_q;                        // registered head entry
    reg [1:0] wq_wait;                     // head-valid settle counter
    wire      wf_empty = (wf_wr == wf_rd);
    wire      wf_full  = (wf_wr + 9'd1 == wf_rd);

    // ---- shifter (SPI mode 0) ----
    reg [7:0] m_tx, m_rx;
    reg [3:0] m_ec;                        // edge counter 0..15 (8 sck periods)
    reg [2:0] m_div;
    reg       m_busy, m_autotx, m_slow;

    // ---- raw-bus write-edge detection (one CPU STA = one edge = one byte) ----
    reg  wr_data_d, wr_ctrl_d;
    wire wr_data_raw = acc_data & ~cpu_rwn;
    wire wr_ctrl_raw = acc_ctrl & ~cpu_rwn;
    wire wr_data_edge = wr_data_raw & ~wr_data_d;
    wire wr_ctrl_edge = wr_ctrl_raw & ~wr_ctrl_d;

    wire busy_ext = m_busy | ~wf_empty;
    // reads stall until everything drains; writes never stall (they enqueue --
    // a stalled write would be LOST anyway, since the CPU ignores RDY on
    // write cycles: the FIFO depth is the protection, not a stall).
    assign stall    = acc_data & cpu_rwn & busy_ext;
    assign data_q   = m_rx;
    assign status_q = {busy_ext, 4'b0000, m_autotx, m_slow, sel};

    // pop the queue head into the shifter when idle (wq_wait guards the one
    // cycle wf_q needs to reflect a just-pushed/just-advanced head)
    wire pop = ~m_busy & ~wf_empty & (wq_wait == 2'd0);
    // auto-tx: a committed $9F3E read starts the next 0xFF transfer.  The
    // read stall guarantees the queue is empty and the shifter idle here.
    wire start_ff = rd_data & m_autotx;

    always @(posedge clk) begin
        if (wr_data_edge) wf_mem[wf_wr] <= cpu_do;
        wf_q <= wf_mem[wf_rd];             // continuous head fetch
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wf_wr <= 9'd0; wf_rd <= 9'd0; wq_wait <= 2'd0;
            wr_data_d <= 1'b0; wr_ctrl_d <= 1'b0;
            m_busy <= 1'b0; sck <= 1'b0; mosi <= 1'b1; sel <= 1'b0;
            m_ec <= 4'd0; m_div <= 3'd0; m_rx <= 8'h00; m_tx <= 8'h00;
            m_autotx <= 1'b0; m_slow <= 1'b0;
        end else begin
            wr_data_d <= wr_data_raw;
            wr_ctrl_d <= wr_ctrl_raw;

            if (wr_ctrl_edge) begin
                m_autotx <= cpu_do[2];
                m_slow   <= cpu_do[1];
                sel      <= cpu_do[0];
            end

            if (wr_data_edge & ~wf_full) wf_wr <= wf_wr + 9'd1;  // enqueue once per STA

            // wf_q settles one cycle after any push or pop moves/exposes the head
            if (wr_data_edge | pop)   wq_wait <= 2'd2;
            else if (wq_wait != 2'd0) wq_wait <= wq_wait - 2'd1;

            if (pop) begin                 // start transfer of the queue head
                m_tx  <= wf_q;
                mosi  <= wf_q[7];
                wf_rd <= wf_rd + 9'd1;
                m_busy <= 1'b1; m_ec <= 4'd0; m_div <= 3'd0; sck <= 1'b0;
            end else if (start_ff) begin   // auto-tx read: shift out 0xFF
                m_tx  <= 8'hFF;
                mosi  <= 1'b1;
                m_busy <= 1'b1; m_ec <= 4'd0; m_div <= 3'd0; sck <= 1'b0;
            end else if (m_busy) begin
                if (m_div == M_HALF) begin
                    m_div <= 3'd0;
                    sck   <= ~sck;         // 16 edges: falling = sample + shift
                    if (sck) begin         // falling edge (1->0): LATE-sample the
                                           // bit of the half-period just ending
                                           // (slave holds it past this edge),
                                           // then advance MOSI for the next bit
                        m_rx  <= {m_rx[6:0], miso};
                        m_tx  <= {m_tx[6:0], 1'b0};
                        mosi  <= m_tx[6];
                    end
                    m_ec <= m_ec + 4'd1;
                    if (m_ec == 4'd15) m_busy <= 1'b0;
                end else m_div <= m_div + 3'd1;
            end
        end
    end

endmodule
