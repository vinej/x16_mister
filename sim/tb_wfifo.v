`timescale 1ns/1ps
// ============================================================================
// tb_wfifo.v -- write-FIFO repeat-push hunt (65C816 branch).
//
// The P65C816 honors rdy on WRITES (RDY_IN is a global clock enable): when
// wf_hi stalls the CPU during a write cycle, the CPU freezes HOLDING
// cs/we/addr/data on the bus.  With the level-triggered
//     wpush = cs & we
// that held write is pushed into the 4-deep FIFO once per clock, every
// clock, while the drain pops ~1 per 3 cycles: wf_wr laps wf_rd (pending
// writes overwritten = LOST), wf_cnt climbs to 7 and wraps to 0 (FIFO
// declared empty with entries stranded).  The r65c02 never hits this: it
// blows through rdy on writes, so a write cycle lasts exactly one clock and
// the wf_hi stall always lands on the FOLLOWING fetch.
//
// PHASE 1 ('816 semantics): 6 writes on consecutive cycles, each held until
//   a posedge with ready=1 -- by write #4 wf_hi coincides with a held write.
//   Then read all 6 back.  Buggy RTL: lost writes + push count > 6.
// PHASE 2 (r65c02 semantics): writes blow through rdy, spaced by 3
//   rdy-honoring dead cycles (sta abs shape).  Must pass before AND after
//   the fix (guards the fix's r65c02 safety).
// ============================================================================
module tb_wfifo;
    reg clk  = 0; always #62.5 clk  = ~clk;    // 8 MHz cpu_clk
    reg fclk = 0; always #5    fclk = ~fclk;   // 100 MHz sdram_clk
    reg res_n = 0;

    reg         cs = 0, we = 0;
    reg  [24:0] ba = 25'd0;
    reg  [7:0]  wd = 8'd0;
    wire [7:0]  rd;
    wire        ready;

    ext_ram_sdram dut (
        .clk(clk), .sdram_clk(fclk), .reset_n(res_n),
        .cs(cs), .we(we), .byte_addr(ba), .wr_data(wd),
        .rd_data(rd), .ready(ready),
        .ld_wr(1'b0), .ld_addr(25'd0), .ld_data(8'd0), .ld_busy(),
        .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
        .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
        .SDRAM_DQML(), .SDRAM_DQMH()
    );

    // ---- instrumentation: count FIFO pushes, track wf_cnt peak ------------
    integer push_cnt = 0;
    integer cnt_peak = 0;
    always @(posedge clk) if (res_n) begin
        if (dut.wpush) push_cnt = push_cnt + 1;
        if (dut.wf_cnt > cnt_peak) cnt_peak = dut.wf_cnt;
    end

    // ---- '816 bus semantics: a cycle commits only on a posedge with -------
    // ---- ready=1; while stalled the CPU freezes holding the bus     -------
    task cyc816(input w, input [24:0] a, input [7:0] d);
        begin
            cs <= 1; we <= w; ba <= a; wd <= d;
            @(negedge clk);
            while (!ready) @(negedge clk);   // comb ready, settled mid-cycle
            @(posedge clk);                  // committing edge
        end
    endtask

    // ---- r65c02 write: blows through rdy, exactly one cycle on the bus ----
    task wr02(input [24:0] a, input [7:0] d);
        begin
            cs <= 1; we <= 1; ba <= a; wd <= d;
            @(negedge clk);
            @(posedge clk);                  // done, rdy or not
        end
    endtask

    // ---- rdy-honoring dead cycle (fetch stand-in, cs=0) --------------------
    task dead02;
        begin
            cs <= 0; we <= 0;
            @(negedge clk);
            while (!ready) @(negedge clk);   // global wf_hi stall parks here
            @(posedge clk);
        end
    endtask

    // all tasks enter and leave in posedge context
    task idle(input integer n);
        begin
            cs <= 0; we <= 0;
            repeat (n) @(posedge clk);
        end
    endtask

    integer k, errors;
    reg [7:0] got;

    initial begin
        repeat (4) @(posedge clk);
        res_n = 1;
        idle(60);                            // SDRAM init (400 fast cycles)

        // ================= PHASE 1: '816 back-to-back writes ==============
        errors   = 0;
        push_cnt = 0;
        for (k = 0; k < 6; k = k + 1)
            cyc816(1'b1, 25'h001000 + k, 8'hA0 + k[7:0]);
        idle(60);                            // drain
        $display("[WF  ] phase1: pushes=%0d (6 writes issued), wf_cnt peak=%0d",
                 push_cnt, cnt_peak);
        if (push_cnt != 6)
            $display("[WF  ] *** REPEAT-PUSH: held write pushed %0d times ***",
                     push_cnt);
        for (k = 0; k < 6; k = k + 1) begin
            cyc816(1'b0, 25'h001000 + k, 8'h00);
            got = rd;
            if (got !== (8'hA0 + k[7:0])) begin
                errors = errors + 1;
                $display("[WF  ] MISMATCH @%06x: got %02x want %02x",
                         25'h001000 + k, got, 8'hA0 + k[7:0]);
            end
        end
        if (errors == 0 && push_cnt == 6)
            $display("[WF  ] phase1 ('816 held writes): PASS");
        else
            $display("[WF  ] phase1 ('816 held writes): FAIL (%0d lost/corrupt)",
                     errors);

        // ================= PHASE 2: r65c02 blow-through writes =============
        errors   = 0;
        push_cnt = 0;
        cnt_peak = 0;
        idle(20);
        for (k = 0; k < 6; k = k + 1) begin
            wr02(25'h002000 + k, 8'h50 + k[7:0]);
            dead02; dead02; dead02;          // sta abs: 3 rdy-honoring cycles
        end
        idle(60);
        $display("[WF  ] phase2: pushes=%0d (6 writes issued), wf_cnt peak=%0d",
                 push_cnt, cnt_peak);
        for (k = 0; k < 6; k = k + 1) begin
            cyc816(1'b0, 25'h002000 + k, 8'h00);
            got = rd;
            if (got !== (8'h50 + k[7:0])) begin
                errors = errors + 1;
                $display("[WF  ] MISMATCH @%06x: got %02x want %02x",
                         25'h002000 + k, got, 8'h50 + k[7:0]);
            end
        end
        if (errors == 0 && push_cnt == 6)
            $display("[WF  ] phase2 (r65c02 blow-through): PASS");
        else
            $display("[WF  ] phase2 (r65c02 blow-through): FAIL (%0d lost/corrupt)",
                     errors);

        $display("[WF  ] done");
        $finish;
    end

    initial begin
        #40000000;
        $display("[WF  ] TIMEOUT (ready wedged? wf_cnt=%0d)", dut.wf_cnt);
        $finish;
    end
endmodule
