`timescale 1ns/1ps
// ============================================================================
// tb_wai.v -- WAI ($CB) / STP ($DB) semantics test (jyv 2026-07-07).
//
// Program (emulation-mode 65C02 code, same on both CPUs):
//   SEI; clear $10-$12
//   WAI            ; #1: I=1 -> wake on IRQ pin, do NOT take handler
//   INC $10        ; -> 1
//   CLI
//   WAI            ; #2: I=0 -> wake on IRQ, take handler (INC $11; RTI)
//   INC $10        ; -> 2
//   STP            ; halt until reset; later IRQ+NMI pulses must be ignored
//   INC $10 / JMP  ; must NEVER run
//
// Configs (run all three):
//   -gCPU816=1          P65C816, native WAI/STP        -> expect PASS
//   -gCPU816=0 -gSHIM=0 bare r65c02                    -> documents the gap
//   -gCPU816=0 -gSHIM=1 r65c02 + rtl/wai_shim.sv       -> expect PASS
// ============================================================================
module tb_wai;
    parameter integer CPU816 = 1;
    parameter integer SHIM   = 0;
    parameter integer CPUMJ  = 0;   // MJoergen 65c02 (+wai_shim, always)

    integer errors = 0;

    // 8 MHz CPU clock
    reg cpu_clk = 0; always #62.5 cpu_clk = ~cpu_clk;

    reg  res_n = 0;
    reg  irq_n = 1;
    reg  nmi_n = 1;

    wire        r_w_n, sync;
    wire [15:0] addr, pc;
    wire [7:0]  dout;

    // ---- 64KB behavioral RAM ----
    reg [7:0] mem [0:65535];
    wire [7:0] mem_q = mem[addr];

    // ---- CPU + optional shim ----
    wire [7:0] cpu_din;
    wire       cpu_rdy;

    // NOTE: r65c02_wrap's `sync` output is LOW during the opcode-fetch
    // cycle (bus-trace proven, tb_wai WAIDBG); the shim wants a fetch
    // indicator, hence the inversion here (same in x16.sv).
    generate if (CPUMJ != 0) begin : g_shim_mj
        // mj65c02: sync is active-HIGH on fetch (no inversion)
        wai_shim u_shim (
            .sync(sync), .din_in(mem_q),
            .irq_n(irq_n), .nmi_n(nmi_n), .rdy_in(1'b1),
            .din_out(cpu_din), .rdy_out(cpu_rdy));
    end else if (CPU816 == 0 && SHIM != 0) begin : g_shim
        wai_shim u_shim (
            .sync(~sync), .din_in(mem_q),
            .irq_n(irq_n), .nmi_n(nmi_n), .rdy_in(1'b1),
            .din_out(cpu_din), .rdy_out(cpu_rdy));
    end else begin : g_noshim
        assign cpu_din = mem_q;
        assign cpu_rdy = 1'b1;
    end endgenerate

    generate if (CPUMJ != 0) begin : g_cpumj
        mj65c02_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(irq_n), .nmi_n(nmi_n), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(cpu_din), .dout(dout), .pc(pc),
            .bus_valid()
        );
    end else if (CPU816 != 0) begin : g_cpu816
        p65c816_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(irq_n), .nmi_n(nmi_n), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(cpu_din), .dout(dout), .pc(pc),
            .emu_mode(), .i_flag(), .vpb(), .bus_valid()
        );
    end else begin : g_cpu02
        r65c02_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(irq_n), .nmi_n(nmi_n), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(cpu_din), .dout(dout), .pc(pc)
        );
    end endgenerate

    // synchronous RAM writes
    always @(posedge cpu_clk)
        if (cpu_rdy && !r_w_n) mem[addr] <= dout;

    task chk(input [7:0] got, input [7:0] expct, input [255:0] what); begin
        if (got !== expct) begin
            $display("[WAI ] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    task irq_pulse; begin
        irq_n = 0; repeat (8) @(posedge cpu_clk); irq_n = 1;
    end endtask

    integer i;
    initial begin
        for (i = 0; i < 65536; i = i + 1) mem[i] = 8'hEA;  // NOP sea

        // program
        mem['h0200] = 8'h78;                                        // SEI
        mem['h0201] = 8'hA9; mem['h0202] = 8'h00;                   // LDA #0
        mem['h0203] = 8'h85; mem['h0204] = 8'h10;                   // STA $10
        mem['h0205] = 8'h85; mem['h0206] = 8'h11;                   // STA $11
        mem['h0207] = 8'h85; mem['h0208] = 8'h12;                   // STA $12
        mem['h0209] = 8'hCB;                                        // WAI #1
        mem['h020A] = 8'hE6; mem['h020B] = 8'h10;                   // INC $10
        mem['h020C] = 8'h58;                                        // CLI
        mem['h020D] = 8'hCB;                                        // WAI #2
        mem['h020E] = 8'hE6; mem['h020F] = 8'h10;                   // INC $10
        mem['h0210] = 8'hDB;                                        // STP
        mem['h0211] = 8'hE6; mem['h0212] = 8'h10;                   // INC $10 (never)
        mem['h0213] = 8'h4C; mem['h0214] = 8'h11; mem['h0215] = 8'h02; // JMP $0211

        mem['h0300] = 8'hE6; mem['h0301] = 8'h11; mem['h0302] = 8'h40; // IRQ: INC $11; RTI
        mem['h0320] = 8'hE6; mem['h0321] = 8'h12; mem['h0322] = 8'h40; // NMI: INC $12; RTI

        mem['hFFFA] = 8'h20; mem['hFFFB] = 8'h03;   // NMI   -> $0320
        mem['hFFFC] = 8'h00; mem['hFFFD] = 8'h02;   // RESET -> $0200
        mem['hFFFE] = 8'h00; mem['hFFFF] = 8'h03;   // IRQ   -> $0300

        repeat (10) @(posedge cpu_clk);
        res_n = 1;

        // --- park at WAI #1: nothing may execute past it ---
        repeat (2000) @(posedge cpu_clk);
        chk(mem['h0010], 8'h00, "parked at WAI1 ($10 still 0)");
        chk(mem['h0011], 8'h00, "no handler before IRQ");

        // --- IRQ with I=1: wake, fall through, no handler ---
        irq_pulse;
        repeat (500) @(posedge cpu_clk);
        chk(mem['h0010], 8'h01, "woke from WAI1 ($10=1)");
        chk(mem['h0011], 8'h00, "I=1: handler NOT taken");

        // --- parked at WAI #2 (I=0 now) ---
        repeat (500) @(posedge cpu_clk);
        chk(mem['h0010], 8'h01, "parked at WAI2 ($10 still 1)");

        // --- IRQ with I=0: wake AND take handler ---
        irq_pulse;
        repeat (500) @(posedge cpu_clk);
        chk(mem['h0011], 8'h01, "I=0: handler taken once");
        chk(mem['h0010], 8'h02, "resumed after WAI2 ($10=2)");

        // --- STP: IRQ and NMI must both be ignored ---
        repeat (500) @(posedge cpu_clk);
        irq_pulse;
        repeat (100) @(posedge cpu_clk);
        nmi_n = 0; repeat (8) @(posedge cpu_clk); nmi_n = 1;
        repeat (1000) @(posedge cpu_clk);
        chk(mem['h0010], 8'h02, "STP: $10 frozen at 2");
        chk(mem['h0011], 8'h01, "STP: IRQ ignored");
        chk(mem['h0012], 8'h00, "STP: NMI ignored");

        if (errors == 0) $display("[WAI ] ALL TESTS PASS (CPUMJ=%0d CPU816=%0d SHIM=%0d)", CPUMJ, CPUMJ ? 0 : CPU816, SHIM);
        else             $display("[WAI ] %0d ERRORS (CPUMJ=%0d CPU816=%0d SHIM=%0d)", errors, CPUMJ, CPUMJ ? 0 : CPU816, SHIM);
        $finish;
    end


`ifdef WAIDBG
    integer dbgcnt = 0;
    always @(posedge cpu_clk) begin
        if (res_n && dbgcnt < 60) begin
            dbgcnt = dbgcnt + 1;
            $display("[WDBG] a=%04x q=%02x sync=%b rdy=%b rwn=%b irq=%b", addr, mem_q, sync, cpu_rdy, r_w_n, irq_n);
        end
    end
`endif

    initial begin
        #5_000_000;
        $display("[WAI ] TIMEOUT");
        $finish;
    end

endmodule
