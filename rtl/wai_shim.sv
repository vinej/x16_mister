// ============================================================================
// wai_shim.sv -- WAI/STP support for the r65c02 control build (jyv 2026-07-07)
//
// The r65c02_tc core predates the WDC WAI ($CB) / STP ($DB) opcodes; X16
// software uses WAI heavily for VSYNC pacing.  The P65C816 (shipping CPU)
// implements both natively -- this shim is only instantiated in the
// `CPU_R65C02 comparison build.
//
// Mechanism (no CPU-core changes):
//   * On the opcode FETCH cycle (sync=1) of $CB, hold the CPU's clock-enable
//     low while irq_n & nmi_n are both high.  The bus is frozen mid-fetch, so
//     din keeps presenting $CB and the stall self-sustains.  When an
//     interrupt line asserts, the enable is released -- matching real WAI:
//     wake regardless of the I flag; the handler (if I=0) is taken at the
//     next instruction boundary.
//   * On the fetch of $DB, hold the enable low unconditionally: STP halts
//     until reset (the FSM resets out of the frozen fetch).
//   * Whenever the fetch does complete, the opcode byte is substituted with
//     $EA (NOP), so the generated r65c02 FSM never decodes the two opcodes
//     it doesn't know (behavior there is unverified).
//
// Known corner (accepted, debug build only): an NMI whose pulse has already
// deasserted but is still internally pending in the CPU cannot be observed
// here; a WAI fetched in that exact boundary window would stall until the
// next IRQ.  The X16's SMC NMI is stretched 16 cpu_clk cycles, making the
// window practically unhittable.
// ============================================================================
module wai_shim (
    input  logic       sync,      // CPU opcode-fetch cycle
    input  logic [7:0] din_in,    // data bus toward the CPU
    input  logic       irq_n,
    input  logic       nmi_n,
    input  logic       rdy_in,    // upstream ready (mem/VERA/SPI stalls)

    output logic [7:0] din_out,   // to CPU din
    output logic       rdy_out    // to CPU clock-enable
);
    wire fetch_wai = sync && (din_in == 8'hCB);
    wire fetch_stp = sync && (din_in == 8'hDB);

    wire stall = (fetch_wai && irq_n && nmi_n)   // WAI: wait for any interrupt
               | fetch_stp;                      // STP: wait for reset

    assign rdy_out = rdy_in & ~stall;

    // Never let the unknown opcodes reach the FSM.
    assign din_out = (fetch_wai || fetch_stp) ? 8'hEA : din_in;

endmodule
