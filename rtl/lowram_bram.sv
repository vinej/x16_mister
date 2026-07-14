//============================================================================
// lowram_bram.sv  -  CPU LowRAM in deterministic M10K BRAM.
//
// Covers CPU address range $0000-$9EFF (40704 bytes; everything below
// the I/O region at $9F00-$9FFF).  Replaces the LowRAM path of
// ext_ram.sv -- the external SRAM is now used ONLY for HiRAM
// ($A000-$BFFF, 16 banks via $0000 register).
//
// Rationale: SRAM contents are non-deterministic across power cycles
// and reflashes.  ramtas tries to clear it, but any glitch leaves
// junk that derails the X16 boot in different ways each time.  M10K
// is zero-initialized by the FPGA configuration (deterministic), so
// the boot path runs with a clean LowRAM every time.
//
// The 40 KB of M10K we need here is freed by reducing rom_banks.sv to
// 13 banks instead of 16 (see rom_banks.sv header).
//
// Same negedge-read convention as rom_banks.sv / ext_ram.sv so the
// CPU sees mem[addr from THIS cycle] when it samples cpu_di on the
// next posedge.
//============================================================================
module lowram_bram (
    input  logic        clk,
    input  logic [15:0] addr,        // CPU address ($0000-$9EFF)
    input  logic        cs,          // 1 = this access targets LowRAM
    input  logic        we,          // 1 = write, 0 = read
    input  logic  [7:0] wr_data,
    output logic  [7:0] rd_data
);

    // 40704 bytes = $9F00.  Quartus will pack this into 32 M10K blocks
    // (each is 10240 bits = 1280 bytes).  A few unused bytes at the
    // top of the last block are not a problem.
    (* ramstyle = "M10K" *) logic [7:0] mem [0:40703];

    // Power-on contents are all zero (FPGA configuration default for
    // M10K with no $readmemh).  This is what ramtas later overwrites
    // for the bits it cares about -- but having a known starting state
    // means boot is identical every power-cycle.

    // Write port (posedge clk).  Gated by cs & we; addr is naturally
    // bounded to $0000-$9EFF by the parent's lowram_cs decode.
    always_ff @(posedge clk) begin
        if (cs & we) mem[addr] <= wr_data;
    end

    // *** NEGEDGE READ *** -- half-cycle head start, see rom_banks.sv.
    always_ff @(negedge clk) begin
        rd_data <= mem[addr];
    end

endmodule
