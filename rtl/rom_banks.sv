//============================================================================
// rom_banks.sv  -  X16 ROM, full 16 physical banks x 16 KB = 256 KB.
//
// Loaded from rom/rom.hex (262144 lines, one byte per line; generated directly
// from rom/rom.bin via `xxd -p -c1`).  Direct {bank,addr} addressing -- no compact remap, no
// skipped banks.  All 16 banks (incl. $09 DEMO, $0D/$0E X16EDIT) are present.
//
// X16 bank layout (per x16-rom-r49/inc/banks.inc):
//   $00 KERNAL  $01 KEYBD   $02 CBDOS   $03 FAT32   $04 BASIC   $05 MONITOR
//   $06 CHARSET $07 DIAG    $08 GRAPH   $09 DEMO    $0A AUDIO   $0B UTIL
//   $0C BANNEX  $0D X16EDIT $0E X16EDIT $0F BASLOAD
//
// 256 KB ROM = 256 M10K blocks; fits the DE10-Nano (553 M10K) alongside the
// 128 KB VERA VRAM and 48 KB LowRAM.
//
// Phase g (DONE): the $readmemh init remains as the BAKED-IN DEFAULT (works
// with no file present); the HPS ROM loader writes over it through the
// dual-clock ioctl write port below (boot1.rom auto-load at core start, or
// OSD "Load ROM" -> ioctl index 1).  The core is held in reset during the
// download (x16.sv rom_loading), so the negedge read port is idle while the
// 100 MHz write port streams.
//============================================================================
module rom_banks (
    input  logic        clk,
    input  logic  [3:0] bank,         // ROM_BANK register at $0001 (CPU view)
    input  logic [13:0] addr,         // CPU address within $C000-$FFFF window
    output logic  [7:0] rd_data,

    // HPS ioctl write port (hps_io clock domain = 100 MHz sdram_clk)
    input  logic        wr_clk,
    input  logic        wr_en,
    input  logic [17:0] wr_addr,
    input  logic  [7:0] wr_data
);

    // 16 banks x 16 KB = 262144 bytes.
    (* ramstyle = "M10K" *) logic [7:0] mem [0:262143];

    // BAKE_ROM (personal builds only): compile the copyrighted X16 ROM into the
    // bitstream from rom/rom.hex.  Regenerate that file first with
    // scripts/rom_bin2hex.py -- it is NOT distributed with this repo.
    // PUBLIC builds leave BAKE_ROM undefined, so the .rbf carries no ROM; the
    // ROM is supplied at runtime via boot1.rom / the OSD "Load ROM" loader
    // (the ioctl write port below), which overwrites mem regardless.
`ifdef BAKE_ROM
    initial $readmemh("rom/rom.hex", mem);
`endif

    // 18-bit byte address: {bank[3:0], addr[13:0]}.
    logic [17:0] full_addr;
    assign full_addr = {bank, addr};

    // *** NEGEDGE BRAM CLOCK *** gives the M10K a half-cycle head start so
    // rd_data reflects mem[addr from THIS cycle] when the CPU samples cpu_di
    // at the next posedge.
    logic [7:0] mem_rd;
    always_ff @(negedge clk) begin
        mem_rd <= mem[full_addr];
    end

    assign rd_data = mem_rd;

    // ioctl write port (dual-clock simple-dual-port M10K: write @ wr_clk,
    // read @ negedge clk -- the CPU is in reset while this streams).
    always_ff @(posedge wr_clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
    end

endmodule
