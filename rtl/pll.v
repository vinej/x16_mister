// jyv 2026-07-07: the X16 core PLL (50 MHz ref -> 25 / 12.5 / 8 / 100 MHz).
// Plain altera_pll instantiation, portable across Quartus 17.0 - 24.1;
// relocated to rtl/pll/ as the single PLL for every toolchain version
// (the old root pll.qip was 24.1-generated metadata that Quartus 17.0.x
// could not consume -- community "pll_0002 is missing" build failures).
`timescale 1 ps / 1 ps
module pll (
        input  wire  refclk,   //  refclk.clk
        input  wire  rst,      //   reset.reset
        output wire  outclk_0, // outclk0.clk  25.0 MHz (VERA pixel)
        output wire  outclk_1, // outclk1.clk  12.5 MHz (unused)
        output wire  outclk_2, // outclk2.clk   8.0 MHz (CPU)
        output wire  outclk_3, // outclk3.clk 100.0 MHz (SDRAM/HPS)
        output wire  locked    //  locked.export
    );

    pll_0002 pll_inst (
        .refclk   (refclk),
        .rst      (rst),
        .outclk_0 (outclk_0),
        .outclk_1 (outclk_1),
        .outclk_2 (outclk_2),
        .outclk_3 (outclk_3),
        .locked   (locked)
    );

endmodule
