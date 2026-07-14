`timescale 1ns/1ps
// ============================================================================
// sdram_sim.v -- behavioral stand-in for rtl/sdram.v (MiST byte controller).
// Same port list, same handshake shape: a 1-cycle `ce` pulse starts an access
// (params latched at the ce rising edge, like the real controller's cycle
// start); the byte lands in mem[] / on dout a few clk later -- comfortably
// before ext_ram_sdram samples at CYCLE_LEN=9.  Chip pins are stubbed.
// 8 MB backing store covers HiRAM (0x000000-0x1FFFFF) and the cart banks
// (0x480000-0x7FFFFF).  Init pattern $DD = "uninitialized RAM".
// ============================================================================
module sdram (
    output [12:0] sd_addr,
    inout  [15:0] sd_data,
    output  [1:0] sd_ba,
    output        sd_cs,
    output        sd_we,
    output        sd_ras,
    output        sd_cas,
    output        sd_clk,
    output  [1:0] sd_dqm,

    input         init,
    input         clk,
    input  [24:0] addr,
    input   [7:0] din,
    output reg [7:0] dout,
    input         refresh,
    input         ce,
    input         we
);
    assign sd_addr = 13'd0;
    assign sd_ba   = 2'd0;
    assign sd_cs   = 1'b1;
    assign sd_we   = 1'b1;
    assign sd_ras  = 1'b1;
    assign sd_cas  = 1'b1;
    assign sd_clk  = 1'b0;
    assign sd_dqm  = 2'b00;

    reg [7:0] mem [0:8388607];
    integer i;
    initial for (i = 0; i < 8388608; i = i + 1) mem[i] = 8'hDD;

    reg        last_ce = 1'b0;
    reg  [3:0] q = 4'd0;
    reg [24:0] a_l;
    reg  [7:0] d_l;
    reg        w_l;

    always @(posedge clk) begin
        last_ce <= ce;
        if (ce && !last_ce) begin
            q   <= 4'd1;
            a_l <= addr;
            d_l <= din;
            w_l <= we;
        end else if (q != 4'd0) begin
            q <= q + 4'd1;
            if (q == 4'd4) begin
                if (w_l) mem[a_l] <= d_l;
                else     dout     <= mem[a_l];
            end
            if (q == 4'd8) q <= 4'd0;
        end
    end
endmodule
