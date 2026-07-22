`timescale 1ns/1ps
// ============================================================================
// tb_bmpregs.v -- bitmap_regs decode / planar-address / palette unit test.
// ============================================================================
module tb_bmpregs;
    reg clk = 0; always #62.5 clk = ~clk;   // 8 MHz
    reg res_n = 0;

    reg        cs = 0, rwn = 1, en = 1;
    reg  [3:0] addr = 0;
    reg  [7:0] di = 0;
    wire [7:0] do_o;
    reg        master_en = 1;
    wire       bmp_enable;
    wire [1:0] bmp_mode;
    wire       bmp_passthru;
    wire       fb_wr_sel, fb_rd_sel;
    wire [24:0] fb_wr_addr;
    wire       pal_we;
    wire [7:0] pal_idx;
    wire [11:0] pal_data;

    bitmap_regs dut (
        .clk(clk), .reset_n(res_n), .cs(cs), .rwn(rwn), .en(en),
        .addr(addr), .di(di), .do_o(do_o), .master_en(master_en),
        .bmp_enable(bmp_enable), .bmp_mode(bmp_mode), .bmp_passthru(bmp_passthru),
        .fb_wr_sel(fb_wr_sel), .fb_rd_sel(fb_rd_sel), .fb_addr(fb_wr_addr),
        .pal_we(pal_we), .pal_idx(pal_idx), .pal_data(pal_data),
        .blit_start(), .blit_src(), .blit_dst(), .blit_len(), .blit_done(1'b0)
    );

    integer errs = 0;
    task chk(input cond, input [255:0] msg);
        if (!cond) begin errs = errs + 1; $display("[BR  ] FAIL: %0s", msg); end
    endtask

    task wr(input [3:0] a, input [7:0] d);
        begin @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=a; di<=d;
              @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1; end
    endtask
    task rd(input [3:0] a);
        begin @(negedge clk); cs<=1; rwn<=1; en<=1; addr<=a; #1; end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        res_n = 1;
        @(negedge clk);

        // ID + CTRL
        rd(4'd1); chk(do_o === 8'hB5, "ID != B5");
        wr(4'd0, 8'b0000_0101);              // enable=1, mode=2 (bits[2:1]=10)
        chk(bmp_enable === 1'b1, "enable");
        chk(bmp_mode   === 2'd2, "mode");
        rd(4'd0); chk(do_o === 8'b0000_0101, "CTRL readback");

        // master_en gating
        master_en = 0; #1;
        chk(bmp_enable === 1'b0, "master_en gate");
        master_en = 1; #1;

        // pointer + planar framebuffer write.  ptr=5 -> {A24=1, 0x800000+2}
        wr(4'd2, 8'h05); wr(4'd3, 8'h00); wr(4'd4, 8'h00);
        @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=4'd5; di<=8'hAB; #1;
        chk(fb_wr_sel  === 1'b1,          "fb_wr_sel");
        chk(fb_wr_addr === 25'h1800002,   "fb_wr_addr planar (ptr=5)");
        @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1;
        chk(dut.ptr === 19'd6, "ptr auto-increment");

        // next DATA write should target ptr=6 -> {A24=0, 0x800000+3}
        @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=4'd5; di<=8'hCD; #1;
        chk(fb_wr_addr === 25'h0800003, "fb_wr_addr planar (ptr=6)");
        @(posedge clk); @(negedge clk); cs<=0; en<=0; rwn<=1;

        // palette write: idx=0x10, {G=3,B=4}, R=7 -> pal_data=0x734, idx++ ->0x11
        wr(4'd6, 8'h10);                     // PALADR
        wr(4'd7, 8'h34);                     // PALLO {G,B}
        @(negedge clk); cs<=1; rwn<=0; en<=1; addr<=4'd8; di<=8'h07; // PALHI {R}
        @(posedge clk); #1;
        chk(pal_we   === 1'b1,     "pal_we pulse");
        chk(pal_idx  === 8'h10,    "pal_idx");
        chk(pal_data === 12'h734,  "pal_data {R,G,B}");
        @(negedge clk); cs<=0; en<=0; rwn<=1;
        chk(dut.cur_idx === 8'h11, "palette cursor auto-increment");

        if (errs == 0) $display("[BR  ] ALL OK");
        else           $display("[BR  ] FAILED (%0d)", errs);
        $finish;
    end
    initial begin #200000; $display("[BR  ] TIMEOUT"); $finish; end
endmodule
