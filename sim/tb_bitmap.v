`timescale 1ns/1ps
// ============================================================================
// tb_bitmap.v -- bitmap_engine scanout vs golden image.
//
// Real path: bitmap_engine <-> ext_ram_sdram <-> sdram_sim (two-plane model).
// A VERA-like raster (de high 640 px/line, hblank, vblank + vs) is driven into
// the engine; a golden planar framebuffer + palette are preloaded; every
// scanned-out pixel (bmp_r/g/b while bmp_active) is checked against
// palette[golden_pixel(x,y)].
//
//   -gTMODE=1  640x480x8bpp (320 words/line, 2 px/word)
//   -gTMODE=2  640x480x4bpp (160 words/line, 4 px/word, hi nibble = left)
//
// Frame 1 warms the ping-pong; frame 2 is checked.  FB_BASE_WORD=0 in sim so
// the framebuffer lands at low word indices (sdram_sim is 8 M words).
// ============================================================================
module tb_bitmap;
    parameter TMODE  = 1;
    parameter NLINES = 6;                          // active lines per frame (small = fast)
    localparam integer WPL = (TMODE == 1) ? 320 : 160;

    reg pclk = 0; always #20 pclk = ~pclk;         // 25 MHz pix_clk
    reg fclk = 0; always #5  fclk = ~fclk;         // 100 MHz sdram_clk
    reg res_n = 0;

    // raster + control
    reg        de = 0, vs = 0, enable = 0;
    reg  [1:0] mode = 0;

    // palette write
    reg        pal_we = 0;
    reg  [7:0] pal_idx = 0;
    reg [11:0] pal_dat = 0;

    // engine <-> sdram fb stream
    wire        fb_go, fb_valid, fb_done;
    wire [23:0] fb_base;
    wire [10:0] fb_len;
    wire [15:0] fb_word;

    wire [3:0] bmp_r, bmp_g, bmp_b;
    wire       bmp_active;

    bitmap_engine #(.FB_BASE_WORD(24'h000000)) eng (
        .pix_clk(pclk), .reset_n(res_n), .enable(enable), .mode(mode), .de(de), .vs(vs),
        .bmp_r(bmp_r), .bmp_g(bmp_g), .bmp_b(bmp_b), .bmp_active(bmp_active),
        .pal_clk(pclk), .pal_we(pal_we), .pal_idx(pal_idx), .pal_data(pal_dat),
        .sdram_clk(fclk),
        .fb_go(fb_go), .fb_base(fb_base), .fb_len(fb_len),
        .fb_valid(fb_valid), .fb_word(fb_word), .fb_done(fb_done)
    );

    ext_ram_sdram sdr (
        .clk(fclk), .sdram_clk(fclk), .reset_n(res_n),
        .cs(1'b0), .we(1'b0), .byte_addr(25'd0), .wr_data(8'd0),
        .rd_data(), .ready(),
        .ld_wr(1'b0), .ld_addr(25'd0), .ld_data(8'd0), .ld_busy(),
        .bk_rd(1'b0), .bk_addr(25'd0), .bk_rdata(), .bk_ack(),
        .wr_snoop(), .wr_snoop_addr(),
        .fb_go(fb_go), .fb_base(fb_base), .fb_len(fb_len),
        .fb_valid(fb_valid), .fb_word(fb_word), .fb_done(fb_done),
        .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
        .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
        .SDRAM_DQML(), .SDRAM_DQMH()
    );

    // ---- golden pattern + palette model -----------------------------------
    function [7:0] gpix(input integer x, input integer y);
        gpix = (TMODE == 1) ? ((x + y*7 + 3) & 8'hFF)
                            : ((x + y*5 + 1) & 8'h0F);
    endfunction
    function [11:0] palf(input [7:0] i);
        palf = { i[3:0], i[7:4], (i ^ 8'h3C) & 8'h0F };  // R,G,B nibbles
    endfunction

    // ---- raster generator -------------------------------------------------
    task active_line; integer p; begin
        for (p = 0; p < 640; p = p + 1) begin de <= 1; @(posedge pclk); end
        for (p = 0; p < 160; p = p + 1) begin de <= 0; @(posedge pclk); end
    end endtask
    task blank_line; integer p; begin
        for (p = 0; p < 800; p = p + 1) begin de <= 0; @(posedge pclk); end
    end endtask
    task do_frame; integer l; begin
        vs <= 1; repeat (4) @(posedge pclk); vs <= 0;
        blank_line; blank_line; blank_line;          // vblank (line 0 fetches here)
        for (l = 0; l < NLINES; l = l + 1) active_line;
        blank_line;
    end endtask

    // ---- checker (pix domain) ---------------------------------------------
    reg        checking = 0;
    reg        vs_rc = 0, ba_rc = 0;
    integer    ox = 0, oline = 0, errs = 0, nchk = 0;
    reg [11:0] expc;
    always @(posedge pclk) begin
        vs_rc <= vs;
        ba_rc <= bmp_active;
        if (vs & ~vs_rc) begin oline <= 0; ox <= 0; end
        else begin
            if (bmp_active) begin
                if (checking) begin
                    expc = palf(gpix(ox, oline));
                    if ({bmp_r, bmp_g, bmp_b} !== expc) begin
                        errs = errs + 1;
                        if (errs <= 8)
                            $display("[BMP ] m%0d MISMATCH (x=%0d,y=%0d) got %03x want %03x",
                                     TMODE, ox, oline, {bmp_r, bmp_g, bmp_b}, expc);
                    end
                    nchk = nchk + 1;
                end
                ox <= ox + 1;
            end
            if (ba_rc & ~bmp_active) begin ox <= 0; oline <= oline + 1; end
        end
    end

    integer k, c, x, y;
    reg [7:0] lo, hi;

    integer go_cnt = 0, val_cnt = 0;
    always @(posedge fclk) begin
        if (fb_go)    go_cnt  = go_cnt + 1;
        if (fb_valid) val_cnt = val_cnt + 1;
    end

    initial begin
        repeat (4) @(posedge fclk);
        res_n = 1;
        repeat (600) @(posedge fclk);                // SDRAM self-init

        // preload palette into the engine
        for (k = 0; k < 256; k = k + 1) begin
            @(negedge pclk);
            pal_we <= 1; pal_idx <= k[7:0]; pal_dat <= palf(k[7:0]);
        end
        @(negedge pclk); pal_we <= 0;

        // preload golden framebuffer (planar) into sdram_sim
        for (y = 0; y < NLINES; y = y + 1)
            for (c = 0; c < WPL; c = c + 1) begin
                if (TMODE == 1) begin
                    lo = gpix(2*c,   y);             // even x -> low-byte plane
                    hi = gpix(2*c+1, y);             // odd  x -> high-byte plane
                end else begin
                    lo = { gpix(4*c,   y)[3:0], gpix(4*c+1, y)[3:0] };
                    hi = { gpix(4*c+2, y)[3:0], gpix(4*c+3, y)[3:0] };
                end
                sdr.u_sdram.wmem[y*WPL + c] = { hi, lo };
            end

        enable = 1; mode = TMODE[1:0];

        do_frame;                 // frame 1: warm the ping-pong
        checking = 1;
        errs = 0; nchk = 0;
        do_frame;                 // frame 2: checked

        @(posedge pclk);
        $display("[BMP ] mode %0d: checked %0d px (expected %0d), errors=%0d",
                 TMODE, nchk, NLINES*640, errs);
        if (errs == 0 && nchk == NLINES*640)
            $display("[BMP ] mode %0d: PASS", TMODE);
        else
            $display("[BMP ] mode %0d: FAIL", TMODE);
        $finish;
    end

    initial begin
        #6000000;
        $display("[BMP ] TIMEOUT (oline=%0d nchk=%0d)", oline, nchk);
        $finish;
    end
endmodule
