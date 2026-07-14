`timescale 1ns/1ps
// ============================================================================
// tb_verafx.v -- VERA FX feature verification against the REAL VERA `top`
// (jyv 2026-07-07).  Exercises every FX feature ported from vera-module
// v47.0.2 through CPU-shaped extbus cycles, the same integration as x16.sv:
//
//   1.  Version registers via 6-bit DCSEL (incl. the ROM's CTRL=$7E path)
//   2.  FX_CTRL readback (DCSEL=2)
//   3.  Multiplier: 69*420 written to VRAM through cache write
//   4.  Accumulator: accumulate, add/subtract offset, reset (reg + read)
//   5.  Cache fill from DATA0 reads + full cache write (mask 0)
//   6.  Cache write with partial nibble mask
//   7.  Transparent writes: plain byte, cache write with zero bytes
//   8.  4-bit mode: nibble writes w/ nibble increment + transparency
//   9.  One-byte cache cycling: single write + 4x duplication
//   10. Line draw helper: 8 pixels of a slope-0.5 line (octant 7)
//   11. Polygon filler helper: 2 scanlines, fill-length register checks
//   12. Affine helper: tile walk, map wrap-around, clip enable
//   13. 16-bit hop (+4 -> +1/+3 alternation)
//   14. Regression: FX_CTRL=0 leaves traditional DATA0/DATA1 unchanged
// ============================================================================
module tb_verafx;
    integer errors = 0;

    // 25 MHz VERA pixel clock
    reg clk25 = 0; always #20 clk25 = ~clk25;

    // ---- VERA (real top) ----
    reg        cs_n = 1, rd_n = 1, wr_n = 1;
    reg  [4:0] a = 0;
    reg  [7:0] dq_out = 8'h00;
    reg        dq_drive = 0;
    wire [7:0] extbus_d = dq_drive ? dq_out : 8'hZZ;
    wire       irq_n;

    wire [3:0] tv_r;
    wire       tv_hs, tv_de;
    integer    hs_edges = 0;
    reg        tv_hs_d = 0;
    always @(posedge clk25) begin
        tv_hs_d <= tv_hs;
        if (tv_hs & ~tv_hs_d) hs_edges = hs_edges + 1;
    end

    top u_vera (
        .clk25          (clk25),
        .extbus_cs_n    (cs_n),
        .extbus_rd_n    (rd_n),
        .extbus_wr_n    (wr_n),
        .extbus_a       (a),
        .extbus_d       (extbus_d),
        .extbus_irq_n   (irq_n),
        .vga_r(tv_r), .vga_g(), .vga_b(),
        .vga_hsync(tv_hs), .vga_vsync(), .vga_de(tv_de),
        .spi_sck(), .spi_mosi(), .spi_miso(1'b1), .spi_ssel_n_sd(),
        .audio_lrck(), .audio_bck(), .audio_data(),
        .dbg_wrdata_r(), .dbg_wraddr_r(), .dbg_do_write(),
        .dbg_video_mode(), .dbg_dcsel(),
        .spi_busy_out(), .spi_autotx_out(),
        .composite_luma(), .composite_chroma()
    );

    // ---- CPU-shaped extbus cycles (x16.sv widens strobes to ~6 clk25) ----
    task vwr(input [4:0] addr, input [7:0] val); begin
        @(posedge clk25);
        a = addr; dq_out = val; dq_drive = 1;
        cs_n = 0; wr_n = 0;
        repeat (8) @(posedge clk25);
        cs_n = 1; wr_n = 1; dq_drive = 0;
        repeat (8) @(posedge clk25);
    end endtask

    reg [7:0] rdv;
    task vrd(input [4:0] addr); begin
        @(posedge clk25);
        a = addr; cs_n = 0; rd_n = 0;
        repeat (8) @(posedge clk25);
        rdv = extbus_d;
        cs_n = 1; rd_n = 1;
        repeat (8) @(posedge clk25);
    end endtask

    task exp(input [7:0] got, input [7:0] expct, input [255:0] what); begin
        if (got !== expct) begin
            $display("[FX  ] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    // CTRL = {reset, DCSEL[5:0], ADDRSEL}
    task ctrl(input [5:0] dcsel, input addrsel); begin
        vwr(5'h05, {1'b0, dcsel, addrsel});
    end endtask

    // Program ADDR0 (also selects ADDRSEL=0, DCSEL=0)
    task a0set(input [16:0] ad, input [3:0] incr, input decr,
               input nibincr, input nib); begin
        ctrl(6'd0, 1'b0);
        vwr(5'h00, ad[7:0]);
        vwr(5'h01, ad[15:8]);
        vwr(5'h02, {incr, decr, nibincr, nib, ad[16]});
    end endtask

    // Program ADDR1 (also selects ADDRSEL=1, DCSEL=0)
    task a1set(input [16:0] ad, input [3:0] incr, input decr,
               input nibincr, input nib); begin
        ctrl(6'd0, 1'b1);
        vwr(5'h00, ad[7:0]);
        vwr(5'h01, ad[15:8]);
        vwr(5'h02, {incr, decr, nibincr, nib, ad[16]});
    end endtask

    // Plain byte poke/peek through ADDR0/DATA0 (FX_CTRL must be 0)
    task vpoke(input [16:0] ad, input [7:0] val); begin
        a0set(ad, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, val);
    end endtask

    task vpeek(input [16:0] ad); begin
        a0set(ad, 4'h0, 1'b0, 1'b0, 1'b0);
        vrd(5'h03);
    end endtask

    task vpeek_exp(input [16:0] ad, input [7:0] expct, input [255:0] what); begin
        vpeek(ad);
        exp(rdv, expct, what);
    end endtask

    // FX register shortcuts (leave DCSEL where they need it)
    task fx_ctrl(input [7:0] v);  begin ctrl(6'd2, 1'b0); vwr(5'h09, v); end endtask
    task fx_mult(input [7:0] v);  begin ctrl(6'd2, 1'b0); vwr(5'h0C, v); end endtask
    task fx_cache(input [31:0] v); begin
        ctrl(6'd6, 1'b0);
        vwr(5'h09, v[7:0]);
        vwr(5'h0A, v[15:8]);
        vwr(5'h0B, v[23:16]);
        vwr(5'h0C, v[31:24]);
    end endtask

    integer i;
    initial begin
        // VERA self-resets via its internal reset_sync; give it a moment
        repeat (400) @(posedge clk25);

        // =================================================================
        // 1. Version registers (6-bit DCSEL) + legacy ROM CTRL=$7E path
        // =================================================================
        vwr(5'h05, 8'h7E);                 // reset=0 DCSEL=63 ADDRSEL=0
        vrd(5'h05); exp(rdv, 8'h7E, "CTRL readback $7E");
        vrd(5'h09); exp(rdv, 8'h56, "version 'V'");
        vrd(5'h0A); exp(rdv, 8'd47, "version major");
        vrd(5'h0B); exp(rdv, 8'd0,  "version minor");
        vrd(5'h0C); exp(rdv, 8'd2,  "version patch");

        // =================================================================
        // 2. FX_CTRL readback at DCSEL=2
        // =================================================================
        fx_ctrl(8'hA5);                    // transp|fill|4bit|mode=01
        ctrl(6'd2, 1'b0);
        vrd(5'h09); exp(rdv, 8'hA5, "FX_CTRL readback");
        fx_ctrl(8'h00);
        ctrl(6'd2, 1'b0);
        vrd(5'h09); exp(rdv, 8'h00, "FX_CTRL cleared");

        // =================================================================
        // 3. Multiplier: 69 * 420 = 28980 ($00007134)
        // =================================================================
        fx_mult(8'h10);                    // multiplier enable
        fx_cache({16'd420, 16'd69});       // B=420, A=69
        fx_ctrl(8'h40);                    // cache write enable
        a0set(17'h00100, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // mask 0 -> write all 4 bytes
        fx_ctrl(8'h00);
        vpeek_exp(17'h00100, 8'h34, "mult byte0");
        vpeek_exp(17'h00101, 8'h71, "mult byte1");
        vpeek_exp(17'h00102, 8'h00, "mult byte2");
        vpeek_exp(17'h00103, 8'h00, "mult byte3");

        // =================================================================
        // 4. Accumulator: 100*10 + 200*5 = 2000, then +/-(3*4), then reset
        // =================================================================
        fx_cache({16'd10, 16'd100});
        ctrl(6'd6, 1'b0);
        vrd(5'h09);                        // FX_ACCUM_RESET (side effect)
        vrd(5'h0A);                        // FX_ACCUM: accum = 1000
        fx_cache({16'd5, 16'd200});
        ctrl(6'd6, 1'b0);
        vrd(5'h0A);                        // accum = 2000
        fx_cache({16'd4, 16'd3});          // product now 12
        fx_ctrl(8'h40);                    // cache write
        a0set(17'h00110, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // VRAM = 2000 + 12 = 2012 ($7DC)
        fx_ctrl(8'h00);
        vpeek_exp(17'h00110, 8'hDC, "accum add byte0");
        vpeek_exp(17'h00111, 8'h07, "accum add byte1");
        vpeek_exp(17'h00112, 8'h00, "accum add byte2");

        fx_mult(8'h30);                    // subtract | multiplier enable
        fx_ctrl(8'h40);
        a0set(17'h00114, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // VRAM = 2000 - 12 = 1988 ($7C4)
        fx_ctrl(8'h00);
        vpeek_exp(17'h00114, 8'hC4, "accum sub byte0");
        vpeek_exp(17'h00115, 8'h07, "accum sub byte1");

        fx_mult(8'h90);                    // reset accum | multiplier enable
        fx_ctrl(8'h40);
        a0set(17'h00118, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // VRAM = 0 + 12
        fx_ctrl(8'h00);
        vpeek_exp(17'h00118, 8'h0C, "accum reset byte0");
        vpeek_exp(17'h00119, 8'h00, "accum reset byte1");
        fx_mult(8'h00);                    // multiplier off again

        // =================================================================
        // 5. Cache fill from DATA0 reads, then full cache write
        // =================================================================
        vpoke(17'h00200, 8'hDE); vpoke(17'h00201, 8'hAD);
        vpoke(17'h00202, 8'hBE); vpoke(17'h00203, 8'hEF);
        fx_mult(8'h00);                    // cache index -> 0
        fx_ctrl(8'h20);                    // cache fill enable
        a0set(17'h00200, 4'h1, 1'b0, 1'b0, 1'b0);
        vrd(5'h03); exp(rdv, 8'hDE, "fill rd0");
        vrd(5'h03); exp(rdv, 8'hAD, "fill rd1");
        vrd(5'h03); exp(rdv, 8'hBE, "fill rd2");
        vrd(5'h03); exp(rdv, 8'hEF, "fill rd3");
        fx_ctrl(8'h40);                    // cache write enable
        a0set(17'h00300, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // flush entire cache
        fx_ctrl(8'h00);
        vpeek_exp(17'h00300, 8'hDE, "cache wr byte0");
        vpeek_exp(17'h00301, 8'hAD, "cache wr byte1");
        vpeek_exp(17'h00302, 8'hBE, "cache wr byte2");
        vpeek_exp(17'h00303, 8'hEF, "cache wr byte3");

        // =================================================================
        // 6. Cache write with partial nibble mask ($0F -> bytes 2,3 only)
        // =================================================================
        vpoke(17'h00304, 8'h99); vpoke(17'h00305, 8'h99);
        vpoke(17'h00306, 8'h99); vpoke(17'h00307, 8'h99);
        fx_cache(32'h44332211);
        fx_ctrl(8'h40);
        a0set(17'h00304, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h0F);                 // mask nibbles 0-3 (bytes 0,1)
        fx_ctrl(8'h00);
        vpeek_exp(17'h00304, 8'h99, "mask keep byte0");
        vpeek_exp(17'h00305, 8'h99, "mask keep byte1");
        vpeek_exp(17'h00306, 8'h33, "mask wr byte2");
        vpeek_exp(17'h00307, 8'h44, "mask wr byte3");

        // =================================================================
        // 7. Transparent writes
        // =================================================================
        vpoke(17'h00310, 8'hAA);
        fx_ctrl(8'h80);                    // transparent writes
        a0set(17'h00310, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // zero -> must NOT write
        fx_ctrl(8'h00);
        vpeek_exp(17'h00310, 8'hAA, "transp zero kept");
        fx_ctrl(8'h80);
        a0set(17'h00310, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'hBB);                 // nonzero -> writes
        fx_ctrl(8'h00);
        vpeek_exp(17'h00310, 8'hBB, "transp nonzero wr");

        // transparent CACHE write: zero bytes in cache stay transparent
        vpoke(17'h00318, 8'h91); vpoke(17'h00319, 8'h92);
        vpoke(17'h0031A, 8'h93); vpoke(17'h0031B, 8'h94);
        fx_cache(32'hDD00CC00);            // bytes 0,2 zero; 1=CC 3=DD
        fx_ctrl(8'hC0);                    // transp | cache write
        a0set(17'h00318, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);
        fx_ctrl(8'h00);
        vpeek_exp(17'h00318, 8'h91, "transp cache keep0");
        vpeek_exp(17'h00319, 8'hCC, "transp cache wr1");
        vpeek_exp(17'h0031A, 8'h93, "transp cache keep2");
        vpeek_exp(17'h0031B, 8'hDD, "transp cache wr3");

        // =================================================================
        // 8. 4-bit mode: nibble writes with nibble increment
        // =================================================================
        vpoke(17'h00320, 8'h00);
        fx_ctrl(8'h04);                    // 4-bit mode
        a0set(17'h00320, 4'h0, 1'b0, 1'b1, 1'b0);  // incr 0, nibble incr, nib=0
        vwr(5'h03, 8'h50);                 // high nibble <= 5
        vwr(5'h03, 8'h06);                 // low  nibble <= 6
        fx_ctrl(8'h00);
        vpeek_exp(17'h00320, 8'h56, "4bit nib writes");

        // 4-bit transparency: zero nibble is kept
        fx_ctrl(8'h84);                    // transp | 4-bit
        a0set(17'h00320, 4'h0, 1'b0, 1'b1, 1'b0);
        vwr(5'h03, 8'h00);                 // high nibble 0 -> kept
        vwr(5'h03, 8'h09);                 // low nibble <= 9
        fx_ctrl(8'h00);
        vpeek_exp(17'h00320, 8'h59, "4bit transp");

        // =================================================================
        // 9. One-byte cache cycling
        // =================================================================
        fx_cache(32'h44332211);
        fx_mult(8'h04);                    // cache byte index = 1
        fx_ctrl(8'h10);                    // one-byte cache cycling
        a0set(17'h00340, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'hFF);                 // value ignored; writes cache8
        fx_ctrl(8'h00);
        vpeek_exp(17'h00340, 8'h22, "one-byte cycling wr");

        fx_mult(8'h04);                    // index = 1 again
        fx_ctrl(8'h50);                    // one-byte | cache write -> 4x dup
        a0set(17'h00344, 4'h0, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h00);                 // mask 0 -> all four lanes
        fx_ctrl(8'h00);
        vpeek_exp(17'h00344, 8'h22, "dup byte0");
        vpeek_exp(17'h00345, 8'h22, "dup byte1");
        vpeek_exp(17'h00346, 8'h22, "dup byte2");
        vpeek_exp(17'h00347, 8'h22, "dup byte3");
        fx_mult(8'h00);

        // =================================================================
        // 10. Line draw helper: slope 0.5, octant 7 (ADDR1 +1, ADDR0 +320)
        // =================================================================
        fx_ctrl(8'h01);                    // line draw mode
        ctrl(6'd3, 1'b0);
        vwr(5'h09, 8'h00);                 // X incr low  (0.5 -> $0100)
        vwr(5'h0A, 8'h01);                 // X incr high (also resets subpixel)
        a1set(17'h01000, 4'h1, 1'b0, 1'b0, 1'b0);   // ADDR1 = start, +1
        a0set(17'h00000, 4'hE, 1'b0, 1'b0, 1'b0);   // ADDR0 incr = +320
        // 8 pixel writes -- FX_CTRL/addr regs untouched in between
        vwr(5'h04, 8'hC1); vwr(5'h04, 8'hC2);
        vwr(5'h04, 8'hC3); vwr(5'h04, 8'hC4);
        vwr(5'h04, 8'hC5); vwr(5'h04, 8'hC6);
        vwr(5'h04, 8'hC7); vwr(5'h04, 8'hC8);
        fx_ctrl(8'h00);
        vpeek_exp(17'h01000, 8'hC1, "line px0");
        vpeek_exp(17'h01141, 8'hC2, "line px1");
        vpeek_exp(17'h01142, 8'hC3, "line px2");
        vpeek_exp(17'h01283, 8'hC4, "line px3");
        vpeek_exp(17'h01284, 8'hC5, "line px4");
        vpeek_exp(17'h013C5, 8'hC6, "line px5");
        vpeek_exp(17'h013C6, 8'hC7, "line px6");
        vpeek_exp(17'h01507, 8'hC8, "line px7");

        // =================================================================
        // 11. Polygon filler helper
        // =================================================================
        fx_ctrl(8'h02);                    // polygon filler mode
        ctrl(6'd3, 1'b0);
        vwr(5'h09, 8'h00);                 // X (left) incr low:  -0.5
        vwr(5'h0A, 8'h7F);                 //  = 15'h7F00 (resets subpixel .5)
        vwr(5'h0B, 8'h00);                 // Y (right) incr low: +0.5
        vwr(5'h0C, 8'h01);
        a1set(17'h00000, 4'h1, 1'b0, 1'b0, 1'b0);   // ADDR1 incr +1
        a0set(17'h02000, 4'hE, 1'b0, 1'b0, 1'b0);   // ADDR0 = top row, +320
        ctrl(6'd4, 1'b0);
        vwr(5'h09, 8'd100);                // X1 = 100
        vwr(5'h0A, 8'h00);
        vwr(5'h0B, 8'd100);                // X2 = 100
        vwr(5'h0C, 8'h00);

        // ---- scanline 0 ----
        vrd(5'h04);                        // step: X1=100.0 X2=101.0, ADDR1=A0+100
        ctrl(6'd5, 1'b0);
        vrd(5'h0B); exp(rdv, 8'h02, "poly len0 (1 px)");
        vwr(5'h04, 8'hD1);                 // draw 1 pixel
        vrd(5'h03);                        // step X positions, ADDR0 += 320
        // ---- scanline 1 ----
        vrd(5'h04);                        // X1=99.0 X2=102.0, ADDR1=A0+99
        ctrl(6'd5, 1'b0);
        vrd(5'h0B); exp(rdv, 8'h66, "poly len1 (3 px, x1&3=3)");
        vrd(5'h0C); exp(rdv, 8'h00, "poly len1 high");
        vwr(5'h04, 8'hD2); vwr(5'h04, 8'hD2); vwr(5'h04, 8'hD2);
        fx_ctrl(8'h00);
        vpeek_exp(17'h02064, 8'hD1, "poly l0 px");
        vpeek_exp(17'h021A3, 8'hD2, "poly l1 px0");
        vpeek_exp(17'h021A4, 8'hD2, "poly l1 px1");
        vpeek_exp(17'h021A5, 8'hD2, "poly l1 px2");

        // =================================================================
        // 12. Affine helper (8bpp, 2x2 map, tiles 0/1)
        // =================================================================
        // map at $4000 (2x2): [0]=t0 [1]=t1 [2]=t1 [3]=t0
        vpoke(17'h04000, 8'h00); vpoke(17'h04001, 8'h01);
        vpoke(17'h04002, 8'h01); vpoke(17'h04003, 8'h00);
        // tile 0 row 0 at $4800: A0..A7; tile 1 row 0 at $4840: B0..B7
        for (i = 0; i < 8; i = i + 1) begin
            vpoke(17'h04800 + i, 8'hA0 + i[7:0]);
            vpoke(17'h04840 + i, 8'hB0 + i[7:0]);
        end

        fx_ctrl(8'h03);                    // affine mode, 8bpp
        ctrl(6'd2, 1'b0);
        vwr(5'h0A, 8'h24);                 // FX_TILEBASE = $4800>>11<<2, clip=0
        vwr(5'h0B, 8'h20);                 // FX_MAPBASE  = ($4000>>11)<<2 = $20, size=2x2
        ctrl(6'd3, 1'b0);
        vwr(5'h09, 8'h00);                 // X incr = +1.0 ($0200)
        vwr(5'h0A, 8'h02);
        vwr(5'h0B, 8'h00);                 // Y incr = 0
        vwr(5'h0C, 8'h00);
        ctrl(6'd5, 1'b0);
        vwr(5'h09, 8'h80);                 // X pos frac = .5
        vwr(5'h0A, 8'h80);                 // Y pos frac = .5
        ctrl(6'd4, 1'b0);
        vwr(5'h09, 8'h00);                 // X pos = 0
        vwr(5'h0A, 8'h00);
        vwr(5'h0B, 8'h00);                 // Y pos = 0
        vwr(5'h0C, 8'h00);

        // 16 reads: tile0 row0 cols 0-7, then tile1 row0 cols 0-7
        for (i = 0; i < 16; i = i + 1) begin
            vrd(5'h04);
            if (i < 8) exp(rdv, 8'hA0 + i[7:0], "affine tile0 walk");
            else       exp(rdv, 8'hB0 + (i[7:0] - 8'd8), "affine tile1 walk");
        end
        vrd(5'h04); exp(rdv, 8'hA0, "affine wrap to tile0");

        // clip disabled: X=108 -> map wraps -> tile1 col 4
        ctrl(6'd4, 1'b0);
        vwr(5'h09, 8'd108);
        vwr(5'h0A, 8'h00);
        vrd(5'h04); exp(rdv, 8'hB4, "affine wrap x=108");
        // clip enabled: X=108 outside map -> forced tile 0 col 4
        ctrl(6'd2, 1'b0);
        vwr(5'h0A, 8'h26);                 // FX_TILEBASE with clip=1
        ctrl(6'd4, 1'b0);
        vwr(5'h09, 8'd108);
        vwr(5'h0A, 8'h00);
        vrd(5'h04); exp(rdv, 8'hA4, "affine clip x=108");
        ctrl(6'd2, 1'b0);
        vwr(5'h0A, 8'h24);                 // clip off again
        fx_ctrl(8'h00);

        // =================================================================
        // 13. 16-bit hop: ADDR1 incr +4 alternates +1/+3
        // =================================================================
        for (i = 0; i < 16; i = i + 1) vpoke(17'h00500 + i, i[7:0]);
        fx_ctrl(8'h08);                    // 16-bit hop, mode 0
        a1set(17'h00500, 4'h3, 1'b0, 1'b0, 1'b0);   // ADDR1 = $500, incr +4
        vrd(5'h04); exp(rdv, 8'h00, "hop rd0");
        vrd(5'h04); exp(rdv, 8'h01, "hop rd1");
        vrd(5'h04); exp(rdv, 8'h04, "hop rd2");
        vrd(5'h04); exp(rdv, 8'h05, "hop rd3");
        vrd(5'h04); exp(rdv, 8'h08, "hop rd4");
        vrd(5'h04); exp(rdv, 8'h09, "hop rd5");
        fx_ctrl(8'h00);

        // =================================================================
        // 14. Regression: traditional DATA0/DATA1, FX off
        // =================================================================
        a0set(17'h00600, 4'h1, 1'b0, 1'b0, 1'b0);
        vwr(5'h03, 8'h11); vwr(5'h03, 8'h22); vwr(5'h03, 8'h33);
        a1set(17'h00602, 4'h1, 1'b1, 1'b0, 1'b0);  // ADDR1 decrementing
        vrd(5'h04); exp(rdv, 8'h33, "trad rd decr0");
        vrd(5'h04); exp(rdv, 8'h22, "trad rd decr1");
        vrd(5'h04); exp(rdv, 8'h11, "trad rd decr2");
        // ADDRx_H readback (incr/decr/nib fields)
        ctrl(6'd0, 1'b1);
        vrd(5'h02); exp(rdv, 8'h18, "ADDR1_H readback incr1|decr");

        // =================================================================
        // 15. 2-bit polygon poke mode (4-bit + 2-bit-polygon + poly filler):
        //     ADDR1_L write arms a read-modify-write of one 2-bit lane,
        //     lane selected by DATA1 write bits [7:6], data from cache8.
        // =================================================================
        vpoke(17'h00700, 8'h99);           // 10_01_10_01
        vpoke(17'h00701, 8'h00);
        fx_mult(8'h00);                    // cache byte index = 0
        fx_cache(32'h000000FF);            // cache8 = $FF (all lanes 11)
        a1set(17'h00700, 4'h0, 1'b0, 1'b0, 1'b0);   // ADDR1 base (mode 0!)
        fx_ctrl(8'h06);                    // poly filler + 4-bit mode
        ctrl(6'd2, 1'b0);
        vwr(5'h0A, 8'h01);                 // FX_TILEBASE: 2-bit polygon = 1
        ctrl(6'd0, 1'b1);                  // ADDRSEL=1
        vwr(5'h00, 8'h00);                 // ADDR1_L: lane addr $700, arm poke
        vwr(5'h04, 8'h40);                 // poke lane 01 <- cache8[5:4]=11
        ctrl(6'd0, 1'b1);
        vwr(5'h00, 8'h01);                 // re-arm: lane addr $701
        vwr(5'h04, 8'hC0);                 // poke lane 11 <- cache8[1:0]=11
        fx_ctrl(8'h00);
        ctrl(6'd2, 1'b0);
        vwr(5'h0A, 8'h00);                 // 2-bit polygon off again
        vpeek_exp(17'h00700, 8'hB9, "2bit poke lane01");   // 10_11_10_01
        vpeek_exp(17'h00701, 8'h03, "2bit poke lane11");   // 00_00_00_11

        // =================================================================
        // 16. Poly-mode DATA0 read bumps the cache byte index when one-byte
        //     cache cycling is on and cache fill is off.
        // =================================================================
        vpoke(17'h00720, 8'h00);
        fx_cache(32'h44332211);
        fx_mult(8'h00);                    // index = 0
        fx_ctrl(8'h12);                    // one-byte cycling + poly filler
        a0set(17'h00720, 4'h0, 1'b0, 1'b0, 1'b0);
        vrd(5'h03);                        // index 0 -> 1
        vrd(5'h03);                        // index 1 -> 2
        vwr(5'h03, 8'hFF);                 // one-byte write: cache8 = $33
        fx_ctrl(8'h00);
        vpeek_exp(17'h00720, 8'h33, "poly DATA0 index bump");

        // =================================================================
        // 17. MiSTer video-mode fallback: hsync must keep running in ALL
        //     output modes (0=off, 2=NTSC, 3=RGB -> VGA timing kept), and
        //     DC_VIDEO bit 3 (240p progressive) must read back.
        // =================================================================
        begin : vidmode
            integer m, e0;
            for (m = 0; m < 4; m = m + 1) begin
                ctrl(6'd0, 1'b0);
                vwr(5'h09, m[7:0]);        // DC_VIDEO: output mode m
                e0 = hs_edges;
                repeat (5000) @(posedge clk25);   // ~6 hsync periods
                if (hs_edges - e0 < 3) begin
                    $display("[FX  ] FAIL sync dead in output mode %0d", m);
                    errors = errors + 1;
                end
            end
            ctrl(6'd0, 1'b0);
            vwr(5'h09, 8'h09);             // bit3 (progressive) + mode 1
            vrd(5'h09);
            exp(rdv & 8'h7F, 8'h09, "DC_VIDEO bit3 readback");
            vwr(5'h09, 8'h01);             // back to plain VGA
            vrd(5'h09);
            exp(rdv & 8'h7F, 8'h01, "DC_VIDEO bit3 cleared");
        end

        if (errors == 0) $display("[FX  ] ALL TESTS PASS");
        else             $display("[FX  ] %0d ERRORS", errors);
        $finish;
    end

`ifdef FXDBG
    // debug: trace every internal-bus VRAM write
    always @(posedge clk25)
        if (u_vera.ib_do_access_r && u_vera.ib_write_r)
            $display("[FXDW] t=%0t wr @%05x = %02x (cw=%b ob=%b)",
                     $time, u_vera.ib_addr_r, u_vera.ib_wrdata_r,
                     u_vera.ib_cache_write_enabled_r,
                     u_vera.ib_one_byte_cache_cycling_r);
`endif

    // global timeout
    initial begin
        #80_000_000;
        $display("[FX  ] TIMEOUT");
        $finish;
    end

endmodule
