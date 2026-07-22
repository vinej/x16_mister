`timescale 1ns/1ps
// ============================================================================
// tb_blit.v -- SDRAM->SDRAM save-under blit ($9F69-$6F).
//
// bitmap_regs + ext_ram_sdram + sdram_sim, wired like x16.sv.  Writes a pattern
// to framebuffer region A, blits A->B in SDRAM, polls busy, then reads region B
// back through $9F65 and checks it matches -- including a different-alignment
// case (A even, B odd) to prove the byte-wise planar copy.
// ============================================================================
module tb_blit;
    reg clk  = 0; always #62.5 clk  = ~clk;    // 8 MHz  cpu_clk
    reg fclk = 0; always #5    fclk = ~fclk;   // 100 MHz sdram_clk
    reg res_n = 0;

    reg        cs = 0, rwn = 1;
    reg  [3:0] addr = 0;
    reg  [7:0] di = 0;
    wire [7:0] bmp_do;

    wire        fb_wr, fb_rd;
    wire [24:0] fb_addr;
    wire        blit_start, blit_done;
    wire [19:0] blit_src, blit_dst, blit_len;

    wire        cpu_rdy;
    wire  [7:0] sdram_rd;
    wire        sdr_ready;

    bitmap_regs #(.FB_BASE_WORD(24'h000000)) u_regs (
        .clk(clk), .reset_n(res_n),
        .cs(cs), .rwn(rwn), .en(cpu_rdy), .addr(addr), .di(di), .do_o(bmp_do),
        .master_en(1'b1),
        .bmp_enable(), .bmp_mode(), .bmp_passthru(),
        .fb_wr_sel(fb_wr), .fb_rd_sel(fb_rd), .fb_addr(fb_addr),
        .pal_we(), .pal_idx(), .pal_data(),
        .blit_start(blit_start), .blit_src(blit_src), .blit_dst(blit_dst),
        .blit_len(blit_len), .blit_done(blit_done)
    );

    wire ext_cs = fb_wr | fb_rd;
    wire ext_we = ext_cs & ~rwn;

    ext_ram_sdram #(.FB_BASE_WORD(24'h000000)) u_sdr (
        .clk(clk), .sdram_clk(fclk), .reset_n(res_n),
        .cs(ext_cs), .we(ext_we), .byte_addr(fb_addr), .wr_data(di),
        .rd_data(sdram_rd), .ready(sdr_ready),
        .ld_wr(1'b0), .ld_addr(25'd0), .ld_data(8'd0), .ld_busy(),
        .bk_rd(1'b0), .bk_addr(25'd0), .bk_rdata(), .bk_ack(),
        .wr_snoop(), .wr_snoop_addr(),
        .fb_go(1'b0), .fb_base(24'd0), .fb_len(11'd0),
        .fb_valid(), .fb_word(), .fb_done(),
        .blit_start(blit_start), .blit_src(blit_src), .blit_dst(blit_dst),
        .blit_len(blit_len), .blit_done(blit_done),
        .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
        .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
        .SDRAM_DQML(), .SDRAM_DQMH()
    );

    assign cpu_rdy = sdr_ready;
    wire [7:0] cpu_di = fb_rd ? sdram_rd : bmp_do;

    localparam CTRL=0, DATA=5, ADDRL=2, ADDRM=3, ADDRH=4,
               BDSTL=9, BDSTM=10, BDSTH=11, BLENL=12, BLENM=13, BLENH=14, BCTRL=15;

    function [7:0] pix(input integer i); pix = (i*13 + 7) & 8'hFF; endfunction

    task wr(input [3:0] a, input [7:0] d);
        begin
            cs<=1; rwn<=0; addr<=a; di<=d;
            @(negedge clk); while (!cpu_rdy) @(negedge clk); @(posedge clk);
            cs<=0; rwn<=1;
        end
    endtask
    task rd(input [3:0] a, output [7:0] q);
        begin
            cs<=1; rwn<=1; addr<=a;
            @(negedge clk); while (!cpu_rdy) @(negedge clk);
            q = cpu_di; @(posedge clk); cs<=0;
        end
    endtask
    task set_ptr(input [19:0] p);
        begin wr(ADDRL,p[7:0]); wr(ADDRM,p[15:8]); wr(ADDRH,{4'b0,p[19:16]}); end
    endtask

    integer i, errors, total, guard;
    reg [7:0] got;

    // fill region `base` with pix(0..n-1) then let the write FIFO drain
    task fill_region(input [19:0] base, input integer n);
        begin
            set_ptr(base);
            for (i = 0; i < n; i = i + 1) wr(DATA, pix(i));
            repeat (60) @(posedge clk);          // drain FIFO before the blit reads SDRAM
        end
    endtask

    task do_blit(input [19:0] src, input [19:0] dst, input [19:0] len);
        begin
            set_ptr(src);                        // DATA pointer = blit source
            wr(BDSTL,dst[7:0]); wr(BDSTM,dst[15:8]); wr(BDSTH,{4'b0,dst[19:16]});
            wr(BLENL,len[7:0]); wr(BLENM,len[15:8]); wr(BLENH,{4'b0,len[19:16]});
            wr(BCTRL, 8'h01);                    // start
            guard = 0;
            rd(BCTRL, got);
            while (got[0] && guard < 100000) begin rd(BCTRL, got); guard = guard + 1; end
        end
    endtask

    // check region `base` reads back pix(0..n-1)
    task check_region(input [19:0] base, input integer n, input [127:0] name);
        begin
            set_ptr(base);
            for (i = 0; i < n; i = i + 1) begin
                rd(DATA, got);
                if (got !== pix(i)) begin
                    errors = errors + 1;
                    if (errors <= 6) $display("[BLIT] %0s @%0d got %02x want %02x",
                                              name, i, got, pix(i));
                end
            end
        end
    endtask

    initial begin
        total = 0;
        repeat (4) @(posedge clk);
        res_n = 1;
        repeat (600) @(posedge fclk);

        // TEST1: aligned blit  A=$0100 -> B=$1000, 128 bytes
        errors = 0;
        fill_region(19'h00100, 128);
        do_blit(19'h00100, 19'h01000, 19'd128);
        check_region(19'h01000, 128, "aligned");
        $display("[BLIT] TEST1 aligned A->B (128B): %s", errors ? "FAIL" : "PASS");
        total = total + errors;

        // TEST2: busy actually asserted (guard didn't just spin 0)
        $display("[BLIT] TEST2 busy observed: %s", (guard > 0) ? "PASS" : "FAIL");
        if (guard == 0) total = total + 1;

        // TEST3: different alignment  A=$0200 (even) -> B=$2001 (odd), 96 bytes
        errors = 0;
        fill_region(19'h00200, 96);
        do_blit(19'h00200, 19'h02001, 19'd96);
        check_region(19'h02001, 96, "odd-dst");
        $display("[BLIT] TEST3 planar A(even)->B(odd) (96B): %s", errors ? "FAIL" : "PASS");
        total = total + errors;

        // TEST4: 20-bit reach -- blit to a HIGH dst (>512 KB, bit19=1) + read back
        errors = 0;
        fill_region(20'h00400, 64);
        do_blit(20'h00400, 20'h90000, 20'd64);
        check_region(20'h90000, 64, "hi-dst");
        $display("[BLIT] TEST4 20-bit dst $90000 (64B): %s", errors ? "FAIL" : "PASS");
        total = total + errors;

        if (total == 0) $display("[BLIT] ALL OK");
        else            $display("[BLIT] FAILED (%0d)", total);
        $finish;
    end

    initial begin #20000000; $display("[BLIT] TIMEOUT"); $finish; end
endmodule
