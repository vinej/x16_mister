`timescale 1ns/1ps
// ============================================================================
// tb_bmpio.v -- bitmap $9F65 DATA write + READ-BACK through the CPU port.
//
// Wires bitmap_regs + ext_ram_sdram + sdram_sim exactly like x16.sv routes
// them: a $9F65 write goes to SDRAM (write FIFO), a $9F65 read comes BACK from
// SDRAM (the CPU read path) -- the read-back that makes GUI save-under
// possible.  Verifies the planar round-trip and the shared auto-inc pointer.
// ============================================================================
module tb_bmpio;
    reg clk  = 0; always #62.5 clk  = ~clk;    // 8 MHz  cpu_clk
    reg fclk = 0; always #5    fclk = ~fclk;   // 100 MHz sdram_clk
    reg res_n = 0;

    // CPU register bus
    reg        cs = 0, rwn = 1;
    reg  [3:0] addr = 0;
    reg  [7:0] di = 0;
    wire [7:0] bmp_do;

    // bitmap_regs -> SDRAM routing (mirrors x16.sv)
    wire        fb_wr, fb_rd;
    wire [24:0] fb_addr;
    wire        bmp_enable, bmp_passthru;
    wire  [1:0] bmp_mode;

    wire        cpu_rdy;                    // = ext_ram_sdram.ready (only stall)
    wire  [7:0] sdram_rd;
    wire        sdr_ready;

    // FB_BASE_WORD=0 in sim so the framebuffer lands at low word indices.
    bitmap_regs #(.FB_BASE_WORD(24'h000000)) u_regs (
        .clk(clk), .reset_n(res_n),
        .cs(cs), .rwn(rwn), .en(cpu_rdy), .addr(addr), .di(di), .do_o(bmp_do),
        .master_en(1'b1),
        .bmp_enable(bmp_enable), .bmp_mode(bmp_mode), .bmp_passthru(bmp_passthru),
        .fb_wr_sel(fb_wr), .fb_rd_sel(fb_rd), .fb_addr(fb_addr),
        .pal_we(), .pal_idx(), .pal_data(),
        .blit_start(), .blit_src(), .blit_dst(), .blit_len(), .blit_done(1'b0)
    );

    wire        ext_cs = fb_wr | fb_rd;
    wire        ext_we = ext_cs & ~rwn;

    ext_ram_sdram u_sdr (
        .clk(clk), .sdram_clk(fclk), .reset_n(res_n),
        .cs(ext_cs), .we(ext_we), .byte_addr(fb_addr), .wr_data(di),
        .rd_data(sdram_rd), .ready(sdr_ready),
        .ld_wr(1'b0), .ld_addr(25'd0), .ld_data(8'd0), .ld_busy(),
        .bk_rd(1'b0), .bk_addr(25'd0), .bk_rdata(), .bk_ack(),
        .wr_snoop(), .wr_snoop_addr(),
        .fb_go(1'b0), .fb_base(24'd0), .fb_len(11'd0),
        .fb_valid(), .fb_word(), .fb_done(),
        .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
        .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
        .SDRAM_DQML(), .SDRAM_DQMH()
    );

    assign cpu_rdy = sdr_ready;                  // ext_ram_sdram is the only stall
    wire [7:0] cpu_di = fb_rd ? sdram_rd : bmp_do;   // x16.sv read mux for $9F6x

    localparam CTRL=4'd0, ID=4'd1, ADDRL=4'd2, ADDRM=4'd3, ADDRH=4'd4,
               DATA=4'd5, PALADR=4'd6, PALLO=4'd7, PALHI=4'd8;

    function [7:0] pix(input integer i); pix = (i*7 + 3) & 8'hFF; endfunction

    task wr(input [3:0] a, input [7:0] d);
        begin
            cs <= 1; rwn <= 0; addr <= a; di <= d;
            @(negedge clk);
            while (!cpu_rdy) @(negedge clk);
            @(posedge clk);
            cs <= 0; rwn <= 1;
        end
    endtask

    task rd(input [3:0] a, output [7:0] q);
        begin
            cs <= 1; rwn <= 1; addr <= a;
            @(negedge clk);
            while (!cpu_rdy) @(negedge clk);
            q = cpu_di;                          // valid while ready is high
            @(posedge clk);
            cs <= 0;
        end
    endtask

    task set_ptr(input [18:0] p);
        begin wr(ADDRL, p[7:0]); wr(ADDRM, p[15:8]); wr(ADDRH, {5'b0, p[18:16]}); end
    endtask

    integer i, errors, total;
    reg [7:0] got;

    initial begin
        total = 0;
        repeat (4) @(posedge clk);
        res_n = 1;
        repeat (600) @(posedge fclk);            // SDRAM self-init

        // ---- registers round-trip ----
        errors = 0;
        rd(ID, got);        if (got !== 8'hB5) begin errors=errors+1; $display("[BIO ] ID=%02x",got); end
        wr(CTRL, 8'b0000_1101);                  // enable, mode=2, passthru=1
        @(negedge clk);                          // let the write's NBA settle
        if (bmp_enable   !== 1'b1) begin errors=errors+1; $display("[BIO ] enable=%b",bmp_enable); end
        if (bmp_mode     !== 2'd2) begin errors=errors+1; $display("[BIO ] mode=%b",bmp_mode); end
        if (bmp_passthru !== 1'b1) begin errors=errors+1; $display("[BIO ] passthru=%b",bmp_passthru); end
        rd(CTRL, got);      if (got !== 8'b0000_1101) begin errors=errors+1; $display("[BIO ] CTRL=%02x",got); end
        $display("[BIO ] regs (ID/CTRL/passthru): %s", errors ? "FAIL" : "PASS");
        total = total + errors;

        // ---- planar write -> read-back at ptr 0 ----
        errors = 0;
        set_ptr(19'd0);
        for (i = 0; i < 64; i = i + 1) wr(DATA, pix(i));   // stream 64 bytes
        set_ptr(19'd0);
        for (i = 0; i < 64; i = i + 1) begin
            rd(DATA, got);                       // read-back auto-increments ptr
            if (got !== pix(i)) begin
                errors = errors + 1;
                if (errors <= 6) $display("[BIO ] @%0d got %02x want %02x", i, got, pix(i));
            end
        end
        $display("[BIO ] read-back @0 (64 bytes, planar even/odd): %s", errors ? "FAIL" : "PASS");
        total = total + errors;

        // ---- write/read-back at a non-zero pointer ----
        errors = 0;
        set_ptr(19'd1000);
        for (i = 0; i < 16; i = i + 1) wr(DATA, 8'hA0 + i[7:0]);
        set_ptr(19'd1000);
        for (i = 0; i < 16; i = i + 1) begin
            rd(DATA, got);
            if (got !== (8'hA0 + i[7:0])) begin errors=errors+1; $display("[BIO ] @1000+%0d got %02x",i,got); end
        end
        $display("[BIO ] read-back @1000 (16 bytes): %s", errors ? "FAIL" : "PASS");
        total = total + errors;

        if (total == 0) $display("[BIO ] ALL OK");
        else            $display("[BIO ] FAILED (%0d)", total);
        $finish;
    end

    initial begin #6000000; $display("[BIO ] TIMEOUT"); $finish; end
endmodule
