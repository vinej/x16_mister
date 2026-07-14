`timescale 1ns/1ps
// ============================================================================
// tb_cart.v -- cart RAM save-back end to end (jyv 2026-07-07): real
// ext_ram_sdram (+ behavioral sdram) + real cart_backer + a multi-block-
// capable mock hps slot.  32 KB image (64 sectors), restore in two 16 KB
// chunks through the loader port, then:
//
//   1. restore integrity: CPU reads through the cpu port match the image
//   2. CPU writes dirty sectors 5 and 20 (+1 outside the image window) ->
//      after SAVE_DELAY exactly TWO blocks are written back, contents =
//      image + the modified bytes; the out-of-window write saves nothing
//   3. no further saves without new writes
//   4. read-only remount: restore works, writes trigger NO save
// ============================================================================
module tb_cart;
    integer errors = 0;

    reg clk  = 0; always #62.5 clk  = ~clk;    // 8 MHz cpu_clk
    reg fclk = 0; always #5    fclk = ~fclk;   // 100 MHz sdram_clk
    reg res_n = 0;

    localparam [24:0] CART_BASE = 25'h480000;
    localparam integer NSEC = 64;              // 32 KB image

    // ---- CPU port ----
    reg         cs = 0, we = 0;
    reg  [24:0] ba = 25'd0;
    reg  [7:0]  wd = 8'd0;
    wire [7:0]  rd;
    wire        ready;

    // ---- backer <-> sdram wiring ----
    wire        crt_ld_wr;
    wire [24:0] crt_ld_addr;
    wire  [7:0] crt_ld_data;
    wire        bk_rd, bk_ack;
    wire [24:0] bk_addr;
    wire  [7:0] bk_rdata;
    wire        wr_snoop;
    wire [24:0] wr_snoop_addr;
    wire        dut_ld_busy;

    ext_ram_sdram dut (
        .clk(clk), .sdram_clk(fclk), .reset_n(res_n),
        .cs(cs), .we(we), .byte_addr(ba), .wr_data(wd),
        .rd_data(rd), .ready(ready),
        .ld_wr(crt_ld_wr), .ld_addr(crt_ld_addr), .ld_data(crt_ld_data),
        .ld_busy(dut_ld_busy),
        .bk_rd(bk_rd), .bk_addr(bk_addr), .bk_rdata(bk_rdata), .bk_ack(bk_ack),
        .wr_snoop(wr_snoop), .wr_snoop_addr(wr_snoop_addr),
        .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
        .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
        .SDRAM_DQML(), .SDRAM_DQMH()
    );

    // ---- mock hps slot 2 ----
    reg         img_mounted = 0, img_readonly = 0;
    reg  [63:0] img_size = 0;
    wire [31:0] sd_lba;
    wire  [5:0] sd_blk_cnt;
    wire        sd_rd, sd_wr;
    reg         sd_ack = 0;
    reg  [13:0] sd_buff_addr = 0;
    reg  [7:0]  sd_buff_dout = 0;
    wire [7:0]  sd_buff_din;
    reg         sd_buff_wr = 0;

    wire restoring;
    cart_backer #(.SAVE_DELAY(20000),            // 200 us in sim
                  .CART_BYTES(25'h8000)          // 32 KB region: fast wipe
    ) u_cart (
        .clk(fclk), .reset_n(res_n),
        .img_mounted(img_mounted), .img_readonly(img_readonly),
        .img_size(img_size),
        .sd_lba(sd_lba), .sd_blk_cnt(sd_blk_cnt),
        .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
        .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .rst_ld_wr(crt_ld_wr), .rst_ld_addr(crt_ld_addr),
        .rst_ld_data(crt_ld_data), .ld_busy(dut_ld_busy),
        .restoring(restoring),
        .bk_rd(bk_rd), .bk_addr(bk_addr), .bk_rdata(bk_rdata), .bk_ack(bk_ack),
        .wr_snoop(wr_snoop), .wr_snoop_addr(wr_snoop_addr)
    );

    // ---- 32 KB image + write capture ----
    reg [7:0] img    [0:32767];
    reg [7:0] wr_img [0:32767];
    integer   wr_blocks = 0;                   // sd_wr services counted
    function [7:0] pat(input integer i);
        pat = i[7:0] ^ {2'b01, i[13:8]};
    endfunction

    integer i;
    initial for (i = 0; i < 32768; i = i + 1) begin
        img[i]    = pat(i);
        wr_img[i] = 8'hEE;
    end

    // ---- mock service loop (multi-block reads, single-block writes) ----
    localparam BYTE_SPACING = 24;              // < ld-FIFO drain rate
    integer a, nb;
    initial forever begin
        @(posedge fclk);
        if (sd_rd) begin
            nb = (sd_blk_cnt + 1) * 512;
            repeat (200) @(posedge fclk);      // Main service latency
            sd_ack = 1;
            @(posedge fclk);
            for (a = 0; a < nb; a = a + 1) begin
                sd_buff_addr = a[13:0];
                sd_buff_dout = img[sd_lba*512 + a];
                sd_buff_wr   = 1;
                @(posedge fclk);
                sd_buff_wr   = 0;
                repeat (BYTE_SPACING-1) @(posedge fclk);
            end
            sd_ack = 0;
            repeat (20) @(posedge fclk);
        end else if (sd_wr) begin
            repeat (200) @(posedge fclk);
            sd_ack = 1;
            @(posedge fclk);
            for (a = 0; a < 512; a = a + 1) begin
                sd_buff_addr = a[13:0];
                repeat (3) @(posedge fclk);    // registered-q contract
                wr_img[sd_lba*512 + a] = sd_buff_din;
                @(posedge fclk);
            end
            wr_blocks = wr_blocks + 1;
            sd_ack = 0;
            repeat (20) @(posedge fclk);
        end
    end

    // ---- CPU-port tasks (r65c02 shapes, from tb_wfifo) ----
    task wr02(input [24:0] a2, input [7:0] d);
        begin
            cs <= 1; we <= 1; ba <= a2; wd <= d;
            @(negedge clk);
            @(posedge clk);
            cs <= 0; we <= 0;
            @(negedge clk);
            while (!ready) @(negedge clk);
            @(posedge clk);
        end
    endtask

    reg [7:0] rv;
    task rd02(input [24:0] a2);
        begin
            cs <= 1; we <= 0; ba <= a2;
            @(negedge clk);
            while (!ready) @(negedge clk);
            rv = rd;
            @(posedge clk);
            cs <= 0;
            @(posedge clk);
        end
    endtask

    task chk8(input [7:0] got, input [7:0] expct, input [255:0] what); begin
        if (got !== expct) begin
            if (errors < 12)
                $display("[CART] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    task chkd(input integer got, input integer expct, input [255:0] what); begin
        if (got !== expct) begin
            $display("[CART] FAIL %0s: got %0d exp %0d", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    integer k;
    initial begin
        repeat (100) @(posedge fclk);
        res_n = 1;
        repeat (6000) @(posedge fclk);         // SDRAM init window

        // ---- mount (during run; the during-reset case is covered by the
        //      same mnt_pend pattern proven in tb_nvram) ----
        img_size = 64'd32768; img_readonly = 0;
        @(posedge fclk) img_mounted = 1;
        @(posedge fclk) img_mounted = 0;
        repeat (4) @(posedge fclk);
        chk8({7'd0, restoring}, 8'h01, "restoring high on mount");

        // wait for the 2-chunk restore to finish
        wait (u_cart.state == u_cart.S_RST_REQ);
        wait (u_cart.state == u_cart.S_IDLE);
        repeat (4) @(posedge fclk);
        chk8({7'd0, restoring}, 8'h00, "restoring released");
        repeat (2000) @(posedge fclk);         // ld FIFO drain tail

        // ---- 1. restore integrity via CPU reads ----
        rd02(CART_BASE + 25'd0);      chk8(rv, pat(0),     "restore byte 0");
        rd02(CART_BASE + 25'd12345);  chk8(rv, pat(12345), "restore byte 12345");
        rd02(CART_BASE + 25'd32767);  chk8(rv, pat(32767), "restore byte 32767");

        // ---- 2. dirty two sectors (+ one outside the window) ----
        wr02(CART_BASE + 25'd5*512 + 25'd7,  8'hAA);
        wr02(CART_BASE + 25'd20*512 + 25'd0, 8'hBB);
        wr02(CART_BASE + 25'd100*512,        8'hCC);   // sector 100 > nsec
        // exactly two block write-backs expected
        wait (wr_blocks == 2);
        repeat (60000) @(posedge fclk);        // > SAVE_DELAY: no third block
        chkd(wr_blocks, 2, "exactly 2 dirty sectors saved");
        for (k = 0; k < 512; k = k + 1) begin
            chk8(wr_img[5*512+k],  (k == 7) ? 8'hAA : pat(5*512+k),  "sector 5");
            chk8(wr_img[20*512+k], (k == 0) ? 8'hBB : pat(20*512+k), "sector 20");
        end
        chk8(wr_img[0], 8'hEE, "sector 0 never written back");

        // ---- 3. read-back through the CPU port still consistent ----
        rd02(CART_BASE + 25'd5*512 + 25'd7); chk8(rv, 8'hAA, "cpu sees AA");

        // ---- 4. read-only remount: restore ok, no save ----
        img_readonly = 1;
        @(posedge fclk) img_mounted = 1;
        @(posedge fclk) img_mounted = 0;
        wait (u_cart.state == u_cart.S_RST_REQ);
        wait (u_cart.state == u_cart.S_IDLE);
        repeat (2000) @(posedge fclk);
        rd02(CART_BASE + 25'd5*512 + 25'd7);
        chk8(rv, pat(5*512+7), "ro restore overwrote AA");
        wr02(CART_BASE + 25'd9*512, 8'h55);
        repeat (80000) @(posedge fclk);
        chkd(wr_blocks, 2, "read-only: no save");

        // ---- 5. EJECT (OSD unmount = size-0 mount): wipe + reboot hold ----
        img_readonly = 0; img_size = 64'd0;
        @(posedge fclk) img_mounted = 1;
        @(posedge fclk) img_mounted = 0;
        repeat (4) @(posedge fclk);
        chk8({7'd0, restoring}, 8'h01, "restoring high during wipe");
        wait (u_cart.state == u_cart.S_WIPE);
        wait (u_cart.state == u_cart.S_IDLE);
        repeat (2000) @(posedge fclk);             // ld FIFO drain tail
        chk8({7'd0, restoring}, 8'h00, "restoring released after wipe");
        rd02(CART_BASE + 25'd0);            chk8(rv, 8'h00, "wiped byte 0");
        rd02(CART_BASE + 25'd5*512 + 25'd7);chk8(rv, 8'h00, "wiped byte 5*512+7");
        rd02(CART_BASE + 25'd32767);        chk8(rv, 8'h00, "wiped byte 32767");
        wr02(CART_BASE + 25'd3*512, 8'h44);        // write after eject
        repeat (80000) @(posedge fclk);
        chkd(wr_blocks, 2, "no save after eject");

        if (errors == 0) $display("[CART] ALL TESTS PASS");
        else             $display("[CART] %0d ERRORS", errors);
        $finish;
    end

    initial begin
        #80_000_000;
        $display("[CART] TIMEOUT");
        $finish;
    end
endmodule
