`timescale 1ns/1ps
// ============================================================================
// tb_cart.v -- typed X16 .crt cartridge parse (2026-07-22 rework).
//
// Builds a real .crt image (CX16 CARTRIDGE header + bank_info[] + sparse 16 KB
// bank blocks) and mounts it through cart_backer.  Verifies:
//
//   1. sparse placement: bank 32 (ROM) lands at CART_BASE+0, bank 34 (RAM)
//      lands at CART_BASE+2*16K -- proving the NONE gap at bank 33 is SKIPPED
//      (if it were not, bank 34 would land at CART_BASE+16K)
//   2. cart_wmask from bank_info: RAM/NVRAM banks writable, ROM/NONE not
//   3. magic-gated: a non-.crt (broken magic) parses nothing, mask = 0
//
// The old raw-image + save-back tests are gone: Mount Cartridge is now typed
// and read-only on disk (RAM/NVRAM writes are volatile in SDRAM); the per-bank
// CPU write gate lives in x16.sv (cart_wmask -> ext_sdram_we), not here.
// ============================================================================
module tb_cart;
    integer errors = 0;

    reg clk  = 0; always #62.5 clk  = ~clk;    // 8 MHz cpu_clk
    reg fclk = 0; always #5    fclk = ~fclk;   // 100 MHz sdram_clk
    reg res_n = 0;

    localparam [24:0] CART_BASE = 25'h480000;
    localparam integer BANK = 16384;

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
    reg         typed_mode = 1;                // 1 = parse .crt, 0 = raw all-RAM
    reg  [63:0] img_size = 0;
    wire [31:0] sd_lba;
    wire  [5:0] sd_blk_cnt;
    wire        sd_rd, sd_wr;
    reg         sd_ack = 0;
    reg  [13:0] sd_buff_addr = 0;
    reg  [7:0]  sd_buff_dout = 0;
    wire [7:0]  sd_buff_din;
    reg         sd_buff_wr = 0;

    wire         restoring;
    wire [255:0] cart_wmask;

    cart_backer #(.SAVE_DELAY(20000),
                  .CART_BYTES(25'hC000)          // 48 KB = banks 32-34
    ) u_cart (
        .clk(fclk), .reset_n(res_n),
        .img_mounted(img_mounted), .img_readonly(img_readonly),
        .img_size(img_size), .typed_mode(typed_mode),
        .sd_lba(sd_lba), .sd_blk_cnt(sd_blk_cnt),
        .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
        .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .rst_ld_wr(crt_ld_wr), .rst_ld_addr(crt_ld_addr),
        .rst_ld_data(crt_ld_data), .ld_busy(dut_ld_busy),
        .restoring(restoring),
        .bk_rd(bk_rd), .bk_addr(bk_addr), .bk_rdata(bk_rdata), .bk_ack(bk_ack),
        .wr_snoop(wr_snoop), .wr_snoop_addr(wr_snoop_addr),
        .cart_wmask(cart_wmask)
    );

    // ---- the .crt image (65 sectors = 33280 bytes: 480 hdr + 2*16K banks) ----
    localparam integer IMG_BYTES = 33280;
    reg [7:0] img [0:65535];
    function [7:0] pat32(input integer i); pat32 = i[7:0] ^ 8'h32; endfunction
    function [7:0] pat34(input integer i); pat34 = i[7:0] ^ 8'h34; endfunction
    function [7:0] rawp (input integer i); rawp  = i[7:0] ^ 8'h5A; endfunction

    // raw all-RAM image: a plain linear ramp, no header (typed_mode=0)
    task build_raw; integer k; begin
        for (k = 0; k < 65536; k = k + 1) img[k] = rawp(k);
    end endtask

    // magic "CX16 CARTRIDGE\r\n"
    reg [7:0] magic [0:15];
    integer i;
    task build_crt(input corrupt_magic);
        begin
            for (i = 0; i < 65536; i = i + 1) img[i] = 8'h00;
            magic[0]=8'h43; magic[1]=8'h58; magic[2]=8'h31; magic[3]=8'h36;
            magic[4]=8'h20; magic[5]=8'h43; magic[6]=8'h41; magic[7]=8'h52;
            magic[8]=8'h54; magic[9]=8'h52; magic[10]=8'h49; magic[11]=8'h44;
            magic[12]=8'h47; magic[13]=8'h45; magic[14]=8'h0D; magic[15]=8'h0A;
            for (i = 0; i < 16; i = i + 1) img[i] = magic[i];
            if (corrupt_magic) img[3] = 8'h00;          // break the magic
            // bank_info at offset 256: bank32=ROM(1) bank33=NONE(0) bank34=RAM(3)
            img[256+0] = 8'd1;
            img[256+1] = 8'd0;
            img[256+2] = 8'd3;
            // data (offset 480): bank32 block then bank34 block (NONE skipped)
            for (i = 0; i < BANK; i = i + 1) img[480 + i]        = pat32(i);
            for (i = 0; i < BANK; i = i + 1) img[480 + BANK + i] = pat34(i);
        end
    endtask

    // ---- mock service loop: multi-block reads (writes never happen: ro) ----
    localparam BYTE_SPACING = 24;
    integer a, nb;
    initial forever begin
        @(posedge fclk);
        if (sd_rd) begin
            nb = (sd_blk_cnt + 1) * 512;
            repeat (200) @(posedge fclk);
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
        end
    end

    // ---- CPU read task ----
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

    task mount_wait; begin
        @(posedge fclk) img_mounted = 1;
        @(posedge fclk) img_mounted = 0;
        wait (u_cart.state == u_cart.S_RST_REQ);
        wait (u_cart.state == u_cart.S_IDLE);
        repeat (3000) @(posedge fclk);        // ld FIFO drain tail
    end endtask

    initial begin
        repeat (100) @(posedge fclk);
        res_n = 1;
        repeat (6000) @(posedge fclk);        // SDRAM init window

        // ================= 1. typed .crt =================
        typed_mode = 1;
        build_crt(1'b0);
        img_size = IMG_BYTES; img_readonly = 0;
        mount_wait;

        // placement: bank 32 -> CART_BASE+0, bank 34 -> CART_BASE+2*BANK
        rd02(CART_BASE + 25'd0);            chk8(rv, pat32(0),        "b32 @0");
        rd02(CART_BASE + (BANK-1));         chk8(rv, pat32(BANK-1),   "b32 @16383");
        rd02(CART_BASE + 2*BANK);           chk8(rv, pat34(0),        "b34 @2*16K");
        rd02(CART_BASE + 2*BANK + (BANK-1));chk8(rv, pat34(BANK-1),   "b34 end");
        // the NONE gap (bank 33 @ CART_BASE+BANK) must NOT hold bank-34 data
        rd02(CART_BASE + BANK);
        if (rv === pat34(0)) begin
            $display("[CART] FAIL gap: bank 34 was shifted into the NONE bank 33");
            errors = errors + 1;
        end

        // mask: bank 34 (RAM) writable; bank 32 (ROM) + 33 (NONE) not
        chk8({7'd0, cart_wmask[34]}, 8'h01, "wmask[34]=RAM");
        chk8({7'd0, cart_wmask[32]}, 8'h00, "wmask[32]=ROM");
        chk8({7'd0, cart_wmask[33]}, 8'h00, "wmask[33]=NONE");
        chk8({7'd0, cart_wmask[35]}, 8'h00, "wmask[35]=absent");

        // ================= 2. non-.crt (bad magic) =================
        build_crt(1'b1);                      // corrupt the magic
        img_size = IMG_BYTES;
        mount_wait;
        // nothing parsed -> mask fully clear
        if (cart_wmask !== 256'd0) begin
            $display("[CART] FAIL bad-magic: mask not clear (%h)", cart_wmask[47:32]);
            errors = errors + 1;
        end

        // ================= 3. raw all-RAM (typed_mode = 0) =================
        typed_mode = 0;
        build_raw;
        img_size = 3*BANK; img_readonly = 1;      // ro -> no save sweep in the TB
        mount_wait;
        // LINEAR placement: file offset X lands at CART_BASE+X (no bank skip)
        rd02(CART_BASE + 25'd0);           chk8(rv, rawp(0),          "raw @0");
        rd02(CART_BASE + 25'd100);         chk8(rv, rawp(100),        "raw @100");
        rd02(CART_BASE + BANK);            chk8(rv, rawp(BANK),       "raw @16K linear");
        rd02(CART_BASE + 2*BANK + 25'd7);  chk8(rv, rawp(2*BANK+7),   "raw @2bank+7");
        // mask: EVERY cart bank 32-255 writable RAM; system banks 0-31 not
        chk8({7'd0, cart_wmask[32]},  8'h01, "raw wmask[32]=RAM");
        chk8({7'd0, cart_wmask[34]},  8'h01, "raw wmask[34]=RAM");
        chk8({7'd0, cart_wmask[255]}, 8'h01, "raw wmask[255]=RAM");
        chk8({7'd0, cart_wmask[31]},  8'h00, "raw wmask[31]=ROM");
        chk8({7'd0, cart_wmask[0]},   8'h00, "raw wmask[0]=ROM");

        if (errors == 0) $display("[CART] ALL TESTS PASS");
        else             $display("[CART] %0d ERRORS", errors);
        $finish;
    end

    initial begin
        #120_000_000;
        $display("[CART] TIMEOUT");
        $finish;
    end
endmodule
