`timescale 1ns/1ps
// ============================================================================
// tb_spi4.v -- 4 MHz guest-SD SPI in the REAL x16.sv topology (jyv
// 2026-07-07, the "PCM video stutters" fix):
//
//   spi_sd_master @ cpu_clk 8 MHz, M_HALF=0 (sck = 4 MHz, default)
//     -> sck/mosi/ss 2-FF synced into 100 MHz (x16.sv hs_*_s)
//     -> sys/sd_card.sv @ 100 MHz (clk_sys = clk_spi, BUG3 topology)
//     -> miso back through the single capture FF (x16.sv m_miso_s)
//
// This is the timing-critical path the old tb_write/tb_cpu TBs do NOT model
// (they clock sd_card at cpu_clk).  Register-level driver, R49-shaped:
//   1. CMD17 lba0: R1 poll, $FE token poll, auto-tx burst of 512 reads --
//      every byte must match the image; report effective throughput.
//   2. CMD24 lba1: 512-byte fast-write burst (STA every 13 CPU cycles, no
//      polling -- the FIFO must absorb it at the 2 us/byte drain), then
//      CMD17 read-back of lba1 must return exactly what was written.
// ============================================================================
module tb_spi100;
    parameter integer MH = 0;            // sck = 8 MHz / (2*(MH+1))
    integer errors = 0;

    reg clk100 = 0;  always #5     clk100  = ~clk100;
    reg cpu_clk = 0; always #62.5  cpu_clk = ~cpu_clk;
    reg res_n = 0;

    // ---- CPU-side register interface (raw bus emulation) ----
    reg  [7:0] cpu_do  = 8'h00;
    reg        acc_data = 0, acc_ctrl = 0, cpu_rwn = 1, rd_data = 0;
    wire       spi_stall;
    wire [7:0] m_data, m_status;
    wire       m_sck, m_mosi, m_sel;
    wire       sd_card_miso;

    spi_sd_master100 u_spimaster (
        .clk_cpu(cpu_clk), .rst_cpu_n(res_n),
        .rd_data(rd_data),
        .cpu_do(cpu_do), .acc_data(acc_data), .acc_ctrl(acc_ctrl),
        .cpu_rwn(cpu_rwn),
        .stall(spi_stall), .data_q(m_data), .status_q(m_status),
        .clk(clk100), .rst_n(res_n),
        .sck(m_sck), .mosi(m_mosi), .sel(m_sel), .miso(sd_card_miso)
    );

    // ---- x16.sv topology: direct same-domain wiring ----
    reg  vsd_sel = 0;
    wire sd_ss_gated = ~m_sel | ~vsd_sel;

    // ---- sd_card @ 100 MHz ----
    reg         img_mounted = 0;
    reg  [63:0] img_size    = 64'd0;
    always @(posedge clk100) if (img_mounted) vsd_sel <= |img_size;

    wire [31:0] sd_lba; wire sd_rd, sd_wr; reg sd_ack = 0;
    reg  [8:0]  sd_buff_addr = 0; reg [7:0] sd_buff_dout = 0;
    wire [7:0]  sd_buff_din;      reg       sd_buff_wr = 0;

    sd_card dut(.clk_sys(clk100), .reset(~res_n), .sdhc(1'b1),
        .img_mounted(img_mounted), .img_size(img_size),
        .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
        .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .clk_spi(clk100), .ss(sd_ss_gated), .sck(m_sck),
        .mosi(m_mosi), .miso(sd_card_miso),
        .dbg_bufdout(), .dbg_bufptr(), .dbg_spibuf(),
        .dbg_sdbuf(), .dbg_wrstate(), .dbg_rdstate(), .dbg_mem510());

    // ---- 4-sector image + hps stub (tb_write's model @ 100 MHz) ----
    reg [7:0] image [0:3][0:511];
    integer i, j;
    initial for (i=0;i<4;i=i+1) for (j=0;j<512;j=j+1) image[i][j] = (j[7:0] ^ (i[7:0]*16 + 3));

    localparam R_LEAD = 2000, W_LEAD = 2000, BYTE_SPACING = 16, ACK_DROP = 80;
    localparam FS_IDLE=0, FS_RLEAD=1, FS_FILL=2, FS_WLEAD=3, FS_WRD=4, FS_DRAIN=5;
    reg [2:0]  fstate = FS_IDLE;
    reg [2:0]  b_wr = 0;
    integer    bidx = 0, spc = 0, leadc = 0, dropc = 0;
    reg [31:0] serve_lba = 0;

    always @(posedge clk100) begin
        sd_buff_wr <= b_wr[0];
        if (b_wr[2] && ~(&sd_buff_addr) && fstate==FS_FILL) sd_buff_addr <= sd_buff_addr + 1'b1;
        b_wr <= (b_wr << 1);
        if (!res_n) begin
            fstate<=FS_IDLE; b_wr<=0; sd_ack<=0; sd_buff_wr<=0;
            sd_buff_addr<=0; bidx<=0; spc<=0; leadc<=0; dropc<=0;
        end else case (fstate)
            FS_IDLE:
                if (sd_rd)      begin leadc<=0; fstate<=FS_RLEAD; end
                else if (sd_wr) begin leadc<=0; fstate<=FS_WLEAD; end
            FS_RLEAD:
                if (leadc >= R_LEAD) begin
                    serve_lba<=sd_lba; sd_ack<=1'b1; sd_buff_addr<=0;
                    sd_buff_dout<=image[sd_lba[1:0]][0];
                    b_wr<=3'b001; bidx<=1; spc<=0; fstate<=FS_FILL;
                end else leadc <= leadc + 1;
            FS_FILL:
                if (bidx < 512) begin
                    if (spc >= BYTE_SPACING) begin
                        spc<=0; sd_buff_dout<=image[serve_lba[1:0]][bidx];
                        b_wr<=3'b001; bidx<=bidx+1;
                    end else spc <= spc + 1;
                end else begin dropc<=0; fstate<=FS_DRAIN; end
            FS_WLEAD:
                if (leadc >= W_LEAD) begin
                    serve_lba<=sd_lba; sd_ack<=1'b1; sd_buff_addr<=0;
                    bidx<=0; spc<=0; fstate<=FS_WRD;
                end else leadc <= leadc + 1;
            FS_WRD:
                if (bidx < 512) begin
                    if (spc >= BYTE_SPACING) begin
                        spc<=0;
                        image[serve_lba[1:0]][bidx] <= sd_buff_din;
                        sd_buff_addr <= sd_buff_addr + 1'b1;
                        bidx <= bidx + 1;
                    end else spc <= spc + 1;
                end else begin dropc<=0; fstate<=FS_DRAIN; end
            FS_DRAIN:
                if (dropc >= ACK_DROP) begin sd_ack<=1'b0; fstate<=FS_IDLE; end
                else dropc <= dropc + 1;
        endcase
    end

    // ---- CPU-cycle-shaped register access tasks ----
    // write: bus condition held ~3 cpu cycles (the master edge-detects)
    task reg_wr(input is_ctrl, input [7:0] v); begin
        @(posedge cpu_clk);
        cpu_do = v; cpu_rwn = 0;
        if (is_ctrl) acc_ctrl = 1; else acc_data = 1;
        repeat (3) @(posedge cpu_clk);
        acc_ctrl = 0; acc_data = 0; cpu_rwn = 1;
        @(posedge cpu_clk);
    end endtask

    // committed $9F3E read: waits out the stall (like cpu_rdy), 1-cycle
    // rd_data commit, returns the byte.  Stall is sampled on NEGEDGES so
    // the combinational spi_stall has settled after each assignment (a
    // same-delta check races the continuous assign and commits early).
    reg [7:0] rb;
    task reg_rd; begin
        @(posedge cpu_clk);
        acc_data = 1; cpu_rwn = 1;
        @(negedge cpu_clk);
        while (spi_stall) @(negedge cpu_clk);
        @(posedge cpu_clk);
        rb = m_data;
        rd_data = 1;
        @(posedge cpu_clk);
        rd_data = 0; acc_data = 0;
    end endtask

    // spi_read (R49 slow poll): send FF, read result
    task spi_read; begin
        reg_wr(1'b0, 8'hFF);
        reg_rd;
    end endtask

    task chk8(input [7:0] got, input [7:0] expct, input [255:0] what); begin
        if (got !== expct) begin
            if (errors < 12)
                $display("[S100] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    // send a command frame, poll R1
    integer p;
    task sd_cmd(input [7:0] cmd, input [31:0] arg); begin
        reg_wr(1'b0, 8'hFF);
        reg_wr(1'b0, cmd);
        reg_wr(1'b0, arg[31:24]); reg_wr(1'b0, arg[23:16]);
        reg_wr(1'b0, arg[15:8]);  reg_wr(1'b0, arg[7:0]);
        reg_wr(1'b0, 8'h95);                       // CRC (ignored, SDHC post-init)
        rb = 8'hFF;
        for (p = 0; p < 16 && rb == 8'hFF; p = p + 1) spi_read;
    end endtask

    integer k;
    integer t0, t1;
    reg [7:0] wrpat [0:511];
    initial begin
        for (k = 0; k < 512; k = k + 1) wrpat[k] = k[7:0] ^ 8'h5A;

        repeat (10) @(posedge clk100);
        res_n = 1;
        repeat (40) @(posedge cpu_clk);

        // ============ 0. PRE-MOUNT boot probe (must not wedge) ============
        reg_wr(1'b1, 8'h01);                       // select (gated off: no image)
        sd_cmd(8'h40, 32'd0);                      // CMD0 -> no card -> FF
        chk8(rb, 8'hFF, "pre-mount CMD0 times out");
        reg_wr(1'b1, 8'h00);                       // deselect
        repeat (40) @(posedge cpu_clk);
        chk8(m_status & 8'h80, 8'h00, "master idle after probe");

        // ============ mount ============
        img_size = 64'd104857600;
        img_mounted = 1; repeat (4) @(posedge clk100); img_mounted = 0;
        repeat (40) @(posedge cpu_clk);

        reg_wr(1'b1, 8'h01);                       // ctrl: select, no auto-tx
        repeat (8) @(posedge cpu_clk);

        // ================= CMD17 read lba0, auto-tx burst =================
        sd_cmd(8'h51, 32'd0);
        chk8(rb, 8'h00, "CMD17 R1");
        rb = 8'hFF;
        for (p = 0; p < 20000 && rb != 8'hFE; p = p + 1) spi_read;
        chk8(rb, 8'hFE, "CMD17 token");

        reg_wr(1'b1, 8'h05);                       // auto-tx on
        reg_rd;                                    // dummy: starts byte 0
        t0 = $time;
        for (k = 0; k < 512; k = k + 1) begin
            reg_rd;
            chk8(rb, image[0][k], "read byte");
            repeat (6) @(posedge cpu_clk);         // ~MACPTR loop pacing
        end
        t1 = $time;
        $display("[S100] 512-byte auto-tx read in %0d ns  (~%0d KB/s)",
                 t1 - t0, (512 * 1000000) / ((t1 - t0) / 1000));
        reg_wr(1'b1, 8'h01);                       // auto-tx off
        spi_read; spi_read;                        // CRC bytes

        // ================= CMD24 write lba1, fast-write burst =============
        sd_cmd(8'h58, 32'd1);
        chk8(rb, 8'h00, "CMD24 R1");
        reg_wr(1'b0, 8'hFE);                       // data token
        // R49 fast write: STA $9F3E every ~13 CPU cycles, NO polling
        for (k = 0; k < 512; k = k + 1) begin
            @(posedge cpu_clk);
            cpu_do = wrpat[k]; cpu_rwn = 0; acc_data = 1;
            repeat (3) @(posedge cpu_clk);
            acc_data = 0; cpu_rwn = 1;
            repeat (9) @(posedge cpu_clk);         // total ~13 cycles/byte
        end
        spi_read; spi_read;                        // dummy CRC (drains queue first)
        spi_read;                                  // data response
        chk8(rb & 8'h1F, 8'h05, "CMD24 data response");
        // busy poll
        rb = 8'h00;
        for (p = 0; p < 20000 && rb != 8'hFF; p = p + 1) spi_read;
        chk8(rb, 8'hFF, "CMD24 busy done");
        // busy released => sd_card's hps commit is already complete
        wait (fstate == FS_IDLE);
        repeat (100) @(posedge cpu_clk);

        // ================= read back lba1 =================
        sd_cmd(8'h51, 32'd1);
        chk8(rb, 8'h00, "CMD17b R1");
        rb = 8'hFF;
        for (p = 0; p < 20000 && rb != 8'hFE; p = p + 1) spi_read;
        chk8(rb, 8'hFE, "CMD17b token");
        reg_wr(1'b1, 8'h05);
        reg_rd;                                    // dummy
        for (k = 0; k < 512; k = k + 1) begin
            reg_rd;
            chk8(rb, wrpat[k], "write-read-back byte");
        end
        reg_wr(1'b1, 8'h01);
        spi_read; spi_read;

        if (errors == 0) $display("[S100] ALL TESTS PASS (12.5 MHz sck, single-domain SPI)");
        else             $display("[S100] %0d ERRORS", errors);
        $finish;
    end

    initial begin
        #80_000_000;
        $display("[S100] TIMEOUT");
        $finish;
    end

endmodule
