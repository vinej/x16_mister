`timescale 1ns/1ps
// ============================================================================
// BUG3 (SAVE corrupts image) write-path testbench.
//
// Real r65c02 + cpu_clk SPI master + real sd_card (fixed ss-gating), image
// mounted from the start.  The CPU runs sdwrite.s = the DOS SAVE access
// pattern with the faithful R49 write flow (send_cmd deselect/select/
// wait_ready, CMD24, token, 512 bytes, dummy CRC, IMMEDIATE deselect).
//
// The hps model serves READS (fills from a 4-sector image, WITH the sd_card
// prefetch traffic) and WRITES (acks after W_LEAD, reads sd_buff_din with
// hps_io cadence, commits into the image).  R_LEAD/W_LEAD model Main's
// service latency -- sweep them to hunt request/serve collisions.
//
// PASS = image sectors 0/1 hold exactly the written patterns, read-backs
// match, R1 responses = 00, no program errors.
// ============================================================================
module tb_write2 #(
    parameter integer R_LEAD       = 200,   // clk cycles sd_rd -> ack
    parameter integer W_LEAD       = 200,   // clk cycles sd_wr -> ack
    parameter integer BYTE_SPACING = 16,
    parameter integer ACK_DROP     = 8
);
    reg clk = 0; always #5 clk = ~clk;
    reg res_n = 0;

    wire        r_w_n, sync;
    wire [15:0] addr;
    wire [7:0]  dout;
    wire [15:0] pc;
    reg  [7:0]  din;

    // ---- memory ----
    reg [7:0] ram [0:32767];
    reg [7:0] rom [0:8191];
    integer i, j;
    initial begin
        $readmemh("sdwrite2.hex", rom);
        for (i=0;i<32768;i=i+1) ram[i]=0;
    end


    // ---- HiRAM $A000-$BFFF bank0: EXACT replica of ext_ram_bram.sv g_fast
    //      (posedge write, NEGEDGE read register, always ready) ----
    // the REAL ext_ram_bram module (posedge read + 1-cycle stall after the
    // BUG3 fix), wired like x16.sv: bank 0, hiram_ready into cpu_rdy
    wire hi_ram_cs = (addr[15:13] == 3'b101);      // $A000-$BFFF
    wire [7:0] hiram_rd;
    wire       hiram_ready;
    ext_ram_bram u_hiram_bram (
        .clk     (clk),
        .cs      (hi_ram_cs),
        .we      (hi_ram_cs & ~r_w_n),
        .bank    (8'd0),
        .addr    (addr[12:0]),
        .wr_data (dout),
        .rd_data (hiram_rd),
        .ready   (hiram_ready)
    );

    wire acc_9f3e = (addr == 16'h9F3E);
    wire acc_9f3f = (addr == 16'h9F3F);

    // ---- cpu_rdy (from x16.sv) ----
    wire vera_read = (acc_9f3e | acc_9f3f) & r_w_n;
    reg [1:0] vera_read_stall = 0;
    always @(posedge clk or negedge res_n)
        if (!res_n) vera_read_stall <= 0;
        else if (vera_read) begin if (vera_read_stall!=2'd3) vera_read_stall <= vera_read_stall+2'd1; end
        else vera_read_stall <= 0;

    // ---- the cpu_clk SPI master: the REAL RTL (rtl/spi_sd_master.sv, with
    //      the 512-byte write FIFO = the BUG3 fix) ----
    wire spi_stall;
    wire cpu_rdy_base = (~vera_read | (vera_read_stall >= 2'd2)) & hiram_ready;
    wire cpu_rdy = cpu_rdy_base & ~spi_stall;
    wire rd_9f3e = cpu_rdy &  r_w_n & acc_9f3e;
    wire miso;
    wire m_sck, m_mosi, m_sel;
    wire [7:0] m_data, m_status;

    spi_sd_master #(.M_HALF(3'd3)) u_spimaster (
        .clk(clk), .rst_n(res_n),
        .rd_data(rd_9f3e),
        .cpu_do(dout), .acc_data(acc_9f3e), .acc_ctrl(acc_9f3f), .cpu_rwn(r_w_n),
        .stall(spi_stall), .data_q(m_data), .status_q(m_status),
        .sck(m_sck), .mosi(m_mosi), .sel(m_sel), .miso(miso)
    );

    always @(*) begin
        if      (addr < 16'h8000) din = ram[addr[14:0]];
        else if (hi_ram_cs)       din = hiram_rd;      // like x16.sv hi_ram_cs -> ext_ram_data
        else if (acc_9f3e)        din = m_data;
        else if (acc_9f3f)        din = m_status;
        else if (addr >= 16'hE000)din = rom[addr[12:0]];
        else                      din = 8'h00;
    end
    always @(posedge clk) if (~r_w_n && addr < 16'h8000) ram[addr[14:0]] <= dout;

    r65c02_wrap u_cpu(.clk(clk), .enable(cpu_rdy), .res_n(res_n), .irq_n(1'b1),
        .nmi_n(1'b1), .rdy(1'b1), .r_w_n(r_w_n), .sync(sync), .addr(addr),
        .din(din), .dout(dout), .pc(pc));

    // ---- mount (fixed ss-gating, mounted before CPU starts) ----
    reg         img_mounted = 0;
    reg  [63:0] img_size    = 64'd0;
    reg         vsd_sel     = 0;
    always @(posedge clk) if (img_mounted) vsd_sel <= |img_size;

    // ---- sd_card ----
    wire [31:0] sd_lba; wire sd_rd, sd_wr; reg sd_ack=0;
    reg [8:0] sd_buff_addr=0; reg [7:0] sd_buff_dout=0; wire [7:0] sd_buff_din; reg sd_buff_wr=0;
    wire [7:0] dbg_bufdout, dbg_mem510; wire [8:0] dbg_bufptr;
    wire [1:0] dbg_spibuf, dbg_sdbuf; wire [2:0] dbg_wrstate, dbg_rdstate;
    sd_card dut(.clk_sys(clk), .reset(~res_n), .sdhc(1'b1), .img_mounted(img_mounted),
        .img_size(img_size), .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .clk_spi(clk), .ss(~m_sel | ~vsd_sel), .sck(m_sck), .mosi(m_mosi), .miso(miso),
        .dbg_bufdout(dbg_bufdout), .dbg_bufptr(dbg_bufptr), .dbg_spibuf(dbg_spibuf),
        .dbg_sdbuf(dbg_sdbuf), .dbg_wrstate(dbg_wrstate), .dbg_rdstate(dbg_rdstate), .dbg_mem510(dbg_mem510));

    // ---- the 4-sector "image" ----
    reg [7:0] image [0:3][0:511];
    initial for (i=0;i<4;i=i+1) for (j=0;j<512;j=j+1) image[i][j] = (j[7:0] ^ (i[7:0]*16 + 3));

    // expected pattern (mirror sdwrite2.s fill of the HiRAM sector buffer)
    function [7:0] pattA(input integer n);
        pattA = (n < 256) ? (n[7:0] ^ 8'h5A) : (n[7:0] ^ 8'hA5); endfunction

    // ========================================================================
    // hps model: serves READS (accurate fill) and WRITES (accurate readout)
    // ========================================================================
    localparam FS_IDLE=0, FS_RLEAD=1, FS_FILL=2, FS_WLEAD=3, FS_WRD=4, FS_DRAIN=5;
    reg [2:0]  fstate = FS_IDLE;
    reg [2:0]  b_wr = 0;
    integer    bidx = 0, spc = 0, leadc = 0, dropc = 0;
    reg [31:0] serve_lba = 0;
    reg        serve_wr  = 0;

    always @(posedge clk) begin
        sd_buff_wr <= b_wr[0];
        if (b_wr[2] && ~(&sd_buff_addr) && fstate==FS_FILL) sd_buff_addr <= sd_buff_addr + 1'b1;
        b_wr <= (b_wr << 1);

        if (!res_n) begin
            fstate<=FS_IDLE; b_wr<=0; sd_ack<=0; sd_buff_wr<=0;
            sd_buff_addr<=0; bidx<=0; spc<=0; leadc<=0; dropc<=0;
        end else begin
            case (fstate)
                FS_IDLE:
                    if (sd_rd)      begin leadc<=0; fstate<=FS_RLEAD; end
                    else if (sd_wr) begin leadc<=0; fstate<=FS_WLEAD; end

                // ---------- serve a read: fill the buffer ----------
                FS_RLEAD:
                    if (leadc >= R_LEAD) begin
                        serve_lba    <= sd_lba;
                        serve_wr     <= 0;
                        sd_ack       <= 1'b1;
                        sd_buff_addr <= 0;
                        sd_buff_dout <= image[sd_lba[1:0]][0];
                        b_wr         <= 3'b001;
                        bidx         <= 1;
                        spc          <= 0;
                        fstate       <= FS_FILL;
                        $display("[HPS ] t=%0t READ  serve lba=%0d (sd_buf=%0d spi_buf=%0d rdst=%0d wrst=%0d)",
                                 $time, sd_lba, dbg_sdbuf, dbg_spibuf, dbg_rdstate, dbg_wrstate);
                    end else leadc <= leadc + 1;

                FS_FILL:
                    if (bidx < 512) begin
                        if (spc >= BYTE_SPACING) begin
                            spc          <= 0;
                            sd_buff_dout <= image[serve_lba[1:0]][bidx];
                            b_wr         <= 3'b001;
                            bidx         <= bidx + 1;
                        end else spc <= spc + 1;
                    end else begin dropc<=0; fstate<=FS_DRAIN; end

                // ---------- serve a write: read the buffer out ----------
                FS_WLEAD:
                    if (leadc >= W_LEAD) begin
                        serve_lba    <= sd_lba;
                        serve_wr     <= 1;
                        sd_ack       <= 1'b1;
                        sd_buff_addr <= 0;
                        bidx         <= 0;
                        spc          <= 0;
                        fstate       <= FS_WRD;
                        $display("[HPS ] t=%0t WRITE serve lba=%0d (sd_buf=%0d spi_buf=%0d wrst=%0d)",
                                 $time, sd_lba, dbg_sdbuf, dbg_spibuf, dbg_wrstate);
                    end else leadc <= leadc + 1;

                FS_WRD:
                    if (bidx < 512) begin
                        if (spc >= BYTE_SPACING) begin
                            spc <= 0;
                            image[serve_lba[1:0]][bidx] <= sd_buff_din;  // capture at settled addr
                            sd_buff_addr <= sd_buff_addr + 1'b1;
                            bidx <= bidx + 1;
                        end else spc <= spc + 1;
                    end else begin dropc<=0; fstate<=FS_DRAIN; end

                FS_DRAIN:
                    if (dropc >= ACK_DROP) begin
                        sd_ack <= 1'b0;
                        fstate <= FS_IDLE;
                    end else dropc <= dropc + 1;
            endcase
        end
    end

    // ---- trace ----
    reg [1:0] last_sdbuf=3, last_spibuf=3; reg [2:0] last_wrs=7;
    always @(posedge clk) begin
        if (dut.sd_buf  !== last_sdbuf)  begin last_sdbuf  <= dut.sd_buf;  end
        if (dut.spi_buf !== last_spibuf) begin last_spibuf <= dut.spi_buf; end
        if (dbg_wrstate !== last_wrs) begin
            $display("[WRST ] t=%0t write_state -> %0d (spi_buf=%0d sd_buf=%0d sd_wr=%0b)", $time, dbg_wrstate, dut.spi_buf, dut.sd_buf, sd_wr);
            last_wrs <= dbg_wrstate;
        end
    end

    // ---- run + verify ----
    integer k, bad, badB, badsec;
    reg [7:0] exp;
    initial begin
        $display("[CFG ] R_LEAD=%0d W_LEAD=%0d BYTE_SPACING=%0d", R_LEAD, W_LEAD, BYTE_SPACING);
        // mount before releasing the CPU
        repeat(10) @(posedge clk);
        img_size <= 64'd104857600;
        img_mounted <= 1'b1; repeat(2) @(posedge clk); img_mounted <= 1'b0;
        repeat(8) @(posedge clk); res_n = 1;

        fork
            begin : t
                while (ram[0] != 8'haa) @(posedge clk);
                $display("[TBC ] DONE. R1: cmd24=%02x cmd17=%02x errc=%02x", ram[1], ram[2], ram[3]);

                badsec = 0; bad = 0; badB = 0;
                for (k=0;k<512;k=k+1) begin
                    if (image[0][k] !== pattA(k)) begin
                        if (bad<8) $display("[IMG0 MISMATCH] off=%0d got=%02x exp=%02x stale_prev=%s",
                            k, image[0][k], pattA(k), (k>0 && image[0][k]===pattA(k-1)) ? "YES" : "no");
                        bad=bad+1;
                        if (k>0 && image[0][k]===pattA(k-1)) badB=badB+1;
                    end
                end
                if (bad) begin badsec=badsec+1;
                    $display("[TBC ] image[0]: %0d/512 BAD (%0d of them = stale previous byte)", bad, badB); end
                else $display("[TBC ] image[0] == HiRAM pattern: OK");

                bad = 0; for (k=0;k<512;k=k+1) if (ram[16'h0600+k] !== pattA(k)) bad=bad+1;
                if (bad) begin badsec=badsec+1; $display("[TBC ] read-back lba0: %0d/512 BAD", bad); end
                else $display("[TBC ] read-back lba0: OK");

                if (badsec==0) $display("[TBC ] *** HIRAM WRITE TEST: ALL OK ***");
                else           $display("[TBC ] *** HIRAM WRITE TEST: FAILED (%0d checks bad) ***", badsec);
                disable w;
            end
            begin : w
                #120000000; $display("[TBC ] TIMEOUT (pc=%04x errc=%02x wrst=%0d)", pc, ram[4], dbg_wrstate); disable t;
            end
        join
        $finish;
    end
endmodule
