`timescale 1ns/1ps
// ============================================================================
// BUG1 wedge-reproduction testbench.
//
// Same real r65c02 + cpu_clk SPI master + real sd_card + accurate hps_io fill
// model as tb_cpu_hps.v, but the image starts UNMOUNTED:
//
//   * the hps model IGNORES sd_rd until `mounted` (Main has no image to serve)
//   * the CPU program (sdwedge.s) probes the card pre-mount (CMD17, short
//     token poll, timeout, deselect) -- exactly what the X16 ROM does at boot
//   * the TB then "mounts" (img_mounted pulse, img_size set, hps model live;
//     any sd_rd left PENDING by the pre-mount probe gets served -- like Main)
//   * the CPU (no reset -- the user just types @$) re-inits and reads
//     lba 0 -> $0400 and lba 2048 -> $0600; the TB verifies both.
//
//   GATE_SS=0 : sd_card.ss = ~m_sel            (the X16 port as it was: BUG)
//   GATE_SS=1 : sd_card.ss = ~m_sel | ~vsd_sel (the BBC/ZX idiom: the FIX)
//
// Expected: GATE_SS=0 corrupts the post-mount read(s); GATE_SS=1 is clean.
// ============================================================================
module tb_wedge #(
    parameter integer GATE_SS      = 0,
    parameter integer BYTE_SPACING = 16,  // clk cycles between data bytes
    parameter integer ACK_LEAD     = 200, // clk cycles sd_rd -> ack asserted
    parameter integer ACK_DROP     = 8    // clk cycles last byte -> ack dropped
);
    reg clk = 0; always #5 clk = ~clk;
    reg res_n = 0;

    wire        r_w_n, sync;
    wire [15:0] addr;
    wire [7:0]  dout;
    wire [15:0] pc;
    reg  [7:0]  din;

    // ---- memory ----
    reg [7:0] ram [0:32767];    // $0000-$7FFF
    reg [7:0] rom [0:8191];     // $E000-$FFFF
    integer i;
    initial begin
        $readmemh("sdwedge.hex", rom);
        for (i=0;i<32768;i=i+1) ram[i]=0;
    end

    wire acc_9f3e = (addr == 16'h9F3E);
    wire acc_9f3f = (addr == 16'h9F3F);

    // ---- cpu_rdy (from x16.sv) ----
    wire vera_read = (acc_9f3e | acc_9f3f) & r_w_n;
    reg [1:0] vera_read_stall = 0;
    always @(posedge clk or negedge res_n)
        if (!res_n) vera_read_stall <= 0;
        else if (vera_read) begin if (vera_read_stall!=2'd3) vera_read_stall <= vera_read_stall+2'd1; end
        else vera_read_stall <= 0;

    // ---- the cpu_clk SPI master: the REAL RTL (rtl/spi_sd_master.sv) ----
    wire spi_stall;
    wire cpu_rdy_base = (~vera_read | (vera_read_stall >= 2'd2));
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

    // ---- din mux ----
    always @(*) begin
        if      (addr < 16'h8000) din = ram[addr[14:0]];
        else if (acc_9f3e)        din = m_data;
        else if (acc_9f3f)        din = m_status;
        else if (addr >= 16'hE000)din = rom[addr[12:0]];
        else                      din = 8'h00;
    end
    // RAM write
    always @(posedge clk) if (~r_w_n && addr < 16'h8000) ram[addr[14:0]] <= dout;

    // ---- CPU ----
    r65c02_wrap u_cpu(.clk(clk), .enable(cpu_rdy), .res_n(res_n), .irq_n(1'b1),
        .nmi_n(1'b1), .rdy(1'b1), .r_w_n(r_w_n), .sync(sync), .addr(addr),
        .din(din), .dout(dout), .pc(pc));

    // ---- mount state + the vsd_sel gating under test ----
    reg         mounted     = 0;      // Main has an image to serve
    reg         img_mounted = 0;      // hps_io mount pulse into sd_card
    reg  [63:0] img_size    = 64'd0;
    reg         vsd_sel     = 0;      // the x16.sv fix: latched on mount pulse
    always @(posedge clk) if (img_mounted) vsd_sel <= |img_size;
    wire ss_pin = (GATE_SS != 0) ? (~m_sel | ~vsd_sel) : ~m_sel;

    // ---- sd_card ----
    wire [31:0] sd_lba; wire sd_rd, sd_wr; reg sd_ack=0;
    reg [8:0] sd_buff_addr=0; reg [7:0] sd_buff_dout=0; wire [7:0] sd_buff_din; reg sd_buff_wr=0;
    wire [7:0] dbg_bufdout, dbg_mem510; wire [8:0] dbg_bufptr;
    wire [1:0] dbg_spibuf, dbg_sdbuf; wire [2:0] dbg_wrstate, dbg_rdstate;
    sd_card dut(.clk_sys(clk), .reset(~res_n), .sdhc(1'b1), .img_mounted(img_mounted),
        .img_size(img_size), .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .clk_spi(clk), .ss(ss_pin), .sck(m_sck), .mosi(m_mosi), .miso(miso),
        .dbg_bufdout(dbg_bufdout), .dbg_bufptr(dbg_bufptr), .dbg_spibuf(dbg_spibuf),
        .dbg_sdbuf(dbg_sdbuf), .dbg_wrstate(dbg_wrstate), .dbg_rdstate(dbg_rdstate), .dbg_mem510(dbg_mem510));

    // sector content the "image" holds, distinguishable per lba
    function [7:0] secbyte(input [8:0] n, input [31:0] lba);
        if (lba == 0)
            secbyte = (n==9'd450)?8'h0C:(n==9'd510)?8'h55:(n==9'd511)?8'hAA:n[7:0];
        else
            secbyte = n[7:0] ^ 8'hA5;
    endfunction

    // ========================================================================
    // ACCURATE hps_io read-fill model -- but Main only serves when `mounted`
    // ========================================================================
    localparam FS_IDLE=0, FS_LEAD=1, FS_FILL=2, FS_DRAIN=3;
    reg [1:0]  fstate = FS_IDLE;
    reg [2:0]  b_wr = 0;
    integer    bidx = 0;
    integer    spc  = 0;
    integer    leadc= 0;
    integer    dropc= 0;
    reg [31:0] serve_lba = 0;

    always @(posedge clk) begin
        sd_buff_wr <= b_wr[0];
        if (b_wr[2] && ~(&sd_buff_addr)) sd_buff_addr <= sd_buff_addr + 1'b1;
        b_wr <= (b_wr << 1);

        if (!res_n) begin
            fstate<=FS_IDLE; b_wr<=0; sd_ack<=0; sd_buff_wr<=0;
            sd_buff_addr<=0; bidx<=0; spc<=0; leadc<=0; dropc<=0;
        end else begin
            case (fstate)
                FS_IDLE:
                    if (mounted && (sd_rd || sd_wr)) begin leadc<=0; fstate<=FS_LEAD; end

                FS_LEAD:
                    if (leadc >= ACK_LEAD) begin
                        serve_lba    <= sd_lba;
                        sd_ack       <= 1'b1;
                        sd_buff_addr <= 0;
                        sd_buff_dout <= secbyte(9'd0, sd_lba);
                        b_wr         <= 3'b001;
                        bidx         <= 1;
                        spc          <= 0;
                        fstate       <= FS_FILL;
                        $display("[HPS ] t=%0t serving sd_rd for lba=%0d", $time, sd_lba);
                    end else leadc <= leadc + 1;

                FS_FILL:
                    if (bidx < 512) begin
                        if (spc >= BYTE_SPACING) begin
                            spc          <= 0;
                            sd_buff_dout <= secbyte(bidx[8:0], serve_lba);
                            b_wr         <= 3'b001;
                            bidx         <= bidx + 1;
                        end else spc <= spc + 1;
                    end else begin
                        dropc  <= 0;
                        fstate <= FS_DRAIN;
                    end

                FS_DRAIN:
                    if (dropc >= ACK_DROP) begin
                        sd_ack <= 1'b0;
                        fstate <= FS_IDLE;
                    end else dropc <= dropc + 1;
            endcase
        end
    end

    // ---- trace ----
    reg last_ack=0; reg [1:0] last_sdbuf=3, last_spibuf=3; reg [2:0] last_rds=7;
    always @(posedge clk) begin
        if (sd_ack !== last_ack) begin
            $display("[ACK ] t=%0t sd_ack -> %0b (lba=%0d)", $time, sd_ack, sd_lba);
            last_ack <= sd_ack;
        end
        if (dut.sd_buf  !== last_sdbuf)  begin $display("[SDBUF ] t=%0t sd_buf  -> %0d", $time, dut.sd_buf);  last_sdbuf  <= dut.sd_buf;  end
        if (dut.spi_buf !== last_spibuf) begin $display("[SPIBUF] t=%0t spi_buf -> %0d", $time, dut.spi_buf); last_spibuf <= dut.spi_buf; end
        if (dbg_rdstate !== last_rds)    begin $display("[RDST  ] t=%0t read_state -> %0d", $time, dbg_rdstate); last_rds <= dbg_rdstate; end
    end

    // ---- run control ----
    integer k, badA, badB;
    reg [7:0] exp;
    initial begin
        $display("[CFG ] GATE_SS=%0d BYTE_SPACING=%0d ACK_LEAD=%0d ACK_DROP=%0d", GATE_SS, BYTE_SPACING, ACK_LEAD, ACK_DROP);
        repeat(20) @(posedge clk); res_n = 1;
        fork
            begin : t
                // wait for phase 1 (pre-mount probe) to finish
                while (ram[1] != 8'h11) @(posedge clk);
                $display("[TBC ] t=%0t phase 1 done. read_state=%0d sd_rd=%0b sd_buf=%0d spi_buf=%0d sd_lba=%0d",
                         $time, dbg_rdstate, sd_rd, dut.sd_buf, dut.spi_buf, sd_lba);

                // mount the image (Main): pulse img_mounted, size nonzero,
                // model live (a pre-mount pending sd_rd now gets served)
                img_size <= 64'd104857600;
                mounted  <= 1'b1;
                @(posedge clk); img_mounted <= 1'b1;
                @(posedge clk); @(posedge clk); img_mounted <= 1'b0;
                $display("[TBC ] t=%0t image MOUNTED", $time);

                // give Main time to serve any stale pending sd_rd, then tell
                // the CPU (the user types @$ a moment later)
                #200000;   // 200 us
                $display("[TBC ] t=%0t post-mount settle: read_state=%0d sd_rd=%0b sd_buf=%0d spi_buf=%0d sd_lba=%0d",
                         $time, dbg_rdstate, sd_rd, dut.sd_buf, dut.spi_buf, sd_lba);
                ram[2] = 8'h01;

                // wait for the CPU to finish both reads
                while (ram[0] != 8'haa) @(posedge clk);
                $display("[TBC ] DONE. CMD0 R1=%02x (exp 01)  CMD8 R1=%02x (exp 01)", ram[3], ram[4]);
                $display("[TBC ] secA[0..7]   = %02x %02x %02x %02x %02x %02x %02x %02x",
                    ram['h400],ram['h401],ram['h402],ram['h403],ram['h404],ram['h405],ram['h406],ram['h407]);
                $display("[TBC ] secA 450/510/511 = %02x/%02x/%02x (exp 0C/55/AA)",
                    ram[16'h0400+450], ram[16'h0400+510], ram[16'h0400+511]);
                $display("[TBC ] secB[0..7]   = %02x %02x %02x %02x %02x %02x %02x %02x",
                    ram['h600],ram['h601],ram['h602],ram['h603],ram['h604],ram['h605],ram['h606],ram['h607]);

                badA = 0; badB = 0;
                for (k=0;k<512;k=k+1) begin
                    exp = secbyte(k[8:0], 32'd0);
                    if (ram[16'h0400+k] !== exp) begin
                        if (badA < 6) $display("[MISMATCH A] off=%0d got=%02x exp=%02x", k, ram[16'h0400+k], exp);
                        badA = badA + 1;
                    end
                    exp = secbyte(k[8:0], 32'd2048);
                    if (ram[16'h0600+k] !== exp) begin
                        if (badB < 6) $display("[MISMATCH B] off=%0d got=%02x exp=%02x", k, ram[16'h0600+k], exp);
                        badB = badB + 1;
                    end
                end
                if (badA==0 && badB==0)
                    $display("[TBC ] *** WEDGE TEST (GATE_SS=%0d): READ OK (both sectors match) ***", GATE_SS);
                else
                    $display("[TBC ] *** WEDGE TEST (GATE_SS=%0d): READ FAILED (lba0: %0d/512 bad, lba2048: %0d/512 bad) ***",
                             GATE_SS, badA, badB);
                disable w;
            end
            begin : w
                #80000000; $display("[TBC ] TIMEOUT (pc=%04x ram1=%02x ram2=%02x)", pc, ram[1], ram[2]); disable t;
            end
        join
        $finish;
    end
endmodule
