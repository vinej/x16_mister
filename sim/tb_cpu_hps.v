`timescale 1ns/1ps
// ============================================================================
// ACCURATE-hps_io testbench for the X16 SD read path.
//
// Same real r65c02 CPU + cpu_clk SPI master + real sd_card as tb_cpu.v, but the
// mock dense-fill hps_io is replaced with a FAITHFUL replica of sys/hps_io.sv's
// sector-fill behaviour for a READ (HPS -> FPGA via command 0x17):
//
//   * global b_wr[2:0] pipeline (verbatim from hps_io.sv:295-297):
//         sd_buff_wr <= b_wr[0];
//         if(b_wr[2] && ~&sd_buff_addr) sd_buff_addr <= sd_buff_addr + 1;
//         b_wr <= b_wr << 1;
//   * per data byte:  sd_buff_dout <= data;  b_wr <= 1;   (hps_io.sv:404-407)
//   * sd_buff_addr reset to 0 at the command byte (hps_io.sv:345)
//   * sd_ack asserted at the command byte and HELD for the whole block,
//     dropped only ACK_DROP cycles after the LAST data byte (models ~io_enable).
//   * data bytes are spaced BYTE_SPACING clk_sys cycles apart (HPS SPI cadence).
//
// Override with:  vsim -gBYTE_SPACING=16 -gACK_DROP=8 tb_cpu_hps
// The theory under test: if sd_ack de-asserts before the b_wr pipeline flushes
// the final 1-2 writes, bytes 510/511 never land in the buffer -> read = 0x00.
// ============================================================================
module tb_cpu_hps #(
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
        $readmemh("sdtest.hex", rom);
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

    // ---- MY master (verbatim) ----
    localparam [2:0] M_HALF = 3'd3;
    reg [7:0] m_tx, m_rx; reg [3:0] m_ec; reg [2:0] m_div;
    reg m_busy=0, m_autotx=0, m_slow=0, m_sck=0, m_mosi=1, m_sel=0;
    wire spi_stall = acc_9f3e & m_busy;
    wire cpu_rdy_base = (~vera_read | (vera_read_stall >= 2'd2));
    wire cpu_rdy = cpu_rdy_base & ~spi_stall;
    wire wr_9f3e = cpu_rdy & ~r_w_n & acc_9f3e;
    wire rd_9f3e = cpu_rdy &  r_w_n & acc_9f3e;
    wire wr_9f3f = cpu_rdy & ~r_w_n & acc_9f3f;
    wire start_xfer = wr_9f3e | (rd_9f3e & m_autotx);
    wire miso;

    always @(posedge clk or negedge res_n) begin
        if (!res_n) begin m_busy<=0; m_sck<=0; m_mosi<=1; m_sel<=0; m_ec<=0; m_div<=0; m_rx<=0; m_tx<=0; m_autotx<=0; m_slow<=0; end
        else begin
            if (wr_9f3f) begin m_autotx<=dout[2]; m_slow<=dout[1]; m_sel<=dout[0]; end
            if (start_xfer) begin
                m_tx <= wr_9f3e ? dout : 8'hFF; m_mosi <= wr_9f3e ? dout[7] : 1'b1;
                m_busy<=1; m_ec<=0; m_div<=0; m_sck<=0;
            end else if (m_busy) begin
                if (m_div==M_HALF) begin
                    m_div<=0; m_sck<=~m_sck;
                    if (~m_sck) m_rx <= {m_rx[6:0], miso};
                    else begin m_tx<={m_tx[6:0],1'b0}; m_mosi<=m_tx[6]; end
                    m_ec<=m_ec+1; if (m_ec==15) m_busy<=0;
                end else m_div<=m_div+1;
            end
        end
    end
    wire [7:0] m_data = m_rx;
    wire [7:0] m_status = {m_busy,4'b0,m_autotx,m_slow,m_sel};

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

    // ---- sd_card ----
    wire [31:0] sd_lba; wire sd_rd, sd_wr; reg sd_ack=0;
    reg [8:0] sd_buff_addr=0; reg [7:0] sd_buff_dout=0; wire [7:0] sd_buff_din; reg sd_buff_wr=0;
    wire [7:0] dbg_bufdout, dbg_mem510; wire [8:0] dbg_bufptr;
    wire [1:0] dbg_spibuf, dbg_sdbuf; wire [2:0] dbg_wrstate, dbg_rdstate;
    sd_card dut(.clk_sys(clk), .reset(~res_n), .sdhc(1'b1), .img_mounted(1'b0),
        .img_size(64'd1073741824), .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .clk_spi(clk), .ss(~m_sel), .sck(m_sck), .mosi(m_mosi), .miso(miso),
        .dbg_bufdout(dbg_bufdout), .dbg_bufptr(dbg_bufptr), .dbg_spibuf(dbg_spibuf),
        .dbg_sdbuf(dbg_sdbuf), .dbg_wrstate(dbg_wrstate), .dbg_rdstate(dbg_rdstate), .dbg_mem510(dbg_mem510));

    function [7:0] secbyte(input [8:0] n);
        secbyte = (n==9'd450)?8'h0C:(n==9'd510)?8'h55:(n==9'd511)?8'hAA:n[7:0]; endfunction

    // ========================================================================
    // ACCURATE hps_io read-fill model
    // ========================================================================
    localparam FS_IDLE=0, FS_LEAD=1, FS_FILL=2, FS_DRAIN=3;
    reg [1:0]  fstate = FS_IDLE;
    reg [2:0]  b_wr = 0;
    integer    bidx = 0;   // next data-byte index to issue (0..512)
    integer    spc  = 0;   // spacing counter
    integer    leadc= 0;   // lead-latency counter
    integer    dropc= 0;   // ack-drop counter

    always @(posedge clk) begin
        // --- global b_wr pipeline (verbatim from hps_io.sv:295-297) ---
        sd_buff_wr <= b_wr[0];
        if (b_wr[2] && ~(&sd_buff_addr)) sd_buff_addr <= sd_buff_addr + 1'b1;
        b_wr <= (b_wr << 1);

        if (!res_n) begin
            fstate<=FS_IDLE; b_wr<=0; sd_ack<=0; sd_buff_wr<=0;
            sd_buff_addr<=0; bidx<=0; spc<=0; leadc<=0; dropc<=0;
        end else begin
            case (fstate)
                FS_IDLE:
                    if (sd_rd || sd_wr) begin leadc<=0; fstate<=FS_LEAD; end

                FS_LEAD:
                    if (leadc >= ACK_LEAD) begin
                        // command byte: assert ack (held), reset buffer addr,
                        // then immediately issue first data byte (addr 0).
                        sd_ack       <= 1'b1;
                        sd_buff_addr <= 0;
                        sd_buff_dout <= secbyte(9'd0);
                        b_wr         <= 3'b001;
                        bidx         <= 1;
                        spc          <= 0;
                        fstate       <= FS_FILL;
                    end else leadc <= leadc + 1;

                FS_FILL:
                    if (bidx < 512) begin
                        if (spc >= BYTE_SPACING) begin
                            spc          <= 0;
                            sd_buff_dout <= secbyte(bidx[8:0]);
                            b_wr         <= 3'b001;
                            bidx         <= bidx + 1;
                        end else spc <= spc + 1;
                    end else begin
                        dropc  <= 0;
                        fstate <= FS_DRAIN;
                    end

                FS_DRAIN:
                    if (dropc >= ACK_DROP) begin
                        sd_ack <= 1'b0;      // models HPS de-asserting io_enable
                        fstate <= FS_IDLE;
                    end else dropc <= dropc + 1;
            endcase
        end
    end

    // ---- concise trace: ack edges, bank moves, last few fill writes ----
    reg last_ack=0; reg [1:0] last_sdbuf=3, last_spibuf=3;
    always @(posedge clk) begin
        if (sd_ack !== last_ack) begin
            $display("[ACK ] t=%0t sd_ack -> %0b   (bidx=%0d sd_buff_addr=%0d)", $time, sd_ack, bidx, sd_buff_addr);
            last_ack <= sd_ack;
        end
        if (dut.sd_buf  !== last_sdbuf)  begin $display("[SDBUF ] t=%0t sd_buf  -> %0d", $time, dut.sd_buf);  last_sdbuf  <= dut.sd_buf;  end
        if (dut.spi_buf !== last_spibuf) begin $display("[SPIBUF] t=%0t spi_buf -> %0d", $time, dut.spi_buf); last_spibuf <= dut.spi_buf; end
        // trace the actual buffer writes for the sector tail
        if (sd_ack && sd_buff_wr && sd_buff_addr >= 9'd505)
            $display("[FILL ] t=%0t write addr=%0d data=%02x bank(sd_buf)=%0d", $time, sd_buff_addr, sd_buff_dout, dut.sd_buf);
    end

    initial begin
        $display("[CFG ] BYTE_SPACING=%0d ACK_LEAD=%0d ACK_DROP=%0d", BYTE_SPACING, ACK_LEAD, ACK_DROP);
        repeat(20) @(posedge clk); res_n = 1;
        fork
            begin : t
                while (ram[0] != 8'haa) @(posedge clk);
                $display("[TBC] DONE. byte450=%02x(0C) byte510=%02x(55) byte511=%02x(AA)",
                         ram[16'h05C2], ram[16'h05FE], ram[16'h05FF]);
                $display("[TBC] tail RAM 05F0: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                    ram['h5F0],ram['h5F1],ram['h5F2],ram['h5F3],ram['h5F4],ram['h5F5],ram['h5F6],ram['h5F7],
                    ram['h5F8],ram['h5F9],ram['h5FA],ram['h5FB],ram['h5FC],ram['h5FD],ram['h5FE],ram['h5FF]);
                $display("[MEM] sd_card bank0[505..511]=%02x %02x %02x %02x %02x %02x %02x",
                    dut.sdbuf_mem[505],dut.sdbuf_mem[506],dut.sdbuf_mem[507],dut.sdbuf_mem[508],
                    dut.sdbuf_mem[509],dut.sdbuf_mem[510],dut.sdbuf_mem[511]);
                begin : verify
                    integer k; integer bad; reg [7:0] exp;
                    bad = 0;
                    for (k=0;k<512;k=k+1) begin
                        exp = secbyte(k[8:0]);
                        if (ram[16'h0400+k] !== exp) begin
                            if (bad < 16) $display("[MISMATCH] off=%0d got=%02x exp=%02x", k, ram[16'h0400+k], exp);
                            bad = bad + 1;
                        end
                    end
                    if (bad==0) $display("[TBC] *** ACCURATE-hps_io: READ OK (all 512 bytes match) ***");
                    else        $display("[TBC] *** ACCURATE-hps_io: READ FAILED (%0d/512 mismatches) ***", bad);
                end
                disable w;
            end
            begin : w
                #20000000; $display("[TBC] TIMEOUT (pc=%04x)", pc); disable t;
            end
        join
        $finish;
    end
endmodule
