`timescale 1ns/1ps
// ============================================================================
// tb_datetest.v -- PRINT DATE$ crash hunt with the REAL RTL chain:
// r65c02 (real) -> via65c22 (real, $9F00-$9F0F) -> bit-banged I2C bus ->
// smc_x16 + rtc_x16 (real), running datetest.s = faithful i2c.s/rtc.s
// transcriptions, with a periodic IRQ whose handler does an SMC read
// (kbd_scan-style) so transactions interleave exactly like on HW.
//
// Outcomes: $0002 = AA pass / EE fail (code in $0003) / EB = BRK = the
// "crash to monitor".  TIMEOUT = hang (wedged bus / infinite re-read).
// ============================================================================
module tb_datetest;
    reg clk = 0; always #62.5 clk = ~clk;   // 8 MHz cpu_clk
    reg res_n = 0;

    wire        r_w_n, sync;
    wire [15:0] addr;
    wire  [7:0] dout;
    wire [15:0] pc;
    wire  [7:0] din;

    // ---- decode (subset of x16.sv: lowram, via1, rom) ----
    wire kernal_cs = (addr[15:14] == 2'b11);
    wire io_cs     = (addr[15:8]  == 8'h9F);
    wire via1_cs   = (addr[15:4]  == 12'h9F0);
    wire hi_ram_cs = (addr[15:13] == 3'b101);
    wire lowram_cs = ~kernal_cs & ~io_cs & ~hi_ram_cs;

    // ---- ROM (bank 0 image only) ----
    reg [7:0] rom [0:16383];
    initial $readmemh("datetest.hex", rom);
    reg [7:0] rom_data;
    always @(negedge clk) rom_data <= rom[addr[13:0]];

    // ---- LowRAM (real) ----
    wire [7:0] lowram_data;
    lowram_bram u_lowram (
        .clk(clk), .addr(addr), .cs(lowram_cs),
        .we(lowram_cs & ~r_w_n), .wr_data(dout), .rd_data(lowram_data)
    );

    // ---- VIA1 (real) + I2C bus exactly as in x16.sv ----
    wire [7:0] via1_data;
    wire       via1_irq_n;
    wire [7:0] via1_pa_in, via1_pa_out, via1_pa_oe;
    wire [7:0] via1_pb_in, via1_pb_out, via1_pb_oe;

    wire smc_sda_drv_low, rtc_sda_drv_low;
    wire via_sda_drv_low = via1_pa_oe[0] & ~via1_pa_out[0];
    wire via_scl_drv_low = via1_pa_oe[1] & ~via1_pa_out[1];
    wire bus_sda = ~(via_sda_drv_low | smc_sda_drv_low | rtc_sda_drv_low);
    wire bus_scl = ~via_scl_drv_low;

    assign via1_pa_in[0]   = bus_sda;
    assign via1_pa_in[1]   = bus_scl;
    assign via1_pa_in[7:2] = via1_pa_out[7:2] | ~via1_pa_oe[7:2];
    assign via1_pb_in      = via1_pb_out | ~via1_pb_oe;

    wire cpu_rdy = 1'b1;   // no SDRAM/VERA in this TB

    via65c22 u_via1 (
        .clk(clk), .reset_n(res_n), .cs(via1_cs), .rwn(r_w_n),
        .enable(cpu_rdy), .addr(addr[3:0]), .di(dout), .do_o(via1_data),
        .pa_in(via1_pa_in), .pa_out(via1_pa_out), .pa_oe(via1_pa_oe),
        .pb_in(via1_pb_in), .pb_out(via1_pb_out), .pb_oe(via1_pb_oe),
        .ca1_in(1'b0), .ca2_in(1'b0), .cb1_in(1'b0), .cb2_in(1'b0),
        .irq_n(via1_irq_n)
    );

    // ---- the two I2C slaves (real) ----
    smc_x16 u_smc (
        .clk(clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(smc_sda_drv_low),
        .uart_byte(8'h00), .uart_byte_valid(1'b0)
    );

    reg [64:0] hps_rtc = 65'd0;
    rtc_x16 u_rtc (             // real CLK_HZ: seconds tick at 1 Hz
        .clk(clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(rtc_sda_drv_low),
        .hps_rtc(hps_rtc)
    );

    // ---- periodic IRQ (stands in for VERA VSYNC), ~2ms apart ----
    reg irq_req = 0;
    integer icnt = 0;
    always @(posedge clk) begin
        if (!res_n) begin irq_req <= 0; icnt <= 0; end
        else begin
            icnt <= icnt + 1;
            if (icnt % 16000 == 0) irq_req <= 1'b1;
            // handler entry detected -> drop the line (edge-ish source)
            if (irq_req && addr == 16'hFFFE) irq_req <= 1'b0;
        end
    end
    wire cpu_irq_n = ~irq_req & via1_irq_n;

    // ---- din mux ----
    assign din = kernal_cs ? rom_data :
                 via1_cs   ? via1_data :
                 lowram_cs ? lowram_data : 8'h00;

    // ---- CPU (real r65c02) ----
    r65c02_wrap u_cpu (
        .clk(clk), .enable(cpu_rdy), .res_n(res_n),
        .irq_n(cpu_irq_n), .nmi_n(1'b1), .rdy(1'b1),
        .r_w_n(r_w_n), .sync(sync), .addr(addr),
        .din(din), .dout(dout), .pc(pc)
    );

    integer i;
    initial begin
        for (i = 0; i < 40704; i = i + 1) u_lowram.mem[i] = 8'h00;
    end

    initial begin
        repeat (20) @(posedge clk);
        res_n = 1;
        repeat (20) @(posedge clk);
        // wall clock: 2026-07-05 21:58:40, wday=3 -> reg 4 (Wed)
        hps_rtc[63:0] <= {8'h40, 5'd0, 3'd3, 8'h26, 8'h07, 8'h05, 8'h21, 8'h58, 8'h40};
        hps_rtc[64]   <= ~hps_rtc[64];

        fork
            begin : t
                while (u_lowram.mem[2] !== 8'hAA && u_lowram.mem[2] !== 8'hEE
                       && u_lowram.mem[2] !== 8'hEB)
                    @(posedge clk);
                case (u_lowram.mem[2])
                    8'hAA: $display("[DATE] *** DATE$ PATH: ALL PASS (8 reads under IRQ traffic) ***");
                    8'hEB: $display("[DATE] *** BRK -> MONITOR reproduced! ***");
                    default: $display("[DATE] *** FAIL code=%0d ***", u_lowram.mem[3]);
                endcase
                $display("[DATE] regs sec=%02x min=%02x hour=%02x wk=%02x date=%02x mon=%02x yr=%02x retry=%0d smc=%02x",
                         u_lowram.mem[16], u_lowram.mem[17], u_lowram.mem[18],
                         u_lowram.mem[19], u_lowram.mem[20], u_lowram.mem[21],
                         u_lowram.mem[22], u_lowram.mem[23], u_lowram.mem[32]);
                disable w;
            end
            begin : w
                #120000000;
                $display("[DATE] TIMEOUT pc=%04x addr=%04x done=%02x code=%0d iter=%02x -- HANG (bus wedged / infinite re-read)",
                         pc, addr, u_lowram.mem[2], u_lowram.mem[3], u_lowram.mem[26]);
                $display("[DATE] bus: sda=%b scl=%b smcdrv=%b rtcdrv=%b",
                         bus_sda, bus_scl, smc_sda_drv_low, rtc_sda_drv_low);
                disable t;
            end
        join
        $finish;
    end
endmodule
