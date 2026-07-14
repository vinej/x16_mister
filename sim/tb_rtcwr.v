`timescale 1ns/1ps
// ============================================================================
// tb_rtcwr.v -- RTC I2C WRITE path with master-side ACK VERIFICATION (jyv
// 2026-07-07).  HW symptom: NVRAM reg $5F reads $FF forever, i.e. the
// KERNAL's boot-time screen_set_default_nvram never lands its writes --
// while reads (DATE$) work.  The ROM's i2c_write_byte aborts on a missing
// ACK after ANY byte; earlier TBs clocked through the ACK slot blindly.
//
// This TB replays the EXACT r49 rtc_set_nvram flow (i2c.s shapes, both
// slaves on the bus like the real system) and CHECKS every ACK the way
// rec_bit does (release SDA, raise SCL, sample):
//   1. plain write reg $40 = $AA: addr/reg/data ACKs all 0, mem updated
//   2. rtc_set_nvram replay: write + 32-byte checksum read-back + checksum
//      write; verify $5F holds the sum
//   3. multi-byte write (addr, reg, d0, d1): both data ACKs, both bytes land
// ============================================================================
module tb_rtcwr;
    integer errors = 0;

    reg clk = 0; always #62.5 clk = ~clk;   // 8 MHz cpu_clk
    reg res_n = 0;

    // ---- I2C bus with BOTH slaves (tb_i2cboot wiring) ----
    reg  m_sda_drv = 0, m_scl_drv = 0;
    wire rtc_sda_low, smc_sda_low;
    wire bus_sda = ~(m_sda_drv | rtc_sda_low | smc_sda_low);
    wire bus_scl = ~m_scl_drv;

    smc_x16 u_smc (
        .clk(clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(smc_sda_low),
        .uart_byte(8'h00), .uart_byte_valid(1'b0),
        .power_off_req(), .reset_req(), .nmi_req(), .act_led_r(),
        .dbg_kbd_count(), .dbg_saw_start(), .dbg_saw_addr_match(),
        .dbg_saw_byte(), .dbg_saw_repeat(), .dbg_saw_stop(), .dbg_saw_tx(),
        .dbg_last_cmd(), .dbg_last_addr_byte(), .dbg_kbd_pop(), .dbg_tx_byte()
    );

    rtc_x16 #(.CLK_HZ(8000)) u_rtc (
        .clk(clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(rtc_sda_low),
        .hps_rtc(65'd0),
        .nv_we(1'b0), .nv_addr(7'd0), .nv_wdata(8'd0),
        .nv_snoop_we(), .nv_snoop_addr(), .nv_snoop_data(), .nv_dirty_toggle()
    );

    // ---- i2c.s primitives (VIA-instruction timing) ----
    task lop; begin repeat (7) @(posedge clk); end endtask
    task brief; begin repeat (40) @(posedge clk); end endtask
    task sda_low;  begin m_sda_drv = 1; lop; end endtask
    task sda_high; begin m_sda_drv = 0; lop; end endtask
    task scl_low;  begin m_scl_drv = 1; lop; end endtask
    task scl_high; begin m_scl_drv = 0; lop; while (!bus_scl) @(posedge clk); end endtask
    task i2c_init;  begin sda_high; scl_high; end endtask
    task i2c_start; begin sda_low; brief; scl_low; end endtask
    task i2c_stop;  begin sda_low; brief; scl_high; brief; sda_high; brief; end endtask
    task send_bit(input b); begin
        if (b) sda_high; else sda_low;
        scl_high; lop; scl_low;
    end endtask
    // rec_bit: release SDA, raise SCL, SAMPLE (this is what i2c.s does)
    reg rbit;
    task rec_bit; begin
        sda_high; scl_high;
        rbit = bus_sda; lop;
        scl_low;
    end endtask
    // i2c_write per i2c.s: 8 bits then rec_bit = ACK (0 = ok)
    reg ack;
    task i2c_write(input [7:0] b); integer i; begin
        for (i = 7; i >= 0; i = i - 1) send_bit(b[i]);
        rec_bit; ack = rbit;
    end endtask
    reg [7:0] rbyte;
    task i2c_read(input do_ack); integer i; begin
        rbyte = 8'h00;
        for (i = 7; i >= 0; i = i - 1) begin rec_bit; rbyte = {rbyte[6:0], rbit}; end
        if (do_ack) begin sda_low; scl_high; lop; scl_low; sda_high; end
        else        begin sda_high; scl_high; lop; scl_low; end
    end endtask

    task chk_ack(input [255:0] what); begin
        if (ack !== 1'b0) begin
            $display("[RTCW] FAIL NAK on %0s", what);
            errors = errors + 1;
        end
    end endtask
    task chk8(input [7:0] got, input [7:0] expct, input [255:0] what); begin
        if (got !== expct) begin
            $display("[RTCW] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    // i2c_write_byte(dev,reg,val) with full ACK checking
    task wr_byte(input [7:0] rg, input [7:0] val); begin
        i2c_init; i2c_start;
        i2c_write({7'h6F, 1'b0}); chk_ack("addr(W)");
        i2c_write(rg);            chk_ack("regptr");
        i2c_write(val);           chk_ack("wrdata");
        i2c_stop;
    end endtask

    // i2c_read_byte(dev,reg) -> rbyte (KERNAL shape: W addr+reg, STOP,
    // START, R addr, read, NACK, STOP)
    task rd_byte(input [7:0] rg); begin
        i2c_init; i2c_start;
        i2c_write({7'h6F, 1'b0}); chk_ack("rd addr(W)");
        i2c_write(rg);            chk_ack("rd regptr");
        i2c_stop; i2c_start;
        i2c_write({7'h6F, 1'b1}); chk_ack("rd addr(R)");
        i2c_read(1'b0);
        i2c_stop;
    end endtask

    integer k;
    reg [7:0] sum;
    initial begin
        repeat (20) @(posedge clk);
        res_n = 1;
        repeat (20) @(posedge clk);

        // ---- 1. plain NVRAM write + read-back ----
        wr_byte(8'h40, 8'hAA);
        chk8(u_rtc.mem[8'h40 - 8'h07], 8'hAA, "mem[$40] committed");
        rd_byte(8'h40);
        chk8(rbyte, 8'hAA, "read-back $40");

        // ---- 2. full rtc_set_nvram replay: write $41=$21, then the
        //         checksum pass (read $40-$5E, sum, write $5F) ----
        wr_byte(8'h41, 8'h21);
        sum = 8'h00;
        for (k = 8'h40; k < 8'h5F; k = k + 1) begin
            rd_byte(k[7:0]);
            sum = sum + rbyte;
        end
        wr_byte(8'h5F, sum);
        rd_byte(8'h5F);
        chk8(rbyte, sum, "checksum committed at $5F");
        chk8(u_rtc.mem[88], sum, "mem[88] = checksum");
        if (sum === 8'hFF) $display("[RTCW] note: sum happens to be FF");

        // ---- 3. multi-byte write (auto-increment): $50=$11, $51=$22 ----
        i2c_init; i2c_start;
        i2c_write({7'h6F, 1'b0}); chk_ack("mb addr");
        i2c_write(8'h50);         chk_ack("mb regptr");
        i2c_write(8'h11);         chk_ack("mb data0");
        i2c_write(8'h22);         chk_ack("mb data1");
        i2c_stop;
        chk8(u_rtc.mem[8'h50 - 8'h07], 8'h11, "mb byte0");
        chk8(u_rtc.mem[8'h51 - 8'h07], 8'h22, "mb byte1");

        if (errors == 0) $display("[RTCW] ALL TESTS PASS");
        else             $display("[RTCW] %0d ERRORS", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("[RTCW] TIMEOUT");
        $finish;
    end
endmodule
