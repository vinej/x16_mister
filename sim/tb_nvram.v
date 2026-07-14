`timescale 1ns/1ps
// ============================================================================
// tb_nvram.v -- RTC NVRAM persistence (jyv 2026-07-07, reworked with the
// single-clock rtc mem + dual-clock backer buffers after Quartus 276001):
// real rtc_x16 + real nvram_backer against a mock hps_io virtual-disk slot,
// with CPU-side traffic driven over REAL I2C (KERNAL i2c.s timing) so the
// snoop -> shadow -> autosave path is exercised end to end.
//
//   1. mount image -> block read -> walker restores rtc mem[0..88]
//   2. I2C writes to NVRAM regs -> autosave after SAVE_DELAY; saved block
//      = written values + restored values + $00 padding for bytes 89-511
//   3. burst of I2C writes -> countdown restarts, exactly one save
//   4. read-only image -> dirty -> NO save
// ============================================================================
module tb_nvram;
    integer errors = 0;

    // 100 MHz hps domain + 8 MHz cpu domain
    reg clk100 = 0; always #5    clk100 = ~clk100;
    reg clk    = 0; always #62.5 clk    = ~clk;     // cpu_clk (i2c tasks use `clk`)
    reg res_n  = 0;

    // ---- I2C bus (tb_i2cboot wiring) ----
    reg  m_sda_drv = 0, m_scl_drv = 0;              // master drives low when 1
    wire rtc_sda_low;
    wire bus_sda = ~(m_sda_drv | rtc_sda_low);
    wire bus_scl = ~m_scl_drv;

    // ---- rtc <-> backer nv port (cpu_clk domain) ----
    wire       nv_we, nv_snoop_we, nv_dirty_toggle;
    wire [6:0] nv_addr, nv_snoop_addr;
    wire [7:0] nv_wdata, nv_snoop_data;

    rtc_x16 #(.CLK_HZ(8000)) u_rtc (
        .clk(clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(rtc_sda_low),
        .hps_rtc(65'd0),
        .nv_we(nv_we), .nv_addr(nv_addr), .nv_wdata(nv_wdata),
        .nv_snoop_we(nv_snoop_we), .nv_snoop_addr(nv_snoop_addr),
        .nv_snoop_data(nv_snoop_data), .nv_dirty_toggle(nv_dirty_toggle)
    );

    // ---- mock hps_io slot ----
    reg         img_mounted = 0;
    reg         img_readonly = 0;
    reg  [63:0] img_size = 0;
    wire [31:0] sd_lba;
    wire        sd_rd, sd_wr;
    reg         sd_ack = 0;
    reg  [8:0]  sd_buff_addr = 0;
    reg  [7:0]  sd_buff_dout = 0;
    wire [7:0]  sd_buff_din;
    reg         sd_buff_wr = 0;

    nvram_backer #(.SAVE_DELAY(40000)) u_backer (   // 400 us in sim
        .clk(clk100), .cpu_clk(clk), .reset_n(res_n),
        .img_mounted(img_mounted), .img_readonly(img_readonly),
        .img_size(img_size),
        .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
        .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .nv_we(nv_we), .nv_addr(nv_addr), .nv_wdata(nv_wdata),
        .nv_snoop_we(nv_snoop_we), .nv_snoop_addr(nv_snoop_addr),
        .nv_snoop_data(nv_snoop_data), .nv_dirty_toggle(nv_dirty_toggle)
    );

    // ---- i2c.s primitives (from tb_i2cboot.v, VIA-instruction timing) ----
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
    task i2c_write(input [7:0] b); integer i; begin
        for (i = 7; i >= 0; i = i - 1) send_bit(b[i]);
        sda_high; scl_high; lop; scl_low;              // ack slot
    end endtask
    // NVRAM reg write: mem[idx] lives at RTC register $07+idx
    task nv_i2c_write(input [6:0] idx, input [7:0] val); begin
        i2c_init; i2c_start;
        i2c_write({7'h6F, 1'b0});
        i2c_write({1'b0, idx} + 8'h07);
        i2c_write(val);
        i2c_stop;
    end endtask

    task chk8(input [7:0] got, input [7:0] expct, input [255:0] what); begin
        if (got !== expct) begin
            if (errors < 12)
                $display("[NVR ] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    // mock: serve a block READ (HPS -> core), pattern byte = addr ^ A5
    task serve_read; integer a; begin
        wait (sd_rd);
        repeat (20) @(posedge clk100);
        sd_ack = 1;
        @(posedge clk100);
        for (a = 0; a < 512; a = a + 1) begin
            sd_buff_addr = a[8:0];
            sd_buff_dout = a[7:0] ^ 8'hA5;
            sd_buff_wr   = 1;
            @(posedge clk100);
            sd_buff_wr   = 0;
            repeat (3) @(posedge clk100);
        end
        sd_ack = 0;
        repeat (10) @(posedge clk100);
    end endtask

    // mock: serve a block WRITE (core -> HPS) into wr_img[]
    reg [7:0] wr_img [0:511];
    task serve_write; integer a; begin
        wait (sd_wr);
        repeat (20) @(posedge clk100);
        sd_ack = 1;
        @(posedge clk100);
        for (a = 0; a < 512; a = a + 1) begin
            sd_buff_addr = a[8:0];
            repeat (3) @(posedge clk100);   // registered-q contract
            wr_img[a] = sd_buff_din;
            @(posedge clk100);
        end
        sd_ack = 0;
        repeat (10) @(posedge clk100);
    end endtask

    integer i;
    reg saw_wr;
    initial begin
        repeat (10) @(posedge clk100);
        res_n = 1;
        repeat (10) @(posedge clk100);

        // ---- 1. mount DURING RESET (the HW-found auto-remount window:
        //         Main delivers the pulse while the ROM download still
        //         holds the system reset) -> restore must still happen ----
        res_n = 0;
        repeat (10) @(posedge clk100);
        img_size = 64'd512; img_readonly = 0;
        @(posedge clk100) img_mounted = 1;
        @(posedge clk100) img_mounted = 0;
        repeat (200) @(posedge clk100);    // reset held well past the pulse
        res_n = 1;
        serve_read;
        repeat (2000) @(posedge clk);              // walker: 89 bytes
        chk8(u_rtc.mem[0],  8'h00 ^ 8'hA5, "restore mem[0]");
        chk8(u_rtc.mem[42], 8'd42 ^ 8'hA5, "restore mem[42]");
        chk8(u_rtc.mem[88], 8'd88 ^ 8'hA5, "restore mem[88]");

        // restore must NOT trigger an autosave
        saw_wr = 0;
        repeat (90000) @(posedge clk100);          // > SAVE_DELAY
        if (sd_wr) saw_wr = 1;
        chk8({7'd0, saw_wr}, 8'h00, "no save after restore");

        // ---- 2. I2C writes -> snoop -> shadow -> autosave ----
        nv_i2c_write(7'd10, 8'hBE);
        nv_i2c_write(7'd88, 8'h5A);
        serve_write;
        chk8(wr_img[10],  8'hBE,          "saved mem[10]");
        chk8(wr_img[88],  8'h5A,          "saved mem[88]");
        chk8(wr_img[0],   8'h00 ^ 8'hA5,  "saved mem[0] unchanged");
        chk8(wr_img[42],  8'd42 ^ 8'hA5,  "saved mem[42] unchanged");
        chk8(wr_img[89],  8'h00,          "pad byte 89 zero");
        chk8(wr_img[511], 8'h00,          "pad byte 511 zero");

        // ---- 3. burst -> exactly one save ----
        nv_i2c_write(7'd20, 8'h01);
        nv_i2c_write(7'd20, 8'h02);
        nv_i2c_write(7'd20, 8'h09);
        serve_write;
        chk8(wr_img[20], 8'h09, "burst last value saved");
        chk8(wr_img[10], 8'hBE, "burst kept mem[10]");
        saw_wr = 0;
        repeat (90000) @(posedge clk100);
        if (sd_wr) saw_wr = 1;
        chk8({7'd0, saw_wr}, 8'h00, "single save per burst");

        // ---- 4. read-only image: no save ----
        img_readonly = 1;
        @(posedge clk100) img_mounted = 1;
        @(posedge clk100) img_mounted = 0;
        serve_read;
        repeat (2000) @(posedge clk);
        nv_i2c_write(7'd30, 8'h77);
        saw_wr = 0;
        repeat (120000) @(posedge clk100);
        if (sd_wr) saw_wr = 1;
        chk8({7'd0, saw_wr}, 8'h00, "read-only: no save");

        if (errors == 0) $display("[NVR ] ALL TESTS PASS");
        else             $display("[NVR ] %0d ERRORS", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("[NVR ] TIMEOUT");
        $finish;
    end

endmodule
