`timescale 1ns/1ps
// ============================================================================
// tb_i2cboot.v -- BOTH I2C slaves (smc_x16 $42 + rtc_x16 $6F) on one bus,
// driven by a cycle-faithful replica of the R49 KERNAL i2c.s bit-bang
// (VIA DDR tsb/trb granularity: ~7 cpu cycles per line operation, scl_high
// with the wait_for_clk poll, i2c_brief_delay ~40 cycles, and the REAL
// transaction shapes: i2c_read_first_byte does write-ptr, STOP, START,
// addr+R -- not a repeated START).
//
// Replays the boot sequence that freezes on HW at the splash:
//   1. NVRAM checksum sweep: 33 single-byte reads of $6F/$40..$5F
//   2. ...interleaved every few bytes with VSYNC-IRQ-style SMC traffic
//      ($42 reads), like irq_emulated_impl firing between mainline bytes
//   3. rtc_get_date_time replica incl. the re-read-if-seconds-changed loop
//   4. NVRAM write + readback
// After EVERY transaction: the bus must be idle (nobody driving SDA/SCL).
// Any stuck driver, wrong data, or non-terminating loop is the HW freeze.
// ============================================================================
module tb_i2cboot;
    reg clk = 0; always #62.5 clk = ~clk;    // 8 MHz cpu_clk
    reg res_n = 0;

    integer errors = 0;

    // ---- open-drain bus ----
    reg  m_sda_drv = 0, m_scl_drv = 0;       // master drives-low (VIA DDR out)
    wire smc_sda_low, rtc_sda_low;
    wire bus_sda = ~(m_sda_drv | smc_sda_low | rtc_sda_low);
    wire bus_scl = ~m_scl_drv;

    // ---- slaves ----
    // SMC fed through the REAL ps2_to_smc_bridge (as in x16.sv), so mouse
    // packets arrive as the bridge's 4-consecutive-cycle burst -- plus a
    // direct injection mux for the byte-level tests.
    reg  [10:0] ps2_key   = 11'h0;
    reg  [24:0] ps2_mouse = 25'h0;
    reg  [7:0]  ps2_mwheel = 8'h0;
    wire [7:0]  br_byte;
    wire        br_valid;
    ps2_to_smc_bridge u_bridge (
        .clk(clk), .reset_n(res_n),
        .ps2_key(ps2_key), .ps2_mouse(ps2_mouse),
        .ps2_mouse_wheel(ps2_mwheel),
        .uart_byte(br_byte), .uart_byte_valid(br_valid)
    );

    reg [7:0] u_byte  = 8'h00;               // injected host events (kbd/mouse)
    reg       u_valid = 1'b0;
    smc_x16 u_smc (
        .clk             (clk),
        .reset_n         (res_n),
        .sda_bus         (bus_sda),
        .scl_bus         (bus_scl),
        .sda_drive_low   (smc_sda_low),
        .uart_byte       (u_valid ? u_byte : br_byte),
        .uart_byte_valid (u_valid | br_valid)
    );

    reg [64:0] hps_rtc = 65'd0;
    rtc_x16 #(.CLK_HZ(2_000_000)) u_rtc (   // fast seconds: forces the
        .clk           (clk),               // "seconds changed, re-read"
        .reset_n       (res_n),             // path in the date-time loop
        .sda_bus       (bus_sda),
        .scl_bus       (bus_scl),
        .sda_drive_low (rtc_sda_low),
        .hps_rtc       (hps_rtc)
    );

    // ---- i2c.s primitives, VIA-instruction timing ----
    task lop; begin repeat (7) @(posedge clk); end endtask       // one tsb/trb
    task brief; begin repeat (40) @(posedge clk); end endtask    // ~5 us

    task sda_low;  begin m_sda_drv = 1; lop; end endtask
    task sda_high; begin m_sda_drv = 0; lop; end endtask
    task scl_low;  begin m_scl_drv = 1; lop; end endtask
    task scl_high; begin                                          // + wait_for_clk
        m_scl_drv = 0; lop;
        while (!bus_scl) @(posedge clk);
    end endtask

    task i2c_init;  begin sda_high; scl_high; end endtask
    task i2c_start; begin sda_low; brief; scl_low; end endtask
    task i2c_stop;  begin sda_low; brief; scl_high; brief; sda_high; brief; end endtask

    task send_bit(input b); begin
        if (b) sda_high; else sda_low;
        scl_high; lop; scl_low;
    end endtask

    reg cbit;
    task rec_bit; begin
        sda_high; scl_high;
        cbit = bus_sda;                       // lda pr / lsr
        lop; scl_low;
    end endtask

    reg ackerr;
    task i2c_write(input [7:0] b); integer i; begin
        for (i = 7; i >= 0; i = i - 1) send_bit(b[i]);
        rec_bit;                              // slave ACK slot
        ackerr = cbit;                        // 1 = NAK
    end endtask

    reg [7:0] rdb;
    task i2c_read; integer i; begin
        rdb = 0;
        for (i = 7; i >= 0; i = i - 1) begin rec_bit; rdb[i] = cbit; end
    end endtask

    task i2c_ack;  begin sda_low;  scl_high; lop; scl_low; sda_high; end endtask
    task i2c_nack; begin sda_high; scl_high; lop; scl_low; end endtask

    // i2c_read_first_byte shape: write ptr, STOP, START, addr+R (NOT restart)
    task read_byte(input [6:0] dev, input [7:0] off); begin
        i2c_init; i2c_start;
        i2c_write({dev, 1'b0});
        i2c_write(off);
        i2c_stop; i2c_start;
        i2c_write({dev, 1'b1});
        i2c_read;                              // first byte after addr ACK
        i2c_nack; i2c_stop;                    // i2c_read_stop (single byte)
    end endtask

    task write_byte(input [6:0] dev, input [7:0] off, input [7:0] val); begin
        i2c_init; i2c_start;
        i2c_write({dev, 1'b0});
        i2c_write(off);
        i2c_write(val);
        i2c_stop;
    end endtask

    // i2c_read_first_byte + i2c_read_next_byte*(n-1) + i2c_read_stop:
    // the KERNAL old-style multi-byte read (e.g. $21 mouse packet).
    reg [7:0] dr [0:4];
    task read_multi(input [6:0] dev, input [7:0] off, input integer n); integer i; begin
        i2c_init; i2c_start;
        i2c_write({dev, 1'b0});
        i2c_write(off);
        i2c_stop; i2c_start;
        i2c_write({dev, 1'b1});
        for (i = 0; i < n; i = i + 1) begin
            if (i > 0) i2c_ack;               // ACK previous byte, want more
            i2c_read; dr[i] = rdb;
        end
        i2c_nack; i2c_stop;                   // i2c_read_stop
    end endtask

    // i2c_direct_read + i2c_read_next_byte*(n-1) + i2c_read_stop:
    // the KERNAL new-style ps2data_fetch (read with NO command byte).
    task direct_read(input integer n); integer i; begin
        i2c_init; i2c_start;
        i2c_write({7'h42, 1'b1});
        for (i = 0; i < n; i = i + 1) begin
            if (i > 0) i2c_ack;
            i2c_read; dr[i] = rdb;
        end
        i2c_nack; i2c_stop;
    end endtask

    // one byte of the ps2_to_smc_bridge event stream
    task upush(input [7:0] b); begin
        @(posedge clk); u_byte <= b; u_valid <= 1'b1;
        @(posedge clk); u_valid <= 1'b0;
        @(posedge clk);
    end endtask

    // after every transaction the bus must be released by everyone
    task check_idle(input [127:0] what); begin
        repeat (10) @(posedge clk);
        if (smc_sda_low || rtc_sda_low || !bus_sda || !bus_scl) begin
            $display("[I2CB] FAIL bus not idle after %0s: smc=%b rtc=%b sda=%b scl=%b",
                     what, smc_sda_low, rtc_sda_low, bus_sda, bus_scl);
            errors = errors + 1;
        end
    end endtask

    task exp(input [7:0] got, input [7:0] expct, input [127:0] what); begin
        if (got !== expct) begin
            $display("[I2CB] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    // IRQ-handler-style SMC burst (ps2data_fetch / kbd_scan reads)
    task smc_irq_burst; begin
        read_byte(7'h42, 8'h07); check_idle("smc rd07");
        read_byte(7'h42, 8'h18); check_idle("smc rd18");
        read_byte(7'h42, 8'h21); check_idle("smc rd21");
    end endtask

    integer k, tries;
    reg [7:0] sec0, v0, v1;
    initial begin
        repeat (20) @(posedge clk);
        res_n = 1;
        repeat (20) @(posedge clk);

        // HPS wall clock lands
        hps_rtc[63:0] <= {8'h40, 5'd0, 3'd3, 8'h26, 8'h07, 8'h05, 8'h21, 8'h58, 8'h40};
        hps_rtc[64]   <= ~hps_rtc[64];
        repeat (10) @(posedge clk);

        // ---- 1+2: NVRAM sweep with IRQ-style SMC interleaving ----
        for (k = 0; k < 32; k = k + 1) begin
            read_byte(7'h6F, 8'h40 + k[7:0]);
            exp(rdb, (k == 31) ? 8'hFF : 8'h00, "nvram byte");  // $5F = guard
            check_idle("nvram rd");
            if (k[1:0] == 2'b11) smc_irq_burst;   // IRQ fires between bytes
        end

        // ---- 3: rtc_get_date_time replica (re-read loop must terminate) ----
        tries = 0;
        v0 = 8'hFF;
        while (tries < 10) begin
            read_byte(7'h6F, 8'h00); sec0 = rdb;
            read_byte(7'h6F, 8'h01);
            read_byte(7'h6F, 8'h02);
            read_byte(7'h6F, 8'h03);
            read_byte(7'h6F, 8'h04);
            read_byte(7'h6F, 8'h05); v0 = rdb;
            read_byte(7'h6F, 8'h06); v1 = rdb;
            read_byte(7'h6F, 8'h00);
            tries = tries + 1;
            if (rdb == sec0) tries = 100;          // loop exit like the KERNAL
        end
        if (tries != 100) begin
            $display("[I2CB] FAIL rtc_get_date_time never stabilized");
            errors = errors + 1;
        end
        exp(v0, 8'h07, "month");
        exp(v1, 8'h26, "year");
        check_idle("date-time");

        // ---- 4: NVRAM write + readback, with SMC traffic in between ----
        write_byte(7'h6F, 8'h40, 8'h3C); check_idle("nvram wr");
        smc_irq_burst;
        read_byte(7'h6F, 8'h40);
        exp(rdb, 8'h3C, "nvram readback");
        check_idle("nvram rb");

        // ---- 5: SMC command must survive the STOP inside i2c_read_byte ----
        // (write cmd, STOP, START, addr+R -- the KERNAL never uses a
        // repeated START).  Before the fix these all returned the $41
        // keycode stream: version read 0 -> KERNAL fell back to old-style
        // fetch, and the $21 mouse read returned keycodes -> first byte 0
        // -> packet discarded every frame -> mouse dead.
        read_byte(7'h42, 8'h30); exp(rdb, 8'd48, "smc ver major");
        check_idle("smc ver1");
        read_byte(7'h42, 8'h31); exp(rdb, 8'd1,  "smc ver minor");
        read_byte(7'h42, 8'h22); exp(rdb, 8'h03, "mouse device id");
        check_idle("mouse id");

        // ---- 6: ps2data_init -- default read op = keycode only ($41) ----
        write_byte(7'h42, 8'h40, 8'h41); check_idle("dflt-op 41");
        upush(8'h1C);                         // PS/2 'a' make -> IBM keycode 31
        direct_read(1); exp(dr[0], 8'd31,  "direct keycode");
        direct_read(1); exp(dr[0], 8'h00,  "kbd fifo drained");
        check_idle("direct kbd");

        // ---- 7: MOUSE 1 -- default read op = keycode + mouse ($43),
        //         then the per-frame ps2data_fetch direct read ----
        write_byte(7'h42, 8'h40, 8'h43); check_idle("dflt-op 43");
        upush(8'hFF); upush(8'h09); upush(8'h12); upush(8'hF7); // status/dx/dy
        upush(8'h01);                                           // wheel +1
        direct_read(5);
        exp(dr[0], 8'h00, "ps2data kbd byte");
        exp(dr[1], 8'h09, "mouse status");
        exp(dr[2], 8'h12, "mouse dx");
        exp(dr[3], 8'hF7, "mouse dy");
        exp(dr[4], 8'h01, "mouse wheel");
        direct_read(2);                       // packet consumed -> zero first
        exp(dr[0], 8'h00, "ps2 kbd byte 2");
        exp(dr[1], 8'h00, "mouse buf empty");
        check_idle("ps2data fetch");

        // ---- 8: old-style mouse read ($21 via write+STOP+read) ----
        upush(8'hFF); upush(8'h0B); upush(8'hFE); upush(8'h01);
        upush(8'h0F);                         // wheel -1 (nibble format)
        read_multi(7'h42, 8'h21, 4);
        exp(dr[0], 8'h0B, "old mouse status");
        exp(dr[1], 8'hFE, "old mouse dx");
        exp(dr[2], 8'h01, "old mouse dy");
        exp(dr[3], 8'h0F, "old mouse wheel");
        read_multi(7'h42, 8'h21, 4);
        exp(dr[0], 8'h00, "old mouse empty");
        check_idle("old mouse rd");

        // ---- 9: FULL CHAIN mouse: ps2_mouse toggle -> real bridge (5-byte
        //         burst on consecutive cycles) -> SMC -> per-frame $43 read.
        //         This is everything south of the hps_io 2-FF sync. ----
        ps2_mwheel      <= 8'hFF;                   // wheel -1 -> nibble $F
        ps2_mouse[23:0] <= {8'h05, 8'h13, 8'h28};   // dy=$05 dx=$13 status=$28
        ps2_mouse[24]   <= ~ps2_mouse[24];          // (status: bit3|Y-sign)
        repeat (20) @(posedge clk);                 // bridge emits FF 28 13 05 0F
        direct_read(5);
        exp(dr[0], 8'h00, "chain kbd byte");
        exp(dr[1], 8'h28, "chain status");
        exp(dr[2], 8'h13, "chain dx");
        exp(dr[3], 8'h05, "chain dy");
        exp(dr[4], 8'h0F, "chain wheel");
        // key event + mouse event back-to-back through the bridge, then the
        // frame read must deliver keycode AND packet; wheel +20 saturates to
        // the KERNAL nibble max +7
        ps2_key         <= {~ps2_key[10], 1'b1, 1'b0, 8'h1C};  // 'a' make
        @(posedge clk);
        ps2_mwheel      <= 8'd20;
        ps2_mouse[23:0] <= {8'hFB, 8'hF0, 8'h39};   // dy=-5 dx=-16 st=$39
        ps2_mouse[24]   <= ~ps2_mouse[24];
        repeat (30) @(posedge clk);
        direct_read(5);
        exp(dr[0], 8'd31,  "chain kbd+mouse");
        exp(dr[1], 8'h39, "chain status 2");
        exp(dr[2], 8'hF0, "chain dx 2");
        exp(dr[3], 8'hFB, "chain dy 2");
        exp(dr[4], 8'h07, "chain wheel sat");
        direct_read(2);                             // consumed
        exp(dr[1], 8'h00, "chain consumed");
        check_idle("chain mouse");

        if (errors == 0) $display("[I2CB] *** I2C BOOT REPLAY: ALL PASS ***");
        else             $display("[I2CB] *** I2C BOOT REPLAY: %0d FAILURES ***", errors);
        $finish;
    end

    initial begin #80000000; $display("[I2CB] TIMEOUT -- BUS WEDGED (smc=%b rtc=%b sda=%b scl=%b)",
                                      smc_sda_low, rtc_sda_low, bus_sda, bus_scl); $finish; end
endmodule
