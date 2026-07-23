`timescale 1ns/1ps
// ============================================================================
// tb_periph.v -- unit tests for the 2026-07-05 peripheral batch, using the
// EXACT shipped modules:
//   1. I2S loop  : vera's real dacif.v serializes known 24-bit samples ->
//                  rtl/x16_periph.sv i2s_rx must recover their top 16 bits.
//   2. snes_pad  : driven with the KERNAL r49 joystick_scan protocol (latch
//                  pulse, then 24x CLK-low/read/CLK-high); the 24 read bits
//                  must match the SNES report for a known button state.
//   3. mouse     : rtl/ps2_to_smc_bridge.sv gets key + mouse events; the
//                  emitted uart stream must be the key bytes and the atomic
//                  FF S DX DY packet.
// ============================================================================
module tb_periph;
    integer errors = 0;

    // ------------------------------------------------------------------
    // 1. I2S: dacif -> i2s_rx
    // ------------------------------------------------------------------
    reg         aclk = 0;  always #20 aclk = ~aclk;   // 25 MHz
    reg         arst = 1;
    reg  [23:0] left_in  = 24'h123456;
    reg  [23:0] right_in = 24'hFEDCBA;                // negative sample
    wire        next_sample, lrck, bck, sdata;
    wire signed [15:0] left_out, right_out;

    dacif u_dac (
        .rst(arst), .clk(aclk),
        .next_sample(next_sample),
        .left_data(left_in), .right_data(right_in),
        .i2s_lrck(lrck), .i2s_bck(bck), .i2s_data(sdata)
    );

    i2s_rx u_rx (
        .clk(aclk), .lrck(lrck), .bck(bck), .data(sdata),
        .left(left_out), .right(right_out)
    );

    task i2s_test;
        begin
            arst = 1;
            repeat (4) @(posedge aclk);
            arst = 0;
            // several full frames (frame = 512 aclk), then check
            repeat (4 * 512) @(posedge aclk);
            if (left_out !== 16'h1234) begin
                $display("[I2S ] FAIL left  = %04x (expect 1234)", left_out);
                errors = errors + 1;
            end
            if (right_out !== 16'hFEDC) begin
                $display("[I2S ] FAIL right = %04x (expect FEDC)", right_out);
                errors = errors + 1;
            end
            // change samples, verify they follow
            left_in  = 24'h800000;    // most negative
            right_in = 24'h7FFFFF;    // most positive
            repeat (4 * 512) @(posedge aclk);
            if (left_out !== 16'h8000) begin
                $display("[I2S ] FAIL left2 = %04x (expect 8000)", left_out);
                errors = errors + 1;
            end
            if (right_out !== 16'h7FFF) begin
                $display("[I2S ] FAIL right2= %04x (expect 7FFF)", right_out);
                errors = errors + 1;
            end
            $display("[I2S ] done (left=%04x right=%04x)", left_out, right_out);
        end
    endtask

    // ------------------------------------------------------------------
    // 2. snes_pad driven like KERNAL r49 joystick_scan
    // ------------------------------------------------------------------
    reg         cclk = 0;  always #62.5 cclk = ~cclk;  // 8 MHz
    reg         crst_n = 0;
    reg  [11:0] joy = 12'h000;
    reg         p_latch = 0, p_jclk = 1;
    wire        p_data;
    reg  [23:0] rxbits;
    integer     bi;

    snes_pad u_pad (
        .clk(cclk), .reset_n(crst_n),
        .joy(joy), .latch(p_latch), .jclk(p_jclk), .data(p_data)
    );

    task pad_scan;   // returns 24 bits in rxbits, MSB = first bit read
        begin
            // latch pulse while clk high (KERNAL: latch+clk high, then low)
            p_jclk = 1; p_latch = 0; repeat (2) @(posedge cclk);
            p_latch = 1;             repeat (4) @(posedge cclk);
            p_latch = 0;             repeat (2) @(posedge cclk);
            for (bi = 0; bi < 24; bi = bi + 1) begin
                p_jclk = 0; repeat (2) @(posedge cclk);
                rxbits = {rxbits[22:0], p_data};   // read while clk low
                p_jclk = 1; repeat (2) @(posedge cclk);
            end
        end
    endtask

    // expected report for a joy vector (mirrors snes_pad's mapping)
    function [23:0] expect_report(input [11:0] j);
        expect_report = { ~j[5], ~j[7], ~j[10], ~j[11],
                          ~j[3], ~j[2], ~j[1],  ~j[0],
                          ~j[4], ~j[6], ~j[8],  ~j[9],
                          4'b1111, 8'b0 };
    endfunction

    task pad_test;
        begin
            crst_n = 1;
            joy = 12'b0000_0000_0000;              // nothing pressed
            pad_scan;
            if (rxbits !== expect_report(joy)) begin
                $display("[PAD ] FAIL idle: got %06x exp %06x", rxbits, expect_report(joy));
                errors = errors + 1;
            end
            joy = 12'b1010_0110_1001;              // arbitrary buttons
            pad_scan;
            if (rxbits !== expect_report(joy)) begin
                $display("[PAD ] FAIL mixed: got %06x exp %06x", rxbits, expect_report(joy));
                errors = errors + 1;
            end
            $display("[PAD ] done (report=%06x)", rxbits);
        end
    endtask

    // ------------------------------------------------------------------
    // 3. mouse / keyboard bridge
    // ------------------------------------------------------------------
    reg  [10:0] ps2_key    = 11'h000;
    reg  [24:0] ps2_mouse  = 25'h0;
    reg  [7:0]  ps2_mwheel = 8'h00;
    wire [7:0]  ubyte;
    wire        uvalid;
    reg  [7:0]  cap [0:31];
    integer     ncap = 0;

    // sim-scale typematic: 200-cycle delay, then every 80 cycles
    ps2_to_smc_bridge #(.TPM_DELAY(200), .TPM_RATE(80)) u_bridge (
        .clk(cclk), .reset_n(crst_n),
        .ps2_key(ps2_key), .ps2_mouse(ps2_mouse),
        .ps2_mouse_wheel(ps2_mwheel),
        .uart_byte(ubyte), .uart_byte_valid(uvalid)
    );

    always @(posedge cclk) if (uvalid && ncap < 32) begin
        cap[ncap] <= ubyte;
        ncap      <= ncap + 1;
    end

    task check_byte(input integer i, input [7:0] v, input [127:0] what);
        if (cap[i] !== v) begin
            $display("[MOUS] FAIL %0s: cap[%0d]=%02x exp %02x", what, i, cap[i], v);
            errors = errors + 1;
        end
    endtask

    // Toggle a new event every cpu clock.  The bridge must keep observing the
    // input while it serializes earlier events; the old idle-only detector
    // dropped alternating toggles in this sequence.
    task key_burst_event(input pressed, input [7:0] code); begin
        @(negedge cclk);
        ps2_key <= {~ps2_key[10], pressed, 1'b0, code};
    end endtask

    task key_burst_test; begin
        ncap = 0;
        key_burst_event(1'b1, 8'h1C);
        key_burst_event(1'b1, 8'h32);
        key_burst_event(1'b1, 8'h21);
        key_burst_event(1'b1, 8'h23);
        key_burst_event(1'b1, 8'h24);
        key_burst_event(1'b1, 8'h2B);
        repeat (30) @(posedge cclk);
        key_burst_event(1'b0, 8'h1C);
        key_burst_event(1'b0, 8'h32);
        key_burst_event(1'b0, 8'h21);
        key_burst_event(1'b0, 8'h23);
        key_burst_event(1'b0, 8'h24);
        key_burst_event(1'b0, 8'h2B);
        repeat (50) @(posedge cclk);

        if (ncap !== 18) begin
            $display("[KEYQ] FAIL: captured %0d bytes (expect 18)", ncap);
            errors = errors + 1;
        end
        check_byte(0,  8'h1C, "burst make 1");
        check_byte(1,  8'h32, "burst make 2");
        check_byte(2,  8'h21, "burst make 3");
        check_byte(3,  8'h23, "burst make 4");
        check_byte(4,  8'h24, "burst make 5");
        check_byte(5,  8'h2B, "burst make 6");
        check_byte(6,  8'hF0, "burst break prefix 1");
        check_byte(7,  8'h1C, "burst break 1");
        check_byte(8,  8'hF0, "burst break prefix 2");
        check_byte(9,  8'h32, "burst break 2");
        check_byte(10, 8'hF0, "burst break prefix 3");
        check_byte(11, 8'h21, "burst break 3");
        check_byte(12, 8'hF0, "burst break prefix 4");
        check_byte(13, 8'h23, "burst break 4");
        check_byte(14, 8'hF0, "burst break prefix 5");
        check_byte(15, 8'h24, "burst break 5");
        check_byte(16, 8'hF0, "burst break prefix 6");
        check_byte(17, 8'h2B, "burst break 6");
        $display("[KEYQ] done (%0d bytes, no dropped events)", ncap);
    end endtask

    task bridge_test;
        begin
            ncap = 0;
            // key make: code $1C ('A'), not extended
            ps2_key[10] <= ~ps2_key[10]; ps2_key[9] <= 1'b1; ps2_key[8] <= 1'b0; ps2_key[7:0] <= 8'h1C;
            repeat (10) @(posedge cclk);
            // mouse event: status $09 (left btn + always-1), dx $12, dy $F7,
            // wheel -3 (saturating nibble)
            ps2_mwheel      <= 8'hFD;
            ps2_mouse[23:0] <= {8'hF7, 8'h12, 8'h09};
            ps2_mouse[24]   <= ~ps2_mouse[24];
            repeat (12) @(posedge cclk);
            // key break of $1C
            ps2_key[10] <= ~ps2_key[10]; ps2_key[9] <= 1'b0;
            repeat (10) @(posedge cclk);

            if (ncap !== 8) begin
                $display("[MOUS] FAIL: captured %0d bytes (expect 8)", ncap);
                errors = errors + 1;
            end
            check_byte(0, 8'h1C, "key make");
            check_byte(1, 8'hFF, "pkt hdr");
            check_byte(2, 8'h09, "status");
            check_byte(3, 8'h12, "dx");
            check_byte(4, 8'hF7, "dy");
            check_byte(5, 8'h0D, "wheel nibble");
            check_byte(6, 8'hF0, "break prefix");
            check_byte(7, 8'h1C, "key break");
            $display("[MOUS] done (%0d bytes)", ncap);
        end
    endtask

    // ------------------------------------------------------------------
    // 4. rtc_x16 (MCP7940N @ $6F) via a bit-banged I2C master, KERNAL-style
    //    transactions (write ptr, repeated START, read; multi-byte auto-inc)
    // ------------------------------------------------------------------
    localparam integer RTC_HZ = 50000;         // fast "seconds" for the TB
    reg         m_sda_rel = 1, m_scl_rel = 1;  // master release (open drain)
    wire        rtc_sda_low;
    wire        bus_sda_r = m_sda_rel & ~rtc_sda_low;
    wire        bus_scl_r = m_scl_rel;
    reg  [64:0] hps_rtc = 65'd0;
    reg         ackbit;
    reg  [7:0]  rdb;

    rtc_x16 #(.CLK_HZ(RTC_HZ)) u_rtc (
        .clk(cclk), .reset_n(crst_n),
        .sda_bus(bus_sda_r), .scl_bus(bus_scl_r),
        .sda_drive_low(rtc_sda_low),
        .hps_rtc(hps_rtc)
    );

    task iw; begin repeat (5) @(posedge cclk); end endtask  // quarter bit

    task i2c_start; begin
        m_sda_rel = 1; iw; m_scl_rel = 1; iw;
        m_sda_rel = 0; iw; m_scl_rel = 0; iw;
    end endtask

    task i2c_stop; begin
        m_sda_rel = 0; iw; m_scl_rel = 1; iw; m_sda_rel = 1; iw;
    end endtask

    task i2c_wbyte(input [7:0] b); integer i; begin
        for (i = 7; i >= 0; i = i - 1) begin
            m_sda_rel = b[i]; iw; m_scl_rel = 1; iw; iw; m_scl_rel = 0; iw;
        end
        m_sda_rel = 1; iw; m_scl_rel = 1; iw;   // ACK slot
        ackbit = bus_sda_r;                     // 0 = slave ACKed
        iw; m_scl_rel = 0; iw;
    end endtask

    task i2c_rbyte(input nack); integer i; begin
        m_sda_rel = 1; rdb = 0;
        for (i = 7; i >= 0; i = i - 1) begin
            iw; m_scl_rel = 1; iw; rdb[i] = bus_sda_r; iw; m_scl_rel = 0; iw;
        end
        m_sda_rel = nack; iw; m_scl_rel = 1; iw; iw; m_scl_rel = 0;   // ACK=low
        m_sda_rel = 1; iw;
    end endtask

    // KERNAL i2c_read_byte shape: START,addr+W,ptr, reSTART,addr+R, byte,NACK,STOP
    task rtc_read(input [7:0] ptr); begin
        i2c_start; i2c_wbyte({7'h6F, 1'b0}); i2c_wbyte(ptr);
        i2c_start; i2c_wbyte({7'h6F, 1'b1}); i2c_rbyte(1); i2c_stop;
    end endtask

    task rtc_check(input [7:0] ptr, input [7:0] exp, input [127:0] what); begin
        rtc_read(ptr);
        if (rdb !== exp) begin
            $display("[RTC ] FAIL %0s: reg %02x = %02x exp %02x", what, ptr, rdb, exp);
            errors = errors + 1;
        end
    end endtask

    task rtc_test; begin
        // HPS wall clock arrives: 2024-02-28 (leap yr) 21:59:23, wday=3(Wed)
        hps_rtc[63:0] <= {8'h40, 5'd0, 3'd3, 8'h24, 8'h02, 8'h28, 8'h21, 8'h59, 8'h23};
        hps_rtc[64]   <= ~hps_rtc[64];
        repeat (10) @(posedge cclk);

        rtc_check(8'h00, 8'hA3, "sec (ST|23)");
        rtc_check(8'h01, 8'h59, "min");
        rtc_check(8'h02, 8'h21, "hour");
        rtc_check(8'h03, 8'h2C, "wkday (OSCRUN|VBATEN|4)");
        rtc_check(8'h04, 8'h28, "date");
        rtc_check(8'h05, 8'h02, "month");
        rtc_check(8'h06, 8'h24, "year");
        rtc_check(8'h5F, 8'hFF, "checksum guard");

        // NVRAM multi-byte write $40.. then auto-increment read-back
        i2c_start; i2c_wbyte({7'h6F, 1'b0}); i2c_wbyte(8'h40);
        i2c_wbyte(8'hAB); i2c_wbyte(8'hCD); i2c_stop;
        i2c_start; i2c_wbyte({7'h6F, 1'b0}); i2c_wbyte(8'h40);
        i2c_start; i2c_wbyte({7'h6F, 1'b1});
        i2c_rbyte(0);
        if (rdb !== 8'hAB) begin $display("[RTC ] FAIL nvram0=%02x", rdb); errors = errors + 1; end
        i2c_rbyte(1); i2c_stop;
        if (rdb !== 8'hCD) begin $display("[RTC ] FAIL nvram1=%02x", rdb); errors = errors + 1; end

        // KERNAL clock_set shape: stop clock, write fields, start clock.
        // 2023-02-28 (non-leap) 23:59:59 -> first tick must roll to
        // 2023-03-01 00:00:00 (full cascade incl. non-leap February).
        i2c_start; i2c_wbyte({7'h6F, 1'b0}); i2c_wbyte(8'h00);
        i2c_wbyte(8'h00);                       // sec=0, ST=0 (stop)
        i2c_wbyte(8'h59); i2c_wbyte(8'h23);     // min, hour
        i2c_wbyte(8'h03);                       // wkday
        i2c_wbyte(8'h28); i2c_wbyte(8'h02); i2c_wbyte(8'h23); // date,month,year
        i2c_stop;
        i2c_start; i2c_wbyte({7'h6F, 1'b0}); i2c_wbyte(8'h00);
        i2c_wbyte(8'hD9); i2c_stop;             // sec=59, ST=1 (start)

        // a HPS update must now be IGNORED (CPU owns the clock)
        hps_rtc[64] <= ~hps_rtc[64];

        // wait 2.2 "seconds": tick1 = midnight rollover, tick2 -> :01
        repeat (RTC_HZ * 2 + RTC_HZ / 5) @(posedge cclk);
        rtc_check(8'h00, 8'h81, "sec after rollover");
        rtc_check(8'h01, 8'h00, "min after rollover");
        rtc_check(8'h02, 8'h00, "hour after rollover");
        rtc_check(8'h03, 8'h2C, "wkday 3->4");
        rtc_check(8'h04, 8'h01, "date -> 01");
        rtc_check(8'h05, 8'h03, "month -> March (non-leap Feb)");
        rtc_check(8'h06, 8'h23, "year kept");
        $display("[RTC ] done");
    end endtask

    // Typematic: hold a key past TPM_DELAY -> the bridge must re-emit the
    // make; a foreign break must NOT stop it; the held key's break must.
    integer n1, n2, n3, tk;
    task typematic_test;
        begin
            ncap = 0;
            // make 'd' (raw $23) and hold
            ps2_key[10] <= ~ps2_key[10]; ps2_key[9] <= 1'b1; ps2_key[8] <= 1'b0; ps2_key[7:0] <= 8'h23;
            repeat (500) @(posedge cclk);       // delay(200) + >=3 rates(80)
            n1 = ncap;
            if (n1 < 4) begin
                $display("[TPM ] FAIL: %0d makes after hold (expect >=4)", n1);
                errors = errors + 1;
            end
            for (tk = 0; tk < n1; tk = tk + 1)
                if (cap[tk] !== 8'h23) begin
                    $display("[TPM ] FAIL: cap[%0d]=%02x (expect 23)", tk, cap[tk]);
                    errors = errors + 1;
                end
            // break of a DIFFERENT key: repeat must continue
            ps2_key[10] <= ~ps2_key[10]; ps2_key[9] <= 1'b0; ps2_key[7:0] <= 8'h1C;
            repeat (180) @(posedge cclk);       // F0,1C + >=2 more repeats
            n2 = ncap;
            if (n2 < n1 + 3) begin
                $display("[TPM ] FAIL: foreign break stopped repeat (%0d -> %0d)", n1, n2);
                errors = errors + 1;
            end
            // break of the held key: repeat must stop
            ps2_key[10] <= ~ps2_key[10]; ps2_key[9] <= 1'b0; ps2_key[7:0] <= 8'h23;
            repeat (60) @(posedge cclk);        // F0,23 flushes out
            n3 = ncap;
            repeat (400) @(posedge cclk);
            if (ncap !== n3) begin
                $display("[TPM ] FAIL: repeats after release (%0d -> %0d)", n3, ncap);
                errors = errors + 1;
            end
            $display("[TPM ] done (held=%0d, +foreign-break=%0d, stopped at %0d)", n1, n2, n3);
        end
    endtask

    // ------------------------------------------------------------------
    initial begin
        fork
            begin
                i2s_test;
            end
            begin
                repeat (4) @(posedge cclk);
                pad_test;
                bridge_test;
                key_burst_test;
                typematic_test;
                rtc_test;
            end
        join
        if (errors == 0) $display("[TBC] *** PERIPH TESTS: ALL PASS ***");
        else             $display("[TBC] *** PERIPH TESTS: %0d FAILURES ***", errors);
        $finish;
    end
endmodule
