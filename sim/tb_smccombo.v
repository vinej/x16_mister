`timescale 1ns/1ps
// ============================================================================
// tb_smccombo.v -- SMC keyboard combo detection (jyv 2026-07-07):
//   Ctrl+Alt+Del            -> reset_req   (system reset)
//   Ctrl+Alt+PrtScr/Restore -> nmi_req     (both E0 7C and SysRq $84 forms)
// through the REAL ps2_to_smc_bridge -> smc_x16 path, hps_io-shaped ps2_key
// events, exactly the x16.sv integration.  Negative cases: combos without
// the full modifier set must do nothing; normal keys still reach the FIFO.
// ============================================================================
module tb_smccombo;
    integer errors = 0;

    // 8 MHz cpu_clk
    reg clk = 0; always #62.5 clk = ~clk;
    reg reset_n = 0;

    reg [10:0] ps2_key = 11'd0;

    wire [7:0] uart_byte;
    wire       uart_byte_valid;

    ps2_to_smc_bridge u_bridge (
        .clk            (clk),
        .reset_n        (reset_n),
        .ps2_key        (ps2_key),
        .ps2_mouse      (25'd0),
        .ps2_mouse_wheel(8'd0),
        .uart_byte      (uart_byte),
        .uart_byte_valid(uart_byte_valid)
    );

    wire reset_req, nmi_req, power_off_req;
    wire [4:0] kbd_count;

    smc_x16 u_smc (
        .clk            (clk),
        .reset_n        (reset_n),
        .sda_bus        (1'b1),          // I2C idle
        .scl_bus        (1'b1),
        .sda_drive_low  (),
        .uart_byte      (uart_byte),
        .uart_byte_valid(uart_byte_valid),
        .power_off_req  (power_off_req),
        .reset_req      (reset_req),
        .nmi_req        (nmi_req),
        .act_led_r      (),
        .dbg_kbd_count  (kbd_count),
        .dbg_saw_start(), .dbg_saw_addr_match(), .dbg_saw_byte(),
        .dbg_saw_repeat(), .dbg_saw_stop(), .dbg_saw_tx(),
        .dbg_last_cmd(), .dbg_last_addr_byte(),
        .dbg_kbd_pop(), .dbg_tx_byte()
    );

    // sticky pulse catchers
    reg saw_rst = 0, saw_nmi = 0;
    always @(posedge clk) begin
        if (reset_req) saw_rst <= 1'b1;
        if (nmi_req)   saw_nmi <= 1'b1;
    end

    // hps_io-shaped key event: toggle strobe, then let the bridge emit
    // (<=3 bytes) and the SMC parse.
    task key(input ext, input pressed, input [7:0] code); begin
        @(posedge clk);
        ps2_key <= {~ps2_key[10], pressed, ext, code};
        repeat (30) @(posedge clk);
    end endtask

    task expect_pulse(input rst_e, input nmi_e, input [255:0] what); begin
        if (saw_rst !== rst_e) begin
            $display("[CMB ] FAIL %0s: reset_req=%b exp %b", what, saw_rst, rst_e);
            errors = errors + 1;
        end
        if (saw_nmi !== nmi_e) begin
            $display("[CMB ] FAIL %0s: nmi_req=%b exp %b", what, saw_nmi, nmi_e);
            errors = errors + 1;
        end
        saw_rst = 0; saw_nmi = 0;
    end endtask

    initial begin
        repeat (10) @(posedge clk);
        reset_n = 1;
        repeat (10) @(posedge clk);

        // --- Ctrl+Alt+Del -> reset ---
        key(1'b0, 1'b1, 8'h14);            // LCtrl make
        key(1'b0, 1'b1, 8'h11);            // LAlt make
        expect_pulse(0, 0, "mods alone");
        key(1'b1, 1'b1, 8'h71);            // Del make (E0 71)
        expect_pulse(1, 0, "Ctrl+Alt+Del");

        // repeat Del while held -> fires again
        key(1'b1, 1'b0, 8'h71);            // Del break
        key(1'b1, 1'b1, 8'h71);            // Del make again
        expect_pulse(1, 0, "Ctrl+Alt+Del again");
        key(1'b1, 1'b0, 8'h71);

        // --- PrtScr forms -> NMI ---
        key(1'b1, 1'b1, 8'h7C);            // PrtScr (E0 7C, MiSTer form)
        expect_pulse(0, 1, "Ctrl+Alt+PrtScr (E0 7C)");
        key(1'b1, 1'b0, 8'h7C);
        key(1'b0, 1'b1, 8'h84);            // SysRq ($84, real-keyboard form)
        expect_pulse(0, 1, "Ctrl+Alt+SysRq ($84)");
        key(1'b0, 1'b0, 8'h84);

        // --- releasing a modifier disarms ---
        key(1'b0, 1'b0, 8'h14);            // LCtrl break
        key(1'b1, 1'b1, 8'h71);            // Del
        key(1'b1, 1'b0, 8'h71);
        key(1'b1, 1'b1, 8'h7C);            // PrtScr
        key(1'b1, 1'b0, 8'h7C);
        expect_pulse(0, 0, "Alt only: no combo");

        // --- right-hand modifiers work too (RCtrl = E0 14) ---
        key(1'b1, 1'b1, 8'h14);            // RCtrl make
        key(1'b1, 1'b1, 8'h71);            // Del
        expect_pulse(1, 0, "RCtrl+LAlt+Del");
        key(1'b1, 1'b0, 8'h71);
        key(1'b1, 1'b0, 8'h14);            // RCtrl break
        key(1'b0, 1'b0, 8'h11);            // LAlt break

        // --- Del alone: nothing ---
        key(1'b1, 1'b1, 8'h71);
        key(1'b1, 1'b0, 8'h71);
        expect_pulse(0, 0, "Del alone");

        // --- normal keys still flow into the FIFO ---
        if (kbd_count == 5'd0) begin
            $display("[CMB ] FAIL: keyboard FIFO empty after key traffic");
            errors = errors + 1;
        end

        if (errors == 0) $display("[CMB ] ALL TESTS PASS");
        else             $display("[CMB ] %0d ERRORS", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("[CMB ] TIMEOUT");
        $finish;
    end

endmodule
