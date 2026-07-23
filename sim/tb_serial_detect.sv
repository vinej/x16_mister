`timescale 1ns/1ps

module tb_serial_detect;
    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg cs0 = 1'b0;
    reg cs1 = 1'b0;
    reg rwn = 1'b1;
    reg enable = 1'b1;
    reg [2:0] addr = 3'd0;
    reg [7:0] di = 8'h00;
    reg shared_rxd = 1'b1;
    reg serial_sel = 1'b0;
    wire [7:0] do0;
    wire [7:0] do1;
    wire tx0, rts0, dtr0;
    wire tx1, rts1, dtr1;
    wire rxd0 = serial_sel ? 1'b1 : shared_rxd;
    wire rxd1 = serial_sel ? shared_rxd : 1'b1;

    always #5 clk = ~clk;

    x16_serial_card u0 (
        .clk(clk), .reset_n(reset_n), .cs(cs0), .rwn(rwn),
        .enable(enable), .addr(addr), .di(di), .do_o(do0),
        .uart_rxd(rxd0), .uart_cts(1'b1), .uart_dsr(1'b1),
        .uart_ri(1'b0),
        .uart_txd(tx0), .uart_rts(rts0),
        .uart_dtr(dtr0)
    );

    x16_serial_card u1 (
        .clk(clk), .reset_n(reset_n), .cs(cs1), .rwn(rwn),
        .enable(enable), .addr(addr), .di(di), .do_o(do1),
        .uart_rxd(rxd1), .uart_cts(1'b1), .uart_dsr(1'b1),
        .uart_ri(1'b0),
        .uart_txd(tx1), .uart_rts(rts1),
        .uart_dtr(dtr1)
    );

    task write0;
        input [2:0] reg_addr;
        input [7:0] value;
        begin
            addr = reg_addr;
            di = value;
            rwn = 1'b0;
            cs0 = 1'b1;
            @(posedge clk);
            #1;
            cs0 = 1'b0;
            rwn = 1'b1;
        end
    endtask

    // External 8-N-1 byte at divisor 8. bit_ticks() resolves to 69 and the
    // implementation holds each bit for ticks+1 clocks.
    task send_shared_byte;
        input [7:0] value;
        integer bit_index;
        begin
            @(negedge clk);
            shared_rxd = 1'b0;
            repeat (70) @(posedge clk);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                shared_rxd = value[bit_index];
                repeat (70) @(posedge clk);
            end
            shared_rxd = 1'b1;
            repeat (140) @(posedge clk);
        end
    endtask

    task write1;
        input [2:0] reg_addr;
        input [7:0] value;
        begin
            addr = reg_addr;
            di = value;
            rwn = 1'b0;
            cs1 = 1'b1;
            @(posedge clk);
            #1;
            cs1 = 1'b0;
            rwn = 1'b1;
        end
    endtask

    task expect0;
        input [2:0] reg_addr;
        input [7:0] value;
        begin
            addr = reg_addr;
            #1;
            if (do0 !== value) begin
                $display("FAIL: UART0 register %0d = %02x, expected %02x", reg_addr, do0, value);
                $stop;
            end
        end
    endtask

    task expect1_mask;
        input [2:0] reg_addr;
        input [7:0] mask;
        input [7:0] value;
        begin
            addr = reg_addr;
            #1;
            if ((do1 & mask) !== value) begin
                $display("FAIL: UART1 register %0d masked by %02x = %02x, expected %02x",
                         reg_addr, mask, do1 & mask, value);
                $stop;
            end
        end
    endtask

    task expect0_mask;
        input [2:0] reg_addr;
        input [7:0] mask;
        input [7:0] value;
        begin
            addr = reg_addr;
            #1;
            if ((do0 & mask) !== value) begin
                $display("FAIL: UART0 register %0d masked by %02x = %02x, expected %02x",
                         reg_addr, mask, do0 & mask, value);
                $stop;
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        reset_n = 1'b1;
        @(posedge clk);

        // ROMTERM first checks the UART reset state and scratch register.
        expect0(3'd1, 8'h00);
        expect0(3'd3, 8'h00);
        expect0(3'd4, 8'h00);
        if (rts0 !== 1'b1 || dtr0 !== 1'b1) begin
            $display("FAIL: reset modem outputs must be deasserted high");
            $stop;
        end
        write0(3'd7, 8'h69);
        expect0(3'd7, 8'h69);

        // ROMTERM identifies a usable UART when the companion RI input does
        // not follow DTR. The physical card does not cross-connect them.
        write0(3'd4, 8'h01);
        if (rts0 !== 1'b1 || dtr0 !== 1'b0) begin
            $display("FAIL: asserted DTR must drive active-low HPS DSR");
            $stop;
        end
        expect1_mask(3'd6, 8'h40, 8'h00);
        write0(3'd4, 8'h00);
        expect1_mask(3'd6, 8'h40, 8'h00);

        // The UARTs remain independent in the reverse direction as well.
        write1(3'd4, 8'h01);
        expect0_mask(3'd6, 8'h40, 8'h00);
        write1(3'd4, 8'h00);
        expect0_mask(3'd6, 8'h40, 8'h00);

        // Preserve the local 16450 loopback behavior used by other probes.
        write0(3'd4, 8'h10);
        if (rts0 !== 1'b1 || dtr0 !== 1'b1) begin
            $display("FAIL: loopback must deassert physical modem outputs");
            $stop;
        end
        write0(3'd0, 8'h41);
        expect0(3'd5, 8'h61);
        expect0(3'd0, 8'h41);

        // With the MiSTer UART mapped to $9FE0, a modem response must enter
        // only UART0. Companion traffic must not change that fixed mapping.
        write0(3'd4, 8'h00);
        write0(3'd3, 8'h03);
        write1(3'd3, 8'h03);
        write0(3'd2, 8'h02);
        write1(3'd2, 8'h02);
        write0(3'd4, 8'h03);
        if (rts0 !== 1'b0 || dtr0 !== 1'b0) begin
            $display("FAIL: asserted RTS/DTR must drive active-low HPS pins");
            $stop;
        end

        write0(3'd0, 8'h41);
        write1(3'd0, 8'h42);
        write1(3'd2, 8'h01);
        write1(3'd4, 8'h03);
        send_shared_byte(8'h55);
        expect0_mask(3'd5, 8'h01, 8'h01);
        expect1_mask(3'd5, 8'h01, 8'h00);
        expect0(3'd0, 8'h55);

        write0(3'd2, 8'h02);
        write1(3'd2, 8'h02);
        serial_sel = 1'b1;
        write1(3'd0, 8'h42);
        write0(3'd0, 8'h41);
        write0(3'd2, 8'h01);
        write0(3'd4, 8'h03);
        send_shared_byte(8'hAA);
        expect0_mask(3'd5, 8'h01, 8'h00);
        expect1_mask(3'd5, 8'h01, 8'h01);
        addr = 3'd0;
        #1;
        if (do1 !== 8'hAA) begin
            $display("FAIL: UART1 received %02x, expected AA", do1);
            $stop;
        end

        $display("PASS: ROMTERM detection and fixed-port routing");
        $finish;
    end
endmodule
