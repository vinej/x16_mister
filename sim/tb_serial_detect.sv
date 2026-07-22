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
    wire [7:0] do0;
    wire [7:0] do1;
    wire tx0, rts0, dtr0;
    wire tx1, rts1, dtr1;

    always #5 clk = ~clk;

    x16_serial_card u0 (
        .clk(clk), .reset_n(reset_n), .cs(cs0), .rwn(rwn),
        .enable(enable), .addr(addr), .di(di), .do_o(do0),
        .uart_rxd(1'b1), .uart_cts(1'b1), .uart_dsr(1'b1),
        .uart_ri(1'b0), .uart_txd(tx0), .uart_rts(rts0),
        .uart_dtr(dtr0)
    );

    x16_serial_card u1 (
        .clk(clk), .reset_n(reset_n), .cs(cs1), .rwn(rwn),
        .enable(enable), .addr(addr), .di(di), .do_o(do1),
        .uart_rxd(1'b1), .uart_cts(1'b1), .uart_dsr(1'b1),
        .uart_ri(1'b0), .uart_txd(tx1), .uart_rts(rts1),
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
        write0(3'd7, 8'h69);
        expect0(3'd7, 8'h69);

        // ROMTERM identifies a usable UART when the companion RI input does
        // not follow DTR. The physical card does not cross-connect them.
        write0(3'd4, 8'h01);
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
        write0(3'd0, 8'h41);
        expect0(3'd5, 8'h61);
        expect0(3'd0, 8'h41);

        $display("PASS: ROMTERM serial-card detection and UART loopback");
        $finish;
    end
endmodule
