`timescale 1ns/1ps

// Runs ROMTERM's indexed UART probe through the real P65C816 wrapper.
module tb_romterm_detect;
    reg clk = 0;
    always #5 clk = ~clk;

    reg res_n = 0;
    wire r_w_n;
    wire sync;
    wire bus_valid;
    wire [15:0] addr;
    wire [7:0] dout;
    wire [15:0] pc;
    reg [7:0] din;

    reg [7:0] mem [0:65535];
    reg [7:0] open_bus = 8'h00;
    integer i;

    wire dec_valid = bus_valid | ~r_w_n;
    wire serial0_cs = dec_valid && addr[15:3] == 13'h13fc;
    wire serial1_cs = dec_valid && addr[15:3] == 13'h13fd;
    wire [7:0] serial0_data;
    wire [7:0] serial1_data;
    wire serial0_dtr;
    wire serial1_dtr;

    p65c816_wrap cpu (
        .clk(clk), .enable(1'b1), .res_n(res_n), .irq_n(1'b1),
        .nmi_n(1'b1), .rdy(1'b1), .r_w_n(r_w_n), .sync(sync),
        .addr(addr), .din(din), .dout(dout), .pc(pc),
        .emu_mode(), .i_flag(), .vpb(), .bus_valid(bus_valid)
    );

    x16_serial_card uart0 (
        .clk(clk), .reset_n(res_n), .cs(serial0_cs), .rwn(r_w_n),
        .enable(1'b1), .addr(addr[2:0]), .di(dout), .do_o(serial0_data),
        .uart_rxd(1'b1), .uart_cts(1'b1), .uart_dsr(1'b1),
        .uart_ri(1'b0), .uart_txd(), .uart_rts(),
        .uart_dtr(serial0_dtr)
    );

    x16_serial_card uart1 (
        .clk(clk), .reset_n(res_n), .cs(serial1_cs), .rwn(r_w_n),
        .enable(1'b1), .addr(addr[2:0]), .di(dout), .do_o(serial1_data),
        .uart_rxd(1'b1), .uart_cts(1'b1), .uart_dsr(1'b1),
        .uart_ri(1'b0), .uart_txd(), .uart_rts(),
        .uart_dtr(serial1_dtr)
    );

    always @(*) begin
        if (serial0_cs)
            din = serial0_data;
        else if (serial1_cs)
            din = serial1_data;
        else if (addr[15:8] == 8'h9f)
            din = open_bus;
        else
            din = mem[addr];
    end

    always @(posedge clk) begin
        if (res_n)
            open_bus <= r_w_n ? din : dout;
        if (res_n && bus_valid && ~r_w_n && !serial0_cs && !serial1_cs)
            mem[addr] <= dout;
    end

    initial begin
        for (i = 0; i < 65536; i = i + 1)
            mem[i] = 8'hea;

        // Reset vector and a compact copy of ROMTERM's $9FE0 probe. ROMTERM
        // keeps the UART offset in X and accesses registers as $9Fxx,X.
        mem[16'hfffc] = 8'h00;
        mem[16'hfffd] = 8'h80;
        mem[16'h8000] = 8'h78;                         // SEI
        mem[16'h8001] = 8'hd8;                         // CLD
        mem[16'h8002] = 8'ha2; mem[16'h8003] = 8'h80; // LDX #$80
        mem[16'h8004] = 8'hbd; mem[16'h8005] = 8'h61; mem[16'h8006] = 8'h9f;
        mem[16'h8007] = 8'h8d; mem[16'h8008] = 8'h00; mem[16'h8009] = 8'h02;
        mem[16'h800a] = 8'hbd; mem[16'h800b] = 8'h63; mem[16'h800c] = 8'h9f;
        mem[16'h800d] = 8'h8d; mem[16'h800e] = 8'h01; mem[16'h800f] = 8'h02;
        mem[16'h8010] = 8'hbd; mem[16'h8011] = 8'h64; mem[16'h8012] = 8'h9f;
        mem[16'h8013] = 8'h8d; mem[16'h8014] = 8'h02; mem[16'h8015] = 8'h02;
        mem[16'h8016] = 8'ha9; mem[16'h8017] = 8'h69;
        mem[16'h8018] = 8'h9d; mem[16'h8019] = 8'h67; mem[16'h801a] = 8'h9f;
        mem[16'h801b] = 8'hbd; mem[16'h801c] = 8'h67; mem[16'h801d] = 8'h9f;
        mem[16'h801e] = 8'h8d; mem[16'h801f] = 8'h03; mem[16'h8020] = 8'h02;
        mem[16'h8021] = 8'ha9; mem[16'h8022] = 8'h01;
        mem[16'h8023] = 8'h9d; mem[16'h8024] = 8'h64; mem[16'h8025] = 8'h9f;
        mem[16'h8026] = 8'hbd; mem[16'h8027] = 8'h6e; mem[16'h8028] = 8'h9f;
        mem[16'h8029] = 8'h8d; mem[16'h802a] = 8'h04; mem[16'h802b] = 8'h02;
        mem[16'h802c] = 8'ha9; mem[16'h802d] = 8'h00;
        mem[16'h802e] = 8'h9d; mem[16'h802f] = 8'h64; mem[16'h8030] = 8'h9f;
        mem[16'h8031] = 8'hbd; mem[16'h8032] = 8'h6e; mem[16'h8033] = 8'h9f;
        mem[16'h8034] = 8'h8d; mem[16'h8035] = 8'h05; mem[16'h8036] = 8'h02;
        mem[16'h8037] = 8'ha9; mem[16'h8038] = 8'ha5;
        mem[16'h8039] = 8'h8d; mem[16'h803a] = 8'h0f; mem[16'h803b] = 8'h02;
        mem[16'h803c] = 8'h4c; mem[16'h803d] = 8'h40; mem[16'h803e] = 8'h80;

        // Probe UART1 at $9FE8. Its companion address $9FF6 is unmapped.
        mem[16'h8040] = 8'ha2; mem[16'h8041] = 8'h88; // LDX #$88
        mem[16'h8042] = 8'hbd; mem[16'h8043] = 8'h61; mem[16'h8044] = 8'h9f;
        mem[16'h8045] = 8'h8d; mem[16'h8046] = 8'h10; mem[16'h8047] = 8'h02;
        mem[16'h8048] = 8'hbd; mem[16'h8049] = 8'h63; mem[16'h804a] = 8'h9f;
        mem[16'h804b] = 8'h8d; mem[16'h804c] = 8'h11; mem[16'h804d] = 8'h02;
        mem[16'h804e] = 8'hbd; mem[16'h804f] = 8'h64; mem[16'h8050] = 8'h9f;
        mem[16'h8051] = 8'h8d; mem[16'h8052] = 8'h12; mem[16'h8053] = 8'h02;
        mem[16'h8054] = 8'ha9; mem[16'h8055] = 8'h69;
        mem[16'h8056] = 8'h9d; mem[16'h8057] = 8'h67; mem[16'h8058] = 8'h9f;
        mem[16'h8059] = 8'hbd; mem[16'h805a] = 8'h67; mem[16'h805b] = 8'h9f;
        mem[16'h805c] = 8'h8d; mem[16'h805d] = 8'h13; mem[16'h805e] = 8'h02;
        mem[16'h805f] = 8'ha9; mem[16'h8060] = 8'h01;
        mem[16'h8061] = 8'h9d; mem[16'h8062] = 8'h64; mem[16'h8063] = 8'h9f;
        mem[16'h8064] = 8'hbd; mem[16'h8065] = 8'h6e; mem[16'h8066] = 8'h9f;
        mem[16'h8067] = 8'h8d; mem[16'h8068] = 8'h14; mem[16'h8069] = 8'h02;
        mem[16'h806a] = 8'ha9; mem[16'h806b] = 8'h00;
        mem[16'h806c] = 8'h9d; mem[16'h806d] = 8'h64; mem[16'h806e] = 8'h9f;
        mem[16'h806f] = 8'hbd; mem[16'h8070] = 8'h6e; mem[16'h8071] = 8'h9f;
        mem[16'h8072] = 8'h8d; mem[16'h8073] = 8'h15; mem[16'h8074] = 8'h02;
        mem[16'h8075] = 8'ha9; mem[16'h8076] = 8'h5a;
        mem[16'h8077] = 8'h8d; mem[16'h8078] = 8'h1f; mem[16'h8079] = 8'h02;
        mem[16'h807a] = 8'h4c; mem[16'h807b] = 8'h7a; mem[16'h807c] = 8'h80;

        repeat (20) @(posedge clk);
        res_n = 1;

        wait (mem[16'h021f] == 8'h5a);
        $display("ROMTERM UART0: IER=%02x LCR=%02x MCR=%02x SCR=%02x MSR1=%02x MSR0=%02x",
                 mem[16'h0200], mem[16'h0201], mem[16'h0202],
                 mem[16'h0203], mem[16'h0204], mem[16'h0205]);
        $display("ROMTERM UART1: IER=%02x LCR=%02x MCR=%02x SCR=%02x MSR1=%02x MSR0=%02x",
                 mem[16'h0210], mem[16'h0211], mem[16'h0212],
                 mem[16'h0213], mem[16'h0214], mem[16'h0215]);
        if (mem[16'h0200] != 0 || mem[16'h0201] != 0 || mem[16'h0202] != 0 ||
            mem[16'h0203] != 8'h69 || mem[16'h0204][6] || mem[16'h0205][6] ||
            mem[16'h0210] != 0 || mem[16'h0211] != 0 || mem[16'h0212] != 0 ||
            mem[16'h0213] != 8'h69 || mem[16'h0214][6] || mem[16'h0215][6])
            $fatal(1, "FAIL: P65C816 ROMTERM probe mismatch");
        $display("PASS: both probes take ROMTERM's detected-port RI-low path");
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "TIMEOUT: pc=%04x addr=%04x valid=%b rwn=%b", pc, addr, bus_valid, r_w_n);
    end
endmodule
