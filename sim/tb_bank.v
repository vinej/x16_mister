`timescale 1ns/1ps
// ============================================================================
// tb_bank.v -- CPU-in-the-loop test of the 256-ROM-bank / cart feature.
//
// Real r65c02 core (8 MHz) + REAL rtl/ext_ram_sdram.sv (with the behavioral
// sdram_sim.v byte controller @100 MHz) + real lowram_bram + real
// ext_ram_bram, glued with a faithful replica of x16.sv's address decode,
// bank registers, SDRAM address mux and cpu_rdy network.  Runs banktest.s
// (see its header for the subtest list) and reports the LowRAM markers plus
// a direct look into the SDRAM model's cart region.
// ============================================================================
module tb_bank #(
    parameter integer CPU816 = 0,   // 0 = r65c02, 1 = P65C816
    parameter integer CPUMJ  = 0    // 1 = MJoergen 65c02 (overrides CPU816)
);
    // 8 MHz CPU clock, 100 MHz SDRAM clock (same 12.5 ratio as hardware)
    reg cpu_clk = 0;   always #62.5 cpu_clk   = ~cpu_clk;
    reg sdram_clk = 0; always #5    sdram_clk = ~sdram_clk;
    reg res_n     = 0;   // CPU reset (cpu_reset_n)
    reg mem_res_n = 0;   // memory-side reset (mem_reset_n: released BEFORE the
                         // CPU, stays up during the cart download, like x16.sv)

    wire        r_w_n, sync;
    wire [15:0] addr;
    wire  [7:0] dout;
    wire [15:0] pc;
    wire  [7:0] din;

    // ---- address decode (mirror of x16.sv) ----
    wire kernal_cs = (addr[15:14] == 2'b11);
    wire io_cs     = (addr[15:8]  == 8'h9F);
    wire hi_ram_cs = (addr[15:13] == 3'b101);
    wire lowram_cs = ~kernal_cs & ~io_cs & ~hi_ram_cs;

    // ---- ROM/RAM bank registers (mirror of x16.sv) ----
    reg [7:0] rom_bank_r = 8'h00;
    reg [7:0] ram_bank_r = 8'h00;
    always @(posedge cpu_clk or negedge res_n) begin
        if (!res_n) begin
            rom_bank_r <= 8'h00;
            ram_bank_r <= 8'h00;
        end else if (lowram_cs && ~r_w_n) begin
            if (addr == 16'h0001) rom_bank_r <= dout;
            if (addr == 16'h0000) ram_bank_r <= dout;
        end
    end

    wire vector_fetch_emu = r_w_n & (addr[15:3] == 13'b1111_1111_1111_1)
                                  & (addr[2:0]  != 3'b000)
                                  & (addr[2:0]  != 3'b001);
    wire vector_fetch_nat = r_w_n & (addr[15:4] == 12'hFFE)
                                  & (addr[3:2]  != 2'b00);
    wire vector_fetch     = vector_fetch_emu | vector_fetch_nat;
    wire [7:0] eff_rom_bank = vector_fetch ? 8'h00 : rom_bank_r;
    wire rom_open_sel = (eff_rom_bank[7:4] == 4'h1);
    wire cart_sel     = |eff_rom_bank[7:5];            // data-mux steer
    wire cart_cs      = kernal_cs & (|rom_bank_r[7:5]); // SDRAM cs (no vector term)

    // ---- BRAM system ROM, banks 0-15 (negedge read like rom_banks.sv) ----
    reg [7:0] rom [0:16383];            // bank 0 = banktest image; 1-15 = $EA
    initial $readmemh("banktest.hex", rom);
    reg [7:0] rom_data;
    always @(negedge cpu_clk)
        rom_data <= (eff_rom_bank[3:0] == 4'h0) ? rom[addr[13:0]] : 8'hEA;

    // ---- LowRAM (real module) ----
    wire [7:0] lowram_data;
    lowram_bram u_lowram (
        .clk(cpu_clk), .addr(addr), .cs(lowram_cs),
        .we(lowram_cs & ~r_w_n), .wr_data(dout), .rd_data(lowram_data)
    );

    // ---- hybrid HiRAM + cart window (mirror of x16.sv) ----
    localparam [7:0] BRAM_BANKS = 8'd2;
    wire       bram_sel = (ram_bank_r < BRAM_BANKS);
    wire [7:0] sdram_rd, bram_rd;
    wire       sdram_rdy, bram_rdy;
    wire [7:0] ext_ram_data = bram_sel ? bram_rd : sdram_rd;
    wire       hiram_we = hi_ram_cs & ~r_w_n;

    wire        ext_sdram_cs   = (hi_ram_cs & ~bram_sel) | cart_cs;
    wire        ext_sdram_we   = ext_sdram_cs & ~r_w_n;
    wire [24:0] ext_sdram_addr = cart_cs
                               ? {3'b001, rom_bank_r, addr[13:0]}
                               : {4'd0,   ram_bank_r, addr[12:0]};

    wire [12:0] SDRAM_A;
    wire [15:0] SDRAM_DQ;
    wire  [1:0] SDRAM_BA;
    wire SDRAM_nCS, SDRAM_nWE, SDRAM_nRAS, SDRAM_nCAS;
    wire SDRAM_CKE, SDRAM_CLK, SDRAM_DQML, SDRAM_DQMH;

    // loader port stimulus (models the hps_io ioctl stream, sdram_clk domain)
    reg        ld_wr = 0;
    reg [24:0] ld_addr = 0;
    reg  [7:0] ld_data = 0;
    wire       ld_busy;

    ext_ram_sdram u_ext (
        .clk(cpu_clk), .sdram_clk(sdram_clk), .reset_n(mem_res_n),
        .cs(ext_sdram_cs), .we(ext_sdram_we),
        .byte_addr(ext_sdram_addr), .wr_data(dout),
        .rd_data(sdram_rd), .ready(sdram_rdy),
        .ld_wr(ld_wr), .ld_addr(ld_addr), .ld_data(ld_data), .ld_busy(ld_busy),
        .SDRAM_A(SDRAM_A), .SDRAM_DQ(SDRAM_DQ), .SDRAM_BA(SDRAM_BA),
        .SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE),
        .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_CKE(SDRAM_CKE), .SDRAM_CLK(SDRAM_CLK),
        .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH)
    );

    ext_ram_bram u_bram (
        .clk(cpu_clk), .cs(hi_ram_cs & bram_sel), .we(hiram_we),
        .bank(ram_bank_r), .addr(addr[12:0]), .wr_data(dout),
        .rd_data(bram_rd), .ready(bram_rdy)
    );

    wire cpu_rdy = bram_rdy & sdram_rdy;

    // ---- CPU data-in mux (mirror of x16.sv) ----
    assign din = kernal_cs ? (cart_sel     ? sdram_rd :
                              rom_open_sel ? 8'hFF    : rom_data) :
                 hi_ram_cs ? ext_ram_data :
                 lowram_cs ? lowram_data  : 8'h00;

    // ---- CPU (r65c02 or P65C816, same wrapper port shape) ----
    generate if (CPUMJ != 0) begin : g_cpumj
        mj65c02_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(1'b1), .nmi_n(1'b1), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(din), .dout(dout), .pc(pc),
            .bus_valid()
        );
    end else if (CPU816 != 0) begin : g_cpu816
        p65c816_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(1'b1), .nmi_n(1'b1), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(din), .dout(dout), .pc(pc),
            .emu_mode(), .i_flag()
        );
    end else begin : g_cpu02
        r65c02_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(1'b1), .nmi_n(1'b1), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(din), .dout(dout), .pc(pc)
        );
    end endgenerate

    // zero the BRAMs like configured hardware (avoid X-prop artifacts)
    integer i;
    initial begin
        for (i = 0; i < 40704; i = i + 1) u_lowram.mem[i] = 8'h00;
        for (i = 0; i < 16384; i = i + 1) u_bram.mem[i]   = 8'h00;
    end

    // cart byte address helper: 0x400000 + bank*16K + offset
    function [24:0] cart_a(input [7:0] bank, input [13:0] off);
        cart_a = 25'h400000 + {bank, 14'd0} + {11'd0, off};
    endfunction

    // push one loader byte, honoring ld_busy exactly like ioctl_wait
    task ld_push(input [24:0] a, input [7:0] d);
        begin
            while (ld_busy) @(posedge sdram_clk);
            @(posedge sdram_clk);
            ld_wr <= 1; ld_addr <= a; ld_data <= d;
            @(posedge sdram_clk);
            ld_wr <= 0;
        end
    endtask

    integer k;
    initial begin
        // memory side comes up first (mem_reset_n excludes the download hold)
        repeat (4) @(posedge cpu_clk);
        mem_res_n = 1;
        // let the fast-domain reset synchronizer release before streaming (on
        // hardware a download starts seconds after reset, never within 20 ns)
        repeat (8) @(posedge sdram_clk);

        // "cart download": CPU still in reset.  Push the CX16 signature +
        // patterns into bank 32 -- back-to-back, so ld_busy/ioctl_wait and the
        // S_INIT window are exercised too.
        ld_push(25'h480000 + 0, 8'h43);          // 'C'
        ld_push(25'h480000 + 1, 8'h58);          // 'X'
        ld_push(25'h480000 + 2, 8'h31);          // '1'
        ld_push(25'h480000 + 3, 8'h36);          // '6'
        for (k = 4; k < 64; k = k + 1)
            ld_push(25'h480000 + k, k[7:0] ^ 8'hA5);
        ld_push(25'h480000 + 25'h3EFF, 8'h99);   // bank 32, CPU $FEFF

        // download done -> release the CPU (like sys_rst_n after ioctl)
        repeat (20) @(posedge cpu_clk);
        $display("[LD  ] after stream: state=%0d ldf_cnt=%0d mem[480000..3]=%02x %02x %02x %02x mem[483EFF]=%02x",
                 u_ext.state, u_ext.ldf_cnt,
                 u_ext.u_sdram.mem[25'h480000], u_ext.u_sdram.mem[25'h480001],
                 u_ext.u_sdram.mem[25'h480002], u_ext.u_sdram.mem[25'h480003],
                 u_ext.u_sdram.mem[25'h483EFF]);
        res_n = 1;
        fork
            begin : t
                while (u_lowram.mem[2] !== 8'hAA && u_lowram.mem[2] !== 8'hEE)
                    @(posedge cpu_clk);
                if (u_lowram.mem[2] === 8'hAA)
                    $display("[TBC] *** BANK TEST: ALL PASS ***");
                else begin
                    $display("[TBC] *** BANK TEST FAILED: code=%0d actual=%02x ***",
                             u_lowram.mem[3], u_lowram.mem[4]);
                    $display("[TBC] t8 diag: wrapbank=%0d exit=%0d expect=%02x reread=%02x xoff=%0d",
                             u_lowram.mem[5], u_lowram.mem[6],
                             u_lowram.mem[7], u_lowram.mem[8], u_lowram.mem[9]);
                end
                $display("[MEM] b40 C000=%02x(5A)  b40 FFF0=%02x(66)  b41 C000=%02x(C3)  b40 C200=%02x(5C)  b40 C100=%02x(A9)",
                         u_ext.u_sdram.mem[cart_a(8'd40, 14'h0000)],
                         u_ext.u_sdram.mem[cart_a(8'd40, 14'h3FF0)],
                         u_ext.u_sdram.mem[cart_a(8'd41, 14'h0000)],
                         u_ext.u_sdram.mem[cart_a(8'd40, 14'h0200)],
                         u_ext.u_sdram.mem[cart_a(8'd40, 14'h0100)]);
                $display("[MEM] hiram b2 A000=%02x(11)  b2 BFFF=%02x(77)  b3 A000=%02x(22)",
                         u_ext.u_sdram.mem[25'd2*8192 + 0],
                         u_ext.u_sdram.mem[25'd2*8192 + 13'h1FFF],
                         u_ext.u_sdram.mem[25'd3*8192 + 0]);
                disable w;
            end
            begin : w
                #80000000;
                $display("[TBC] TIMEOUT pc=%04x addr=%04x done=%02x code=%0d",
                         pc, addr, u_lowram.mem[2], u_lowram.mem[3]);
                disable t;
            end
        join
        $finish;
    end
endmodule
