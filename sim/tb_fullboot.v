`timescale 1ns/1ps
// ============================================================================
// tb_fullboot.v -- boot the REAL R49 ROM (rom/rom.hex, all 16 banks) on the
// real core RTL: r65c02 + rom_banks + lowram + hybrid HiRAM (ext_ram_bram +
// ext_ram_sdram/sdram_sim) + via1/via2 + smc_x16 + rtc_x16, with vera_stub
// standing in for VERA (registers + VSYNC IRQ + "no SD card" SPI).
//
// Decode/banking/vector logic mirrors x16.sv exactly.  After the boot
// banner settles (VERA write activity goes quiet), the TB types
//     ?DA$ <return>
// through the SMC keyboard path (PS/2 set-2 make/break codes) -- the same
// call chain as PRINT DATE$.
//
// Instrumentation:
//   * BRK trap: opcode fetch of $00 -> dump PC, banks, SP and the last 32
//     opcode-fetch PCs (the wild-jump trail) -- the "crash to monitor".
//   * progress heartbeat with PC / IRQ count / VERA write count.
// ============================================================================
module tb_fullboot #(
    // 1 = single-cycle behavioral HiRAM for banks 2-255 (10-20x faster sim;
    //     the SDRAM handshake is separately proven by the bank suite).
    // 0 = the real ext_ram_sdram + 100 MHz behavioral controller.
    parameter integer FASTRAM = 1,
    // 0 = r65c02 (shipping core), 1 = P65C816 (65C816 branch)
    parameter integer CPU816  = 0,
    parameter integer CPUMJ   = 0,   // MJoergen 65c02 (overrides CPU816)
    // 1 = stream cart.hex (boot2.rom) into cart bank 32 before reset release
    //     (MiSTer auto-load replica; the boot cart probe will FIND and RUN
    //     it).  Requires FASTRAM=0.
    parameter integer CARTLOAD = 0
);
    reg cpu_clk = 0;   always #62.5 cpu_clk = ~cpu_clk;
    // 100 MHz clock only exists in full-SDRAM mode: in FASTRAM mode it never
    // toggles, cutting simulator event count by an order of magnitude
    reg sdram_clk = 0;
    initial if (FASTRAM == 0) forever #5 sdram_clk = ~sdram_clk;
    reg res_n = 0;

    wire        r_w_n, sync;
    wire [15:0] addr;
    wire  [7:0] dout;
    wire [15:0] pc;
    wire  [7:0] din;
    wire        cpu_rdy;          // forward-decl (assigned below)

    // ---------------- decode (x16.sv replica) ----------------
    // qualified with bus_valid (VDA|VPA) like x16.sv: no ghost accesses on
    // the '816's internal cycles (r65c02: constant 1)
    // write cycles exempt from the VDA|VPA gate, like x16.sv (dec_valid)
    wire cpu_bus_valid;
    wire dec_valid = cpu_bus_valid | ~r_w_n;
    wire kernal_cs = dec_valid & (addr[15:14] == 2'b11);
    wire io_cs     = dec_valid & (addr[15:8]  == 8'h9F);
    wire vera_cs   = dec_valid & (addr[15:5]  == 11'b10011111001);
    wire ym_cs     = dec_valid & (addr[15:4]  == 12'h9F4);
    wire via1_cs   = dec_valid & (addr[15:4]  == 12'h9F0);
    wire via2_cs   = dec_valid & (addr[15:4]  == 12'h9F1);
    wire hi_ram_cs = dec_valid & (addr[15:13] == 3'b101);
    wire lowram_cs = dec_valid & ~(addr[15:14] == 2'b11)
                   & ~(addr[15:8] == 8'h9F) & ~(addr[15:13] == 3'b101);

    // NOTE: NO cpu_rdy gate -- x16.sv captures bank writes ungated (the
    // r65c02 blows through rdy on writes; '816 held writes repeat the same
    // value = idempotent).  An earlier TB version gated this with cpu_rdy,
    // which LOSES r65c02 writes during wf_hi stalls -- a TB-only artifact.
    //
    // VPB (X16 R44+ platform requirement, x16-rom README): an interrupt
    // vector pull SETS the hardware ROM bank latch to 0; the lowram shadow
    // at $0001 keeps the old value for the handler to save.  Without this
    // the c816 KERNAL RAM stub ($038B: JMP $F80A = bank-0 ROM) executes
    // from whatever bank is live -> BANNEX $AA filler -> the splash freeze.
    // The r65c02 has no VPB pin (its stub does stz rom_bank itself): tied 1.
    wire cpu_vpb;
    reg [7:0] rom_bank_r = 0, ram_bank_r = 0;
    always @(posedge cpu_clk or negedge res_n) begin
        if (!res_n) begin rom_bank_r <= 0; ram_bank_r <= 0; end
        else if (!cpu_vpb) rom_bank_r <= 8'h00;   // vector pull -> KERNAL bank
        else if (lowram_cs && ~r_w_n) begin
            if (addr == 16'h0001) rom_bank_r <= dout;
            if (addr == 16'h0000) ram_bank_r <= dout;
        end
    end

    // vector pulls -> bank 0 (x16.sv replica): '816 by VPB pin only (an
    // address match on $FFE4-$FFEF would also catch program fetches of the
    // KERNAL API entries GETIN/CLALL/UDTIM/SCREEN from other banks);
    // r65c02 by the proven address match on the emulation vectors.
    wire vector_fetch = (CPU816 != 0) ? ~cpu_vpb
                      : (r_w_n & (addr[15:3] == 13'b1111_1111_1111_1)
                               & (addr[2:0] != 3'b000) & (addr[2:0] != 3'b001));
    wire [7:0] eff_rom_bank = vector_fetch ? 8'h00 : rom_bank_r;
    wire rom_open_sel = (eff_rom_bank[7:4] == 4'h1);
    wire cart_sel     = |eff_rom_bank[7:5];
    wire cart_cs      = kernal_cs & (|rom_bank_r[7:5]);

    // ---------------- ROM: the real rom_banks + real rom.hex --------------
    wire [7:0] rom_data;
    rom_banks u_rom (
        .clk(cpu_clk), .bank(eff_rom_bank[3:0]), .addr(addr[13:0]),
        .rd_data(rom_data),
        .wr_clk(sdram_clk), .wr_en(1'b0), .wr_addr(18'd0), .wr_data(8'd0)
    );

    // ---------------- LowRAM ----------------
    wire [7:0] lowram_data;
    lowram_bram u_lowram (
        .clk(cpu_clk), .addr(addr), .cs(lowram_cs),
        .we(lowram_cs & ~r_w_n), .wr_data(dout), .rd_data(lowram_data)
    );

    // ---------------- hybrid HiRAM + cart (x16.sv replica) ----------------
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

    // ---- cart loader port (x16.sv ioctl replica; used when CARTLOAD=1) ----
    // mem_rst_n releases the SDRAM controller early so the loader can stream
    // boot2.rom into cart bank 32 while the CPU is still in reset -- exactly
    // the MiSTer auto-load sequence.  The KERNAL's end-of-boot cart probe
    // then FINDS the CX16 signature and jumps into the cart, like on HW.
    reg         mem_rst_n = 0;
    reg         ld_wr = 0;
    reg  [24:0] ld_addr = 25'd0;
    reg  [7:0]  ld_data = 8'd0;
    wire        ld_busy;

    generate if (FASTRAM != 0) begin : g_fastram
        // single-cycle stand-in for the SDRAM client: same 8 MB byte space
        // (HiRAM 0x000000+, cart 0x480000+), ext_ram_bram-style handshake
        reg [7:0] fmem [0:8388607];
        reg [7:0] frd;
        reg       fdone;
        always @(posedge cpu_clk) begin
            if (ext_sdram_cs & ext_sdram_we) fmem[ext_sdram_addr[22:0]] <= dout;
            frd   <= fmem[ext_sdram_addr[22:0]];
            fdone <= ext_sdram_cs & ~ext_sdram_we & ~fdone;
        end
        assign sdram_rd  = frd;
        assign sdram_rdy = ~ext_sdram_cs | ext_sdram_we | fdone;
        assign ld_busy   = 1'b0;    // loader unused in FASTRAM mode
    end else begin : g_sdram
        // memory gets its own reset so the CART LOADER can stream while the
        // CPU is still held in reset (x16.sv's split-reset download scheme)
        ext_ram_sdram u_ext (
            .clk(cpu_clk), .sdram_clk(sdram_clk), .reset_n(mem_rst_n),
            .cs(ext_sdram_cs), .we(ext_sdram_we),
            .byte_addr(ext_sdram_addr), .wr_data(dout),
            .rd_data(sdram_rd), .ready(sdram_rdy),
            .ld_wr(ld_wr), .ld_addr(ld_addr), .ld_data(ld_data),
            .ld_busy(ld_busy),
            .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
            .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
            .SDRAM_DQML(), .SDRAM_DQMH()
        );

        // SHADOW MIRROR: an independent copy of every committed SDRAM write
        // (same commit condition as the RTL's FIFO push), compared against
        // every delivered read.  The first mismatch = the exact transaction
        // where the controller returns wrong data.  $DD = sdram_sim's init.
        reg [7:0] shadow [0:8388607];
        integer sh_i, shad_n = 0;
        initial for (sh_i = 0; sh_i < 8388608; sh_i = sh_i + 1)
            shadow[sh_i] = 8'hDD;
        // loader bytes land in cart space too: mirror them
        always @(posedge sdram_clk) if (ld_wr) shadow[ld_addr[22:0]] <= ld_data;
        always @(posedge cpu_clk) begin
            if (u_ext.wpush) shadow[ext_sdram_addr[22:0]] <= dout;
            // delivery cycle: CPU consumes rd_data (sdram_rdy is the only
            // low term of cpu_rdy during an SDRAM read in this TB)
            if (ext_sdram_cs && ~ext_sdram_we && sdram_rdy
                && sdram_rd !== shadow[ext_sdram_addr[22:0]]) begin
                shad_n = shad_n + 1;
                $display("[SHAD] t=%0t READ MISMATCH #%0d @%06x: got %02x want %02x (pc=%04x rom=%02x ram=%02x)",
                         $time, shad_n, ext_sdram_addr, sdram_rd,
                         shadow[ext_sdram_addr[22:0]], pc, rom_bank_r,
                         ram_bank_r);
                if (shad_n >= 30) begin
                    $display("[SHAD] 30 mismatches -- stopping");
                    $finish;
                end
            end
        end
    end endgenerate

    ext_ram_bram u_bram (
        .clk(cpu_clk), .cs(hi_ram_cs & bram_sel), .we(hiram_we),
        .bank(ram_bank_r), .addr(addr[12:0]), .wr_data(dout),
        .rd_data(bram_rd), .ready(bram_rdy)
    );

    // ---------------- VIA1 + I2C bus + SMC + RTC (x16.sv replica) ---------
    wire [7:0] via1_data, via2_data;
    wire       via1_irq_n, via2_irq_n;
    wire [7:0] via1_pa_in, via1_pa_out, via1_pa_oe;
    wire [7:0] via1_pb_in, via1_pb_out, via1_pb_oe;
    wire [7:0] via2_pa_in, via2_pa_out, via2_pa_oe;
    wire [7:0] via2_pb_in, via2_pb_out, via2_pb_oe;

    wire smc_sda_drv_low, rtc_sda_drv_low;
    wire via_sda_drv_low = via1_pa_oe[0] & ~via1_pa_out[0];
    wire via_scl_drv_low = via1_pa_oe[1] & ~via1_pa_out[1];
    wire bus_sda = ~(via_sda_drv_low | smc_sda_drv_low | rtc_sda_drv_low);
    wire bus_scl = ~via_scl_drv_low;

    assign via1_pa_in[0]   = bus_sda;
    assign via1_pa_in[1]   = bus_scl;
    assign via1_pa_in[3:2] = via1_pa_out[3:2] | ~via1_pa_oe[3:2];
    assign via1_pa_in[7:4] = 4'b1111;          // no SNES pads: absent
    assign via1_pb_in      = via1_pb_out | ~via1_pb_oe;
    assign via2_pa_in      = via2_pa_out | ~via2_pa_oe;
    assign via2_pb_in      = via2_pb_out | ~via2_pb_oe;

    // VERA read stall, VERBATIM from x16.sv: every committed VERA read holds
    // cpu_rdy low for 2 cycles.  vera_stub answers same-cycle so this is not
    // needed for data correctness -- it is here to exercise the CPU's
    // enable-stall behavior on IO reads exactly as on HW (the VSYNC handler
    // reads VERA ISR every IRQ).  The '816 treats enable as a global clock
    // gate; this interaction existed on HW only until now.
    wire vera_read = vera_cs & r_w_n;
    reg [1:0] vera_read_stall = 2'h0;
    always @(posedge cpu_clk) begin
        if (!res_n)               vera_read_stall <= 2'h0;
        else if (vera_read) begin
            if (vera_read_stall != 2'd3) vera_read_stall <= vera_read_stall + 2'd1;
        end else                  vera_read_stall <= 2'h0;
    end
    assign cpu_rdy = (~vera_read | (vera_read_stall >= 2'd2)) & bram_rdy & sdram_rdy;

    via65c22 u_via1 (
        .clk(cpu_clk), .reset_n(res_n), .cs(via1_cs), .rwn(r_w_n),
        .enable(cpu_rdy), .addr(addr[3:0]), .di(dout), .do_o(via1_data),
        .pa_in(via1_pa_in), .pa_out(via1_pa_out), .pa_oe(via1_pa_oe),
        .pb_in(via1_pb_in), .pb_out(via1_pb_out), .pb_oe(via1_pb_oe),
        .ca1_in(1'b0), .ca2_in(1'b0), .cb1_in(1'b0), .cb2_in(1'b0),
        .irq_n(via1_irq_n)
    );
    via65c22 u_via2 (
        .clk(cpu_clk), .reset_n(res_n), .cs(via2_cs), .rwn(r_w_n),
        .enable(cpu_rdy), .addr(addr[3:0]), .di(dout), .do_o(via2_data),
        .pa_in(via2_pa_in), .pa_out(via2_pa_out), .pa_oe(via2_pa_oe),
        .pb_in(via2_pb_in), .pb_out(via2_pb_out), .pb_oe(via2_pb_oe),
        .ca1_in(1'b0), .ca2_in(1'b0), .cb1_in(1'b0), .cb2_in(1'b0),
        .irq_n(via2_irq_n)
    );

    reg  [7:0] kbd_byte  = 8'h00;
    reg        kbd_valid = 1'b0;
    smc_x16 u_smc (
        .clk(cpu_clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(smc_sda_drv_low),
        .uart_byte(kbd_byte), .uart_byte_valid(kbd_valid)
    );

    reg [64:0] hps_rtc = 65'd0;
    rtc_x16 u_rtc (
        .clk(cpu_clk), .reset_n(res_n),
        .sda_bus(bus_sda), .scl_bus(bus_scl), .sda_drive_low(rtc_sda_drv_low),
        .hps_rtc(hps_rtc)
    );

    // ---------------- VERA stub + YM stub ----------------
    wire [7:0] vera_data;
    wire       vera_irq_n;
    vera_stub u_vera (
        .clk(cpu_clk), .reset_n(res_n),
        .cs(vera_cs), .we(vera_cs & ~r_w_n),   // ungated, like x16.sv's pipeline
        .addr(addr[4:0]), .wr_data(dout),
        .rd_data(vera_data), .irq_n(vera_irq_n)
    );
    wire [7:0] ym_data = 8'h00;                // YM2151 status: never busy

    // ---------------- IRQ net + CPU ----------------
    wire cpu_irq_n = vera_irq_n & via1_irq_n & via2_irq_n;

    assign din = kernal_cs ? (cart_sel ? sdram_rd :
                              rom_open_sel ? 8'hFF : rom_data) :
                 hi_ram_cs ? ext_ram_data :
                 vera_cs   ? vera_data :
                 ym_cs     ? ym_data :
                 via1_cs   ? via1_data :
                 via2_cs   ? via2_data :
                 lowram_cs ? lowram_data : 8'h00;

    generate if (CPUMJ != 0) begin : g_cpumj
        mj65c02_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(cpu_irq_n), .nmi_n(1'b1), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(din), .dout(dout), .pc(pc),
            .bus_valid(cpu_bus_valid)
        );
        assign cpu_vpb = 1'b1;         // 65C02: vector_fetch by address match
    end else if (CPU816 != 0) begin : g_cpu816
        p65c816_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(cpu_irq_n), .nmi_n(1'b1), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(din), .dout(dout), .pc(pc),
            .emu_mode(), .i_flag(), .vpb(cpu_vpb), .bus_valid(cpu_bus_valid)
        );
    end else begin : g_cpu02
        r65c02_wrap u_cpu (
            .clk(cpu_clk), .enable(cpu_rdy), .res_n(res_n),
            .irq_n(cpu_irq_n), .nmi_n(1'b1), .rdy(1'b1),
            .r_w_n(r_w_n), .sync(sync), .addr(addr),
            .din(din), .dout(dout), .pc(pc)
        );
        assign cpu_vpb       = 1'b1;   // no VPB pin; its RAM stub sets bank 0
        assign cpu_bus_valid = 1'b1;   // 65C02: every cycle is a real bus cycle
    end endgenerate

    integer i;
    initial begin
        for (i = 0; i < 40704; i = i + 1) u_lowram.mem[i] = 8'h00;
        for (i = 0; i < 16384; i = i + 1) u_bram.mem[i]   = 8'h00;
    end

    // ---------------- instrumentation ----------------
    // opcode-fetch ring buffer + BRK trap
    reg [15:0] pchist [0:31];
    reg [7:0]  bankhist [0:31];
    integer    ph = 0;
    integer    brkn = 0;
    integer    vera_wr = 0, irqn = 0;
    integer    dbg_n = 0;      // early-life cycle counter (probe below)
    reg        irq_was = 1'b1;

    always @(posedge cpu_clk) if (res_n) begin
        if (sync && cpu_rdy) begin
            pchist[ph & 31]   <= addr;
            bankhist[ph & 31] <= rom_bank_r;
            ph <= ph + 1;
            // (r65c02 sync over-asserts on operand cycles, so din==0 here is
            //  NOT a reliable BRK detector -- crash detection is done below
            //  via the MONITOR bank instead)
        end
        if (vera_cs && ~r_w_n && cpu_rdy) vera_wr = vera_wr + 1;
        irq_was <= cpu_irq_n;
        if (irq_was && !cpu_irq_n) irqn = irqn + 1;

        // CRASH DETECTOR: ROM bank 5 = the MONITOR bank.  Nothing selects it
        // during normal boot/BASIC; entering it = "crashed to monitor".
        if (rom_bank_r == 8'h05 && brkn == 0) begin : mon
            integer k;
            brkn = 1;
            $display("[MON ] t=%0t *** MONITOR BANK SELECTED (crash!) *** pc=%04x ram_bank=%02x",
                     $time, pc, ram_bank_r);
            $write("[MON ] fetch trail (bank:pc):");
            for (k = 31; k >= 0; k = k - 1)
                $write(" %02x:%04x", bankhist[(ph - k) & 31], pchist[(ph - k) & 31]);
            $write("\n");
            // keep running a little to observe, then stop
            repeat (200000) @(posedge cpu_clk);
            $display("[MON ] post-crash pc=%04x rom=%02x", pc, rom_bank_r);
            $finish;
        end
    end

    // ---------------- full bus-trace ring + garbage-bank trigger -----------
    // 2048-cycle rolling bus trace, dumped the moment a garbage ROM bank
    // (>= $10: open/cart space, no cart loaded) is WRITTEN to $0001 -- this
    // catches the corrupt jsrfar in the act, including the (imparm),y
    // argument reads that produced the bad bank byte.
    reg [15:0] tr_addr [0:2047];
    reg  [7:0] tr_din  [0:2047];
    reg  [7:0] tr_dout [0:2047];
    reg        tr_rwn  [0:2047];
    reg        tr_sync [0:2047];
    reg        tr_rdy  [0:2047];
    integer    trp = 0;
    reg        trg_done = 0;
    time       last_fall_t = 0;    // last cpu_irq_n falling edge

    always @(posedge cpu_clk) if (res_n && !trg_done) begin
        tr_addr[trp & 2047] <= addr;
        tr_din [trp & 2047] <= din;
        tr_dout[trp & 2047] <= dout;
        tr_rwn [trp & 2047] <= r_w_n;
        tr_sync[trp & 2047] <= sync;
        tr_rdy [trp & 2047] <= cpu_rdy;
        trp <= trp + 1;
        // NOTE: an earlier trigger here ("rom bank write >= $10 = garbage")
        // was a FALSE POSITIVE: the KERNAL's end-of-boot cartridge probe
        // legitimately banks in ROM banks 32+ to look for the CX16 signature
        // (see $F6EB: ptr=$C000, X=$20, JSR $FA63 far-fetch helper).
        //
        // REAL-anomaly trap: opcode fetch inside BANNEX's $AA-filler region
        // ($D5AF-$FA7F in ROM bank $0C -- no legit code there).  In the sick
        // '816 run the PC lands here at t~600s and then wanders forever
        // (filler sweep -> jsrfar copy with garbage stack -> bank $7F ...).
        // The ring then holds the 2048 cycles ENDING with the bad jump.
        if (sync && cpu_rdy && kernal_cs && rom_bank_r == 8'h0C
            && addr >= 16'hD600 && addr <= 16'hFA7F) begin : filler
            integer k;
            trg_done = 1;
            $display("[FILL] t=%0t fetch @%04x in BANNEX FILLER (ram=%02x irqs=%0d verawr=%0d)",
                     $time, addr, ram_bank_r, irqn, vera_wr);
            $display("[FILL] last 2048 bus cycles:");
            for (k = 2047; k >= 0; k = k - 1)
                if (trp - k >= 0)
                    $display("[TR  ] %0d a=%04x din=%02x do=%02x rwn=%b sy=%b rdy=%b",
                             trp - k, tr_addr[(trp - k) & 2047],
                             tr_din[(trp - k) & 2047], tr_dout[(trp - k) & 2047],
                             tr_rwn[(trp - k) & 2047], tr_sync[(trp - k) & 2047],
                             tr_rdy[(trp - k) & 2047]);
            $finish;
        end
    end

    // ---------------- wild-bank hunt (the FASTRAM=0 '816 wedge) ------------
    // ring of the last 256 writes to $0000/$0001: who set the bank, from where
    reg [15:0] bw_pc  [0:255];
    reg  [7:0] bw_val [0:255];
    reg        bw_rom [0:255];       // 1 = $0001 (rom bank), 0 = $0000 (ram)
    time       bw_t   [0:255];
    integer    bwp = 0;
    integer    wildcnt = 0;   // consecutive fetches in bank >= 16

    always @(posedge cpu_clk) if (res_n) begin
        if (lowram_cs && ~r_w_n && cpu_rdy
            && (addr == 16'h0000 || addr == 16'h0001)) begin
            bw_pc [bwp & 255] <= pc;
            bw_val[bwp & 255] <= dout;
            bw_rom[bwp & 255] <= addr[0];
            bw_t  [bwp & 255] <= $time;
            bwp <= bwp + 1;
        end
        // TRAP: SUSTAINED opcode fetching from ROM bank >= 16 (cart space, no
        // cart in this TB) = the CPU flew into the weeds.  Requires 8
        // CONSECUTIVE such fetches: the KERNAL's end-of-boot cart probe does
        // isolated reads there (and the r65c02's over-asserted sync makes
        // those look like fetches) -- a single hit is a false positive.
        if (sync && cpu_rdy) begin
            if (kernal_cs && rom_bank_r >= 8'd16) wildcnt = wildcnt + 1;
            else                                  wildcnt = 0;
        end
        if (wildcnt >= 8 && brkn == 0 && CARTLOAD == 0)   // cart runs are legit
        begin : wild
            integer k;
            brkn = 1;
            $display("[WILD] t=%0t fetch @%04x in ROM bank %02x (ram=%02x irqs=%0d verawr=%0d)",
                     $time, addr, rom_bank_r, ram_bank_r, irqn, vera_wr);
            $write("[WILD] fetch trail (bank:pc):");
            for (k = 31; k >= 0; k = k - 1)
                $write(" %02x:%04x", bankhist[(ph - k) & 31], pchist[(ph - k) & 31]);
            $write("\n");
            $write("[WILD] last bank writes (t/us pc reg<=val):");
            for (k = 255; k >= 0; k = k - 1)
                if (bwp - k > 0)
                    $write(" [%0d %04x %s<=%02x]",
                           bw_t[(bwp - k) & 255] / 1000000, bw_pc[(bwp - k) & 255],
                           bw_rom[(bwp - k) & 255] ? "rom" : "ram",
                           bw_val[(bwp - k) & 255]);
            $write("\n");
            $finish;
        end
    end

    // early-life probe: full trace of the first 300 cycles after reset
    always @(posedge cpu_clk) if (res_n && dbg_n < 300) begin
        dbg_n = dbg_n + 1;
        $display("[DBG ] %0d a=%04x d=%02x rwn=%b sync=%b rdy=%b(br=%b sd=%b) irq=%b rom=%02x",
                 dbg_n, addr, din, r_w_n, sync, cpu_rdy, bram_rdy, sdram_rdy,
                 cpu_irq_n, rom_data);
    end

    // progress heartbeat
    initial forever begin
        #25000000;   // every 200k cpu cycles
        $display("[HB  ] t=%0tms pc=%04x rom=%02x ram=%02x irqs=%0d verawr=%0d irqn=%b isr=%02x ien=%02x sda=%b scl=%b",
                 $time / 1000000, pc, rom_bank_r, ram_bank_r, irqn, vera_wr,
                 cpu_irq_n, u_vera.isr, u_vera.regs[6], bus_sda, bus_scl);
    end

    // ---------------- keyboard injection: ?DA$ <CR> ----------------
    task key(input [7:0] code); begin
        kbd_byte <= code; kbd_valid <= 1'b1; @(posedge cpu_clk);
        kbd_valid <= 1'b0;
        repeat (160000) @(posedge cpu_clk);    // ~20ms: let kbd_scan drain
    end endtask
    task keyrel(input [7:0] code); begin
        kbd_byte <= 8'hF0; kbd_valid <= 1'b1; @(posedge cpu_clk);
        kbd_valid <= 1'b0; repeat (8000) @(posedge cpu_clk);
        key(code);
    end endtask

    // CARTLOAD=1: stream cart.hex (boot2.rom) into cart bank 32 via the
    // loader port before CPU reset release -- the MiSTer auto-load replica.
    // Requires FASTRAM=0 (the loader lives in ext_ram_sdram).
    reg [7:0] cart_img [0:16383];
    integer settle, ci;
    initial begin
        repeat (20) @(posedge cpu_clk);
        mem_rst_n = 1;                       // memory out of reset first
        if (CARTLOAD != 0) begin
            $readmemh("cart.hex", cart_img);
            repeat (600) @(posedge sdram_clk);   // SDRAM init
            for (ci = 0; ci < 16384; ci = ci + 1) begin
                while (ld_busy) @(posedge sdram_clk);
                @(posedge sdram_clk);
                // cart bank 32 base = 0x400000 + 32*16K = 0x480000
                ld_wr <= 1; ld_addr <= 25'h480000 + ci; ld_data <= cart_img[ci];
                @(posedge sdram_clk);
                ld_wr <= 0;
            end
            repeat (200) @(posedge sdram_clk);   // drain loader FIFO
            $display("[CART] boot2.rom streamed into bank 32 (%02x %02x %02x %02x @C000)",
                     cart_img[0], cart_img[1], cart_img[2], cart_img[3]);
        end
        repeat (20) @(posedge cpu_clk);
        res_n = 1;
        repeat (20) @(posedge cpu_clk);
        hps_rtc[63:0] <= {8'h40, 5'd0, 3'd3, 8'h26, 8'h07, 8'h05, 8'h21, 8'h58, 8'h40};
        hps_rtc[64]   <= ~hps_rtc[64];

        // wait for boot: VERA writes plateau (banner done) after >=2000 writes
        settle = 0;
        begin : bootwait
            integer last;
            last = -1;
            forever begin
                repeat (200000) @(posedge cpu_clk);
                if (vera_wr == last && vera_wr >= 2000) begin
                    settle = settle + 1;
                    if (settle >= 3) disable bootwait;
                end else settle = 0;
                last = vera_wr;
            end
        end
        $display("[KEY ] t=%0t boot settled (verawr=%0d irqs=%0d) -- typing ?DA$<CR>", $time, vera_wr, irqn);

        key(8'h12);                    // shift make
        key(8'h4A);  keyrel(8'h4A);    // '/' -> '?'
        kbd_byte <= 8'hF0; kbd_valid <= 1'b1; @(posedge cpu_clk); kbd_valid <= 1'b0;
        repeat (8000) @(posedge cpu_clk);
        key(8'h12);                    // shift release (F0 12)
        key(8'h23);  keyrel(8'h23);    // D
        key(8'h1C);  keyrel(8'h1C);    // A
        key(8'h12);                    // shift make
        key(8'h25);  keyrel(8'h25);    // '4' -> '$'
        kbd_byte <= 8'hF0; kbd_valid <= 1'b1; @(posedge cpu_clk); kbd_valid <= 1'b0;
        repeat (8000) @(posedge cpu_clk);
        key(8'h12);                    // shift release
        key(8'h5A);  keyrel(8'h5A);    // ENTER -> executes ?DA$

        // give it plenty of time to run DATE$ (and crash, if it will)
        repeat (8000000) @(posedge cpu_clk);
        $display("[TBC ] done. BRKs=%0d irqs=%0d verawr=%0d %s",
                 brkn, irqn, vera_wr,
                 (brkn == 0) ? "*** NO CRASH -- DATE$ path survived ***"
                             : "*** CRASH REPRODUCED ***");
        // KERNAL detect verdicts (RAM bank 0 KVARS, ext_ram_bram):
        // machine_properties=$A871 (bit0=816, bit1=24-bit model),
        // last_far_bank=$A873.  Compare with HW monitor M 00A871 00A873.
        $display("[PROP] machine_properties=%02x last_far_bank=%02x",
                 u_bram.mem[13'h0871], u_bram.mem[13'h0873]);
        $finish;
    end

    initial begin
        repeat (48_000_000) @(posedge cpu_clk);   // 6 s sim time hard stop
        $display("[TBC ] TIMEOUT pc=%04x rom=%02x irqs=%0d verawr=%0d", pc, rom_bank_r, irqn, vera_wr);
        $finish;
    end
endmodule
