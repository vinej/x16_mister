`timescale 1ns/1ps
// ============================================================================
// RELAUNCH SD-wedge reproduction  (tester bug, 2026-07-15).
//
// Scenario the user hits: mount a FAT32 .img, DOS"$ works.  RELAUNCH the core
// (RBF reconfig, no power cycle) -> LOCKED UP, no flashing cursor.  Fixed by
// remount-img or CPU-toggle.
//
// Difference from tb_wedge.v: there the mount happens AFTER the CPU boots (the
// user types @$).  Here the image is an SC0 *auto-remount* slot, so Main
// re-announces it AROUND/BEFORE the CPU boots.  vsd_sel (x16.sv:248-249, NO
// reset) is therefore already 1 when the R49 ROM runs its boot SD probe
// (select -> CMD17 lba0 -> token poll -> deselect, replayed by sdwedge.s
// phase 1).  That CMD17 hits a SELECTED sd_card and can wedge read_state.
//
// Timeline modelled here:
//   * vsd_sel starts 1 (auto-remount already happened) unless the fix resets it
//   * the HPS fill model is LIVE (image present) but its readiness to serve
//     sd_rd LAGS reset-release by HPS_READY_LAG cycles (Main's virtual-disk
//     path is not instantaneous after core config) -- during that window the
//     boot-probe CMD17's prefetch sd_rd goes UN-acked
//   * mnt is pre-set: the CPU flows straight from the boot probe into the real
//     reads (like the ROM probing at boot, then F7 later hitting the SD)
//
//   FIX=0 : vsd_sel = |img_size latched on mount, no reset   (SHIPPING = BUG)
//   FIX=1 : vsd_sel forced 0 while ~res_n (deselect through the boot probe),
//           re-latched after release                          (candidate fix)
//
// Expected: FIX=0 -> post-probe reads FAIL (read_state wedged non-IDLE);
//           FIX=1 -> READ OK.
// ============================================================================
module tb_relaunch #(
    parameter integer FIX           = 0,
    parameter integer BYTE_SPACING  = 16,   // clk cycles between served data bytes
    parameter integer ACK_LEAD      = 200,  // clk cycles sd_rd -> ack asserted
    parameter integer ACK_DROP      = 8,    // clk cycles last byte -> ack dropped
    parameter integer HPS_READY_LAG = 4000, // clk cycles after res_n before Main serves
    parameter integer WATCHDOG      = 12000000, // ns of sim time before declaring a wedge
    parameter integer BOOT_HOLD     = 12000000, // FIX=4: clk cycles vsd_sel held low post-reset
    parameter integer MHALF         = 3          // SPI master sck half-period-1 (0 = fastest)
);
    reg clk = 0; always #5 clk = ~clk;
    reg res_n = 0;

    wire        r_w_n, sync;
    wire [15:0] addr;
    wire [7:0]  dout;
    wire [15:0] pc;
    reg  [7:0]  din;

    // ---- memory ----
    reg [7:0] ram [0:32767];    // $0000-$7FFF
    reg [7:0] rom [0:8191];     // $E000-$FFFF
    integer i;
    initial begin
        $readmemh("sdwedge.hex", rom);
        for (i=0;i<32768;i=i+1) ram[i]=0;
        // ram[2] (mnt) is set by the TB AFTER the boot probe -- for FIX=4 also
        // after the boot hold expires -- modeling the user reaching F7 seconds
        // after boot, not in the same instant the probe ends.
    end

    wire acc_9f3e = (addr == 16'h9F3E);
    wire acc_9f3f = (addr == 16'h9F3F);

    // ---- cpu_rdy (from x16.sv) ----
    wire vera_read = (acc_9f3e | acc_9f3f) & r_w_n;
    reg [1:0] vera_read_stall = 0;
    always @(posedge clk or negedge res_n)
        if (!res_n) vera_read_stall <= 0;
        else if (vera_read) begin if (vera_read_stall!=2'd3) vera_read_stall <= vera_read_stall+2'd1; end
        else vera_read_stall <= 0;

    // ---- the cpu_clk SPI master: the REAL RTL (rtl/spi_sd_master.sv) ----
    wire spi_stall;
    wire cpu_rdy_base = (~vera_read | (vera_read_stall >= 2'd2));
    wire cpu_rdy = cpu_rdy_base & ~spi_stall;
    wire rd_9f3e = cpu_rdy &  r_w_n & acc_9f3e;
    wire miso;
    wire m_sck, m_mosi, m_sel;
    wire [7:0] m_data, m_status;

    spi_sd_master #(.M_HALF(MHALF[2:0])) u_spimaster (
        .clk(clk), .rst_n(res_n),
        .rd_data(rd_9f3e),
        .cpu_do(dout), .acc_data(acc_9f3e), .acc_ctrl(acc_9f3f), .cpu_rwn(r_w_n),
        .stall(spi_stall), .data_q(m_data), .status_q(m_status),
        .sck(m_sck), .mosi(m_mosi), .sel(m_sel), .miso(miso)
    );

    // ---- din mux ----
    always @(*) begin
        if      (addr < 16'h8000) din = ram[addr[14:0]];
        else if (acc_9f3e)        din = m_data;
        else if (acc_9f3f)        din = m_status;
        else if (addr >= 16'hE000)din = rom[addr[12:0]];
        else                      din = 8'h00;
    end
    always @(posedge clk) if (~r_w_n && addr < 16'h8000) ram[addr[14:0]] <= dout;

    // ---- CPU ----
    r65c02_wrap u_cpu(.clk(clk), .enable(cpu_rdy), .res_n(res_n), .irq_n(1'b1),
        .nmi_n(1'b1), .rdy(1'b1), .r_w_n(r_w_n), .sync(sync), .addr(addr),
        .din(din), .dout(dout), .pc(pc));

    // ---- mount state + the vsd_sel gating under test ----
    // AUTO-REMOUNT: img already mounted, size set, BEFORE the CPU boots.
    reg         img_mounted = 0;
    reg  [63:0] img_size    = 64'd104857600;   // 100 MB, already known at config
    reg         vsd_sel_raw = 1;               // auto-remount latched it to 1

    // FIX=1 : force vsd_sel low ONLY while ~res_n (naive -- probe runs after
    //          release, so vsd_sel is 1 again by then; expected NOT to help).
    always @(posedge clk) if (img_mounted) vsd_sel_raw <= |img_size;

    // FIX=4 : hold vsd_sel LOW for BOOT_HOLD cycles after reset release, so the
    //          card stays DESELECTED all through the ROM's boot SD probe --
    //          exactly the first-launch condition.  After the hold expires the
    //          card selects normally and the guest's CMD13/CMD0 re-inits it.
    reg boot_hold = 1'b1;
    integer bhc = 0;
    always @(posedge clk or negedge res_n)
        if (!res_n) begin boot_hold <= 1'b1; bhc <= 0; end
        else if (boot_hold) begin
            if (bhc >= BOOT_HOLD) boot_hold <= 1'b0;
            else bhc <= bhc + 1;
        end

    wire vsd_sel = (FIX == 1) ? (res_n ? vsd_sel_raw : 1'b0)
                 : (FIX == 4) ? (vsd_sel_raw & ~boot_hold)
                 :               vsd_sel_raw;
    wire ss_pin  = ~m_sel | ~vsd_sel;

    // ---- sd_card ----
    wire [31:0] sd_lba; wire sd_rd, sd_wr; reg sd_ack=0;
    reg [8:0] sd_buff_addr=0; reg [7:0] sd_buff_dout=0; wire [7:0] sd_buff_din; reg sd_buff_wr=0;
    wire [7:0] dbg_bufdout, dbg_mem510; wire [8:0] dbg_bufptr;
    wire [1:0] dbg_spibuf, dbg_sdbuf; wire [2:0] dbg_wrstate, dbg_rdstate;

    // FIX=3 : the real candidate.  While the card is DESELECTED (ss high) and
    //         the read machine is parked in an ABANDONED read (START=1 or
    //         WAIT_IO=2 -- i.e. a CMD17 whose data never got serialised), pulse
    //         sd_card.reset for one cycle.  This clears read_state/pref_state/
    //         sd_rd so the guest's next CMD17 is ACCEPTED.  The SPI shifter is
    //         idle while deselected, so the reset is harmless there; an
    //         in-progress data transfer (SEND_TOKEN/DATA=3/4) is NEVER touched.
    localparam RD_START = 3'd1, RD_WAIT_IO = 3'd2;
    wire abandoned_rd = ss_pin & ((dbg_rdstate == RD_START) || (dbg_rdstate == RD_WAIT_IO));
    wire sd_reset = ~res_n | ((FIX == 3) & abandoned_rd);

    sd_card dut(.clk_sys(clk), .reset(sd_reset), .sdhc(1'b1), .img_mounted(img_mounted),
        .img_size(img_size), .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
        .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
        .clk_spi(clk), .ss(ss_pin), .sck(m_sck), .mosi(m_mosi), .miso(miso),
        .dbg_bufdout(dbg_bufdout), .dbg_bufptr(dbg_bufptr), .dbg_spibuf(dbg_spibuf),
        .dbg_sdbuf(dbg_sdbuf), .dbg_wrstate(dbg_wrstate), .dbg_rdstate(dbg_rdstate), .dbg_mem510(dbg_mem510));

    // sector content the "image" holds, distinguishable per lba
    function [7:0] secbyte(input [8:0] n, input [31:0] lba);
        if (lba == 0)
            secbyte = (n==9'd450)?8'h0C:(n==9'd510)?8'h55:(n==9'd511)?8'hAA:n[7:0];
        else
            secbyte = n[7:0] ^ 8'hA5;
    endfunction

    // ========================================================================
    // ACCURATE hps_io read-fill model -- image PRESENT, but readiness LAGS reset
    // ========================================================================
    localparam FS_IDLE=0, FS_LEAD=1, FS_FILL=2, FS_DRAIN=3;
    reg [1:0]  fstate = FS_IDLE;
    reg [2:0]  b_wr = 0;
    integer    bidx = 0;
    integer    spc  = 0;
    integer    leadc= 0;
    integer    dropc= 0;
    integer    readyc = 0;
    reg        hps_ready = 0;      // Main able to serve sd_rd
    reg [31:0] serve_lba = 0;

    always @(posedge clk) begin
        sd_buff_wr <= b_wr[0];
        if (b_wr[2] && ~(&sd_buff_addr)) sd_buff_addr <= sd_buff_addr + 1'b1;
        b_wr <= (b_wr << 1);

        if (!res_n) begin
            fstate<=FS_IDLE; b_wr<=0; sd_ack<=0; sd_buff_wr<=0;
            sd_buff_addr<=0; bidx<=0; spc<=0; leadc<=0; dropc<=0;
            readyc<=0; hps_ready<=0;
        end else begin
            if (!hps_ready) begin
                if (readyc >= HPS_READY_LAG) hps_ready <= 1;
                else readyc <= readyc + 1;
            end
            case (fstate)
                FS_IDLE:
                    if (hps_ready && (sd_rd || sd_wr)) begin leadc<=0; fstate<=FS_LEAD; end

                FS_LEAD:
                    if (leadc >= ACK_LEAD) begin
                        serve_lba    <= sd_lba;
                        sd_ack       <= 1'b1;
                        sd_buff_addr <= 0;
                        sd_buff_dout <= secbyte(9'd0, sd_lba);
                        b_wr         <= 3'b001;
                        bidx         <= 1;
                        spc          <= 0;
                        fstate       <= FS_FILL;
                        $display("[HPS ] t=%0t serving sd_rd for lba=%0d", $time, sd_lba);
                    end else leadc <= leadc + 1;

                FS_FILL:
                    if (bidx < 512) begin
                        if (spc >= BYTE_SPACING) begin
                            spc          <= 0;
                            sd_buff_dout <= secbyte(bidx[8:0], serve_lba);
                            b_wr         <= 3'b001;
                            bidx         <= bidx + 1;
                        end else spc <= spc + 1;
                    end else begin
                        dropc  <= 0;
                        fstate <= FS_DRAIN;
                    end

                FS_DRAIN:
                    if (dropc >= ACK_DROP) begin
                        sd_ack <= 1'b0;
                        fstate <= FS_IDLE;
                    end else dropc <= dropc + 1;
            endcase
        end
    end

    // ---- trace ----
    reg last_ack=0; reg [1:0] last_sdbuf=3, last_spibuf=3; reg [2:0] last_rds=7;
    always @(posedge clk) begin
        if (sd_ack !== last_ack) begin
            $display("[ACK ] t=%0t sd_ack -> %0b (lba=%0d)", $time, sd_ack, sd_lba);
            last_ack <= sd_ack;
        end
        if (dut.sd_buf  !== last_sdbuf)  begin $display("[SDBUF ] t=%0t sd_buf  -> %0d", $time, dut.sd_buf);  last_sdbuf  <= dut.sd_buf;  end
        if (dut.spi_buf !== last_spibuf) begin $display("[SPIBUF] t=%0t spi_buf -> %0d", $time, dut.spi_buf); last_spibuf <= dut.spi_buf; end
        if (dbg_rdstate !== last_rds)    begin $display("[RDST  ] t=%0t read_state -> %0d (ss=%b sel=%b sd_rd=%b)", $time, dbg_rdstate, ss_pin, m_sel, sd_rd); last_rds <= dbg_rdstate; end
    end
    reg last_ab=0;
    always @(posedge clk) begin
        if (FIX==3 && abandoned_rd && !last_ab)
            $display("[FIX3] t=%0t reset ASSERTED (read_state=%0d ss=%b sel=%b)", $time, dbg_rdstate, ss_pin, m_sel);
        last_ab <= abandoned_rd;
    end

    // ---- run control ----
    integer k, badA, badB;
    reg [7:0] exp;
    initial begin
        $display("[CFG ] FIX=%0d BYTE_SPACING=%0d ACK_LEAD=%0d ACK_DROP=%0d HPS_READY_LAG=%0d",
                 FIX, BYTE_SPACING, ACK_LEAD, ACK_DROP, HPS_READY_LAG);
        repeat(20) @(posedge clk); res_n = 1;
        $display("[TBC ] t=%0t res_n released.  vsd_sel=%0b (FIX=%0d)  -- CPU boots, probes SD, then reads", $time, vsd_sel, FIX);
        fork
            begin : t
                // The boot probe runs (phase 1).  Wait for it to finish.
                while (ram[1] != 8'h11) @(posedge clk);
                $display("[TBC ] t=%0t boot probe done. read_state=%0d sd_rd=%0b sd_buf=%0d spi_buf=%0d sd_lba=%0d",
                         $time, dbg_rdstate, sd_rd, dut.sd_buf, dut.spi_buf, sd_lba);

                // The user reaches F7 SECONDS after boot -- long after any
                // boot hold has expired.  Wait for the hold, plus margin.
                while (boot_hold) @(posedge clk);
                repeat (1000) @(posedge clk);
                ram[2] = 8'h01;
                $display("[TBC ] t=%0t F7 typed (mnt set): vsd_sel=%0b read_state=%0d", $time, vsd_sel, dbg_rdstate);

                // real reads (CMD0/CMD8/CMD17 lba0 -> $0400, CMD17 lba2048 ->
                // $0600); wait for the CPU to finish (or the watchdog fires).
                while (ram[0] != 8'haa) @(posedge clk);
                $display("[TBC ] DONE. CMD0 R1=%02x (exp 01)  CMD8 R1=%02x (exp 01)", ram[3], ram[4]);
                $display("[TBC ] secA 450/510/511 = %02x/%02x/%02x (exp 0C/55/AA)",
                    ram[16'h0400+450], ram[16'h0400+510], ram[16'h0400+511]);

                badA = 0; badB = 0;
                for (k=0;k<512;k=k+1) begin
                    exp = secbyte(k[8:0], 32'd0);
                    if (ram[16'h0400+k] !== exp) badA = badA + 1;
                    exp = secbyte(k[8:0], 32'd2048);
                    if (ram[16'h0600+k] !== exp) badB = badB + 1;
                end
                if (badA==0 && badB==0)
                    $display("[TBC ] *** RELAUNCH TEST (FIX=%0d): READ OK (both sectors match) ***", FIX);
                else
                    $display("[TBC ] *** RELAUNCH TEST (FIX=%0d): READ FAILED (lba0: %0d/512 bad, lba2048: %0d/512 bad) ***",
                             FIX, badA, badB);
                disable w;
            end
            begin : w
                #(WATCHDOG);
                $display("[TBC ] TIMEOUT (pc=%04x ram1=%02x ram0=%02x read_state=%0d) -- WEDGED",
                         pc, ram[1], ram[0], dbg_rdstate);
                $display("[TBC ] *** RELAUNCH TEST (FIX=%0d): READ FAILED (WEDGE -- CPU hung, no cursor) ***", FIX);
                disable t;
            end
        join
        $finish;
    end
endmodule
