`timescale 1ns/1ps
// ============================================================================
// tb_pcm.v -- VERA PCM audio END-TO-END: the real VERA `top` (extbus register
// decode -> pcm.v sample-rate engine -> audio_fifo -> dacif I2S) feeding the
// shipping i2s_rx deserializer, exactly the x16.sv integration.
//
// (tb_periph only proved dacif <-> i2s_rx with hand-driven samples; the PCM
//  FIFO/register path itself was untested before this TB.)
//
// Checks, all through CPU-shaped extbus cycles at $9F3B-$9F3D (VERA a=1B-1D):
//   1. audio_ctrl readback: FIFO empty/full flags, mode/volume bits
//   2. 16-bit stereo playback @ rate $80 (48828 Hz), volume 15
//   3. FIFO drains -> output returns to 0, empty flag pops back up
//   4. 8-bit mono playback
//   5. ISR bit 3 (audio_fifo_low) visible at $9F27
//   6. volume 0 -> silence even with data flowing
//
// SCALE (intentional, matches real HW): i2s_rx taps the TOP 16 of the 24-bit
// I2S word (proven so in tb_periph).  PCM vol 15 scales a 16-bit sample s by
// x128 into that word, so the recovered value is s>>1 (arithmetic) -- real
// VERA also peaks 16-bit PCM at half the DAC range (headroom to sum PSG+PCM).
//
// QUIRK (upstream pcm.v, real X16 too): the FSM zeroes its output while IDLE
// with an empty FIFO, so the LAST queued sample is latched for <1 frame and
// never reaches the DAC -- a drained FIFO drops its final sample.  Tests
// queue N+1 samples and check the first N.
// ============================================================================
module tb_pcm;
    integer errors = 0;

    // 25 MHz VERA pixel clock
    reg clk25 = 0; always #20 clk25 = ~clk25;

    // ---- VERA (real top) ----
    reg        cs_n = 1, rd_n = 1, wr_n = 1;
    reg  [4:0] a = 0;
    reg  [7:0] dq_out = 8'h00;
    reg        dq_drive = 0;
    wire [7:0] extbus_d = dq_drive ? dq_out : 8'hZZ;
    wire       irq_n;
    wire       lrck, bck, sdata;

    top u_vera (
        .clk25          (clk25),
        .extbus_cs_n    (cs_n),
        .extbus_rd_n    (rd_n),
        .extbus_wr_n    (wr_n),
        .extbus_a       (a),
        .extbus_d       (extbus_d),
        .extbus_irq_n   (irq_n),
        .vga_r(), .vga_g(), .vga_b(),
        .vga_hsync(), .vga_vsync(), .vga_de(),
        .spi_sck(), .spi_mosi(), .spi_miso(1'b1), .spi_ssel_n_sd(),
        .audio_lrck     (lrck),
        .audio_bck      (bck),
        .audio_data     (sdata),
        .dbg_wrdata_r(), .dbg_wraddr_r(), .dbg_do_write(),
        .dbg_video_mode(), .dbg_dcsel(),
        .spi_busy_out(), .spi_autotx_out(),
        .composite_luma(), .composite_chroma()
    );

    // ---- the shipping deserializer, same clock + tap as x16.sv ----
    wire signed [15:0] al, ar;
    i2s_rx u_rx (
        .clk(clk25), .lrck(lrck), .bck(bck), .data(sdata),
        .left(al), .right(ar)
    );

    // ---- CPU-shaped extbus cycles (x16.sv widens strobes to ~6 clk25) ----
    task vwr(input [4:0] addr, input [7:0] val); begin
        @(posedge clk25);
        a = addr; dq_out = val; dq_drive = 1;
        cs_n = 0; wr_n = 0;
        repeat (8) @(posedge clk25);
        cs_n = 1; wr_n = 1; dq_drive = 0;
        repeat (8) @(posedge clk25);
    end endtask

    reg [7:0] rdv;
    task vrd(input [4:0] addr); begin
        @(posedge clk25);
        a = addr; cs_n = 0; rd_n = 0;
        repeat (8) @(posedge clk25);
        rdv = extbus_d;
        cs_n = 1; rd_n = 1;
        repeat (8) @(posedge clk25);
    end endtask

    task exp(input [7:0] got, input [7:0] expct, input [127:0] what); begin
        if (got !== expct) begin
            $display("[PCM ] FAIL %0s: got %02x exp %02x", what, got, expct);
            errors = errors + 1;
        end
    end endtask

    // Capture one (L,R) pair per I2S frame, on lrck FALLING: left updates
    // during the lrck=0 half, right during the lrck=1 half, so at the falling
    // edge both halves of the SAME audio frame have just completed.
    reg               lrck_d = 0;
    reg signed [15:0] capl [0:63];
    reg signed [15:0] capr [0:63];
    integer           ncap = 0;
    reg               capturing = 0;
    always @(posedge clk25) begin
        lrck_d <= lrck;
        if (capturing && !lrck && lrck_d && ncap < 64) begin
            capl[ncap] <= al;
            capr[ncap] <= ar;
            ncap       <= ncap + 1;
        end
    end

    // find `want` in capl, then require the (L,R) sequence to follow
    integer f;
    task find_seq4(input signed [15:0] l0, input signed [15:0] r0,
                   input signed [15:0] l1, input signed [15:0] r1); begin
        f = -1;
        begin : srch
            integer i;
            for (i = 0; i < ncap; i = i + 1)
                if (capl[i] === l0) begin f = i; disable srch; end
        end
        if (f < 0 || f + 1 >= ncap) begin
            $display("[PCM ] FAIL sample 0 (%04x) never appeared (ncap=%0d)", l0, ncap);
            begin : dump
                integer d;
                for (d = 0; d < ncap; d = d + 1)
                    $display("[PCMD]   frame %0d: L=%04x R=%04x", d, capl[d], capr[d]);
            end
            errors = errors + 1;
        end else begin
            if (capr[f]   !== r0) begin $display("[PCM ] FAIL R0: got %04x exp %04x", capr[f],   r0); errors = errors + 1; end
            if (capl[f+1] !== l1) begin $display("[PCM ] FAIL L1: got %04x exp %04x", capl[f+1], l1); errors = errors + 1; end
            if (capr[f+1] !== r1) begin $display("[PCM ] FAIL R1: got %04x exp %04x", capr[f+1], r1); errors = errors + 1; end
        end
    end endtask

`ifdef PSGDBG
    initial begin
        #15_000_000;
        $display("[PDBG] psg: state=%0d ch=%0d lfsr=%02x Lsmp=%0d Lacc=%0d attr_rd=%02x sig=%0d",
                 u_vera.audio.psg.state, u_vera.audio.psg.cur_channel_r,
                 u_vera.audio.psg.lfsr_r,
                 $signed(u_vera.audio.psg.left_sample_r),
                 $signed(u_vera.audio.psg.left_accum_r),
                 u_vera.audio.psg.attr_rddata,
                 $signed(u_vera.audio.psg.signed_signal));
    end
    reg dbg_on = 0;
    always @(posedge clk25) begin
        if (dbg_on && u_vera.ib_do_access_r && u_vera.ib_write_r)
            $display("[PDBG] ib wr addr=%05x data=%02x audio_write=%b",
                     u_vera.ib_addr_r, u_vera.ib_wrdata_r, u_vera.audio_write);
    end
`endif

    integer i, zi;
    initial begin
        // VERA self-resets via its internal reset_sync; give it a moment
        repeat (400) @(posedge clk25);

        // ---- 1. control register / flags ----
        vwr(5'h1B, 8'h80);                       // FIFO reset
        vwr(5'h1B, 8'h3F);                       // 16-bit | stereo | vol 15
        vrd(5'h1B); exp(rdv, 8'h7F, "ctrl empty+mode"); // full=0 empty=1 + $3F
        vrd(5'h1C); exp(rdv, 8'h00, "rate off");

        // ---- 2. queue three 16-bit stereo samples (L lo,hi / R lo,hi);
        //         the third is the drain-swallowed sacrifice ----
        vwr(5'h1D, 8'h34); vwr(5'h1D, 8'h12);    // L0 = $1234
        vwr(5'h1D, 8'hCC); vwr(5'h1D, 8'hED);    // R0 = $EDCC (-$1234)
        vwr(5'h1D, 8'h00); vwr(5'h1D, 8'h40);    // L1 = $4000
        vwr(5'h1D, 8'h00); vwr(5'h1D, 8'hC0);    // R1 = $C000 (-$4000)
        vwr(5'h1D, 8'hFF); vwr(5'h1D, 8'h7F);    // L2 = $7FFF (never plays)
        vwr(5'h1D, 8'h01); vwr(5'h1D, 8'h80);    // R2 = $8001 (never plays)
        vrd(5'h1B); exp(rdv, 8'h3F, "ctrl: not empty");

        // ISR bit3 (audio_fifo_low): 12 of 4096 bytes -> low is set
        vrd(5'h07); exp(rdv & 8'h08, 8'h08, "ISR fifo-low");

        // ---- start playback at $80 = 48828 Hz (1 sample per I2S frame);
        //      expected at i2s_rx = sample>>1 (vol 15 = x128 into a 24-bit
        //      word, top-16 tap) ----
        ncap = 0; capturing = 1;
        vwr(5'h1C, 8'h80);
        repeat (10 * 512) @(posedge clk25);      // ~10 frames
        capturing = 0;

        find_seq4(16'sh091A, -16'sh091A, 16'sh2000, -16'sh2000);

        // ---- 3. FIFO drained: silence (sample 2 swallowed) + empty flag ----
        if (f >= 0 && f + 2 < ncap) begin
            if (capl[f+2] !== 16'sh0000 || capr[f+2] !== 16'sh0000) begin
                $display("[PCM ] FAIL post-drain not silent: %04x/%04x", capl[f+2], capr[f+2]);
                errors = errors + 1;
            end
        end
        vrd(5'h1B); exp(rdv, 8'h7F, "ctrl: drained");
        vwr(5'h1C, 8'h00);                       // rate off again

        // ---- 4. 8-bit mono: byte b -> both channels = {b,$00} >> 1 ----
        vwr(5'h1B, 8'h8F);                       // FIFO reset | 8-bit mono vol 15
        vwr(5'h1D, 8'h40);                       // $40 -> $4000 -> $2000
        vwr(5'h1D, 8'hA0);                       // $A0 -> $A000 -> $D000
        vwr(5'h1D, 8'h20);                       // sacrifice (never plays)
        ncap = 0; capturing = 1;
        vwr(5'h1C, 8'h80);
        repeat (8 * 512) @(posedge clk25);
        capturing = 0;
        find_seq4(16'sh2000, 16'sh2000, -16'sh3000, -16'sh3000); // $D000 = -$3000

        // ---- 6. volume 0 -> silence even with data queued ----
        vwr(5'h1C, 8'h00);
        vwr(5'h1B, 8'h80);                       // FIFO reset
        vwr(5'h1B, 8'h30);                       // 16-bit stereo, vol 0
        vwr(5'h1D, 8'h00); vwr(5'h1D, 8'h7F);    // L = $7F00
        vwr(5'h1D, 8'h00); vwr(5'h1D, 8'h7F);    // R = $7F00
        vwr(5'h1D, 8'h00); vwr(5'h1D, 8'h7F);    // extra sample
        vwr(5'h1D, 8'h00); vwr(5'h1D, 8'h7F);
        ncap = 0; capturing = 1;
        vwr(5'h1C, 8'h80);
        repeat (6 * 512) @(posedge clk25);
        capturing = 0;
        begin : zchk
            for (zi = 0; zi < ncap; zi = zi + 1)
                if (capl[zi] !== 16'sh0000 || capr[zi] !== 16'sh0000) begin
                    $display("[PCM ] FAIL vol0 not silent: frame %0d = %04x/%04x",
                             zi, capl[zi], capr[zi]);
                    errors = errors + 1;
                    disable zchk;
                end
        end

        // ---- 7. PSG level check (jyv 2026-07-07, official 48.0.1 audio):
        //      ch0 full-volume 50% pulse => the top-16 I2S tap must see the
        //      upstream amplitudes: +((31*511)>>3)>>1 = +990 and
        //      -((32*511)>>3)>>1 = -1022.  Then volume 0 => silence.
        begin : psg_level
            integer mn, mx, ci;
`ifdef PSGDBG
            dbg_on = 1;
`endif
            vwr(5'h00, 8'hC0);                 // ADDR0 = $1F9C0, incr +1
            vwr(5'h01, 8'hF9);
            vwr(5'h02, 8'h11);
            vwr(5'h03, 8'h00);                 // freq lo  ($1000 -> 16-sample period)
            vwr(5'h03, 8'h10);                 // freq hi
            vwr(5'h03, 8'hFF);                 // L+R enable, volume 63
            vwr(5'h03, 8'h20);                 // pulse waveform, 50% width
`ifdef PSGDBG
            $display("[PDBG] ADDR0 now %05x, psg attr[0..3]=%02x %02x %02x %02x",
                     u_vera.vram_addr_0_r,
                     u_vera.audio.psg.audio_attr_ram.mem[0],
                     u_vera.audio.psg.audio_attr_ram.mem[1],
                     u_vera.audio.psg.audio_attr_ram.mem[2],
                     u_vera.audio.psg.audio_attr_ram.mem[3]);
            dbg_on = 0;
`endif
            repeat (20000) @(posedge clk25);   // let it run a few periods
            ncap = 0; capturing = 1;
            repeat (40000) @(posedge clk25);   // ~40 audio frames
            capturing = 0;
            mn = 0; mx = 0;
            for (ci = 0; ci < ncap; ci = ci + 1) begin
                if (capl[ci] > mx) mx = capl[ci];
                if (capl[ci] < mn) mn = capl[ci];
            end
            if (mx < 985 || mx > 995 || mn > -1017 || mn < -1027) begin
                $display("[PCM ] FAIL PSG level: min %0d (exp ~-1022) max %0d (exp ~+990)", mn, mx);
                errors = errors + 1;
            end

            // volume 0 -> silence
            vwr(5'h00, 8'hC2);                 // $1F9C2 (vol byte)
            vwr(5'h01, 8'hF9);
            vwr(5'h02, 8'h11);
            vwr(5'h03, 8'hC0);                 // L+R on, volume 0
            repeat (20000) @(posedge clk25);
            ncap = 0; capturing = 1;
            repeat (20000) @(posedge clk25);
            capturing = 0;
            for (ci = 0; ci < ncap; ci = ci + 1)
                if (capl[ci] !== 16'sd0) begin
                    if (errors < 12)
                        $display("[PCM ] FAIL PSG vol0 not silent: %0d", capl[ci]);
                    errors = errors + 1;
                end
        end

        if (errors == 0) $display("[PCM ] *** VERA PCM END-TO-END: ALL PASS ***");
        else             $display("[PCM ] *** VERA PCM END-TO-END: %0d FAILURES ***", errors);
        $finish;
    end

    initial begin #20000000; $display("[PCM ] TIMEOUT"); $finish; end
endmodule
