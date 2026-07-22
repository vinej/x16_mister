`timescale 1ns/1ps
// ============================================================================
// tb_fb.v -- framebuffer 16-bit-tap read/stream path (SDRAM-backed bitmap).
//
// Verifies the additions that make a 640x480 4/8bpp linear framebuffer feasible
// on the single-port MiST SDRAM controller:
//   * sdram.v `dout16` tap  (both byte lanes of a read word)
//   * PLANAR layout          (even px -> low-byte plane A24=0,
//                             odd  px -> high-byte plane A24=1, same word)
//   * ext_ram_sdram fb stream client (fb_go/base/len -> fb_valid/word/done),
//     running back-to-back at ~8 fclk/word, CPU interleaved between words.
//
// Golden framebuffer: pixel(p) = (p + 16) & 0xFF.  Word k packs two pixels:
//   wmem[base+k] = { pixel(2k+1) /*high=odd*/, pixel(2k) /*low=even*/ }
// so a correct read returns fb_word[7:0]=even, fb_word[15:8]=odd -- a lane
// swap or an off-by-one in the planar map is caught immediately.
//
// TEST1  single word (len=1)            -- tap + one word
// TEST2  full 640px line (320 words)    -- stream integrity + cadence
// TEST3  stream WITH concurrent CPU r/w -- interleave: fb intact + CPU correct
// TEST4  planar WRITE round-trip        -- the exact byte_addr x16.sv will emit
// ============================================================================
module tb_fb;
    reg clk  = 0; always #62.5 clk  = ~clk;    // 8 MHz  cpu_clk
    reg fclk = 0; always #5    fclk = ~fclk;   // 100 MHz sdram_clk
    reg res_n = 0;

    // CPU port
    reg         cs = 0, we = 0;
    reg  [24:0] ba = 25'd0;
    reg  [7:0]  wd = 8'd0;
    wire [7:0]  rd;
    wire        ready;

    // Framebuffer stream port
    reg         fb_go   = 0;
    reg  [23:0] fb_base = 0;
    reg  [10:0] fb_len  = 0;
    wire        fb_valid;
    wire [15:0] fb_word;
    wire        fb_done;

    ext_ram_sdram dut (
        .clk(clk), .sdram_clk(fclk), .reset_n(res_n),
        .cs(cs), .we(we), .byte_addr(ba), .wr_data(wd),
        .rd_data(rd), .ready(ready),
        .ld_wr(1'b0), .ld_addr(25'd0), .ld_data(8'd0), .ld_busy(),
        .bk_rd(1'b0), .bk_addr(25'd0), .bk_rdata(), .bk_ack(),
        .wr_snoop(), .wr_snoop_addr(),
        .fb_go(fb_go), .fb_base(fb_base), .fb_len(fb_len),
        .fb_valid(fb_valid), .fb_word(fb_word), .fb_done(fb_done),
        .SDRAM_A(), .SDRAM_DQ(), .SDRAM_BA(), .SDRAM_nCS(), .SDRAM_nWE(),
        .SDRAM_nRAS(), .SDRAM_nCAS(), .SDRAM_CKE(), .SDRAM_CLK(),
        .SDRAM_DQML(), .SDRAM_DQMH()
    );

    localparam [23:0] FB0     = 24'h001000;   // golden read region (word base)
    localparam [23:0] FB0B    = 24'h002000;   // write round-trip region
    localparam integer LINE_W = 320;          // 640 px @ 8bpp = 320 words

    function [7:0] pix(input integer p); pix = (p + 16) & 8'hFF; endfunction

    // ---- free-running fclk tick + collector (all writes stay in this domain)
    integer tick = 0;
    always @(posedge fclk) tick <= tick + 1;

    reg        collect = 0;
    integer    gi = 0;
    reg [15:0] got [0:1023];
    integer    first_tick = -1, last_tick = -1;
    always @(posedge fclk) begin
        if (!collect) begin
            gi <= 0; first_tick <= -1;
        end else if (fb_valid) begin
            got[gi] <= fb_word;
            gi <= gi + 1;
            if (first_tick < 0) first_tick <= tick;
            last_tick <= tick;
        end
    end

    // ---- helpers ----------------------------------------------------------
    task arm_collect;                 // reset the collector, gi -> 0
        begin
            @(negedge fclk); collect <= 1'b0;
            @(posedge fclk);
            @(negedge fclk); collect <= 1'b1;
        end
    endtask

    task start_run(input [23:0] base, input [10:0] len);
        begin
            @(negedge fclk); fb_go <= 1'b1; fb_base <= base; fb_len <= len;
            @(negedge fclk); fb_go <= 1'b0;
        end
    endtask

    // CPU bus cycle (ready-honouring, like the '816: commit on a ready posedge)
    task cpu_cyc(input w, input [24:0] a, input [7:0] d);
        begin
            cs <= 1; we <= w; ba <= a; wd <= d;
            @(negedge clk);
            while (!ready) @(negedge clk);
            @(posedge clk);
            cs <= 0; we <= 0;
        end
    endtask

    integer k, i, j, errors, total_errors;
    integer wi;
    reg [15:0] expw;
    reg [7:0]  rq;

    initial begin
        total_errors = 0;
        repeat (4) @(posedge clk);
        res_n = 1;
        repeat (600) @(posedge fclk);           // SDRAM self-init (~4us) + slack

        // preload the golden framebuffer straight into the model (read-path test)
        for (k = 0; k < LINE_W; k = k + 1)
            dut.u_sdram.wmem[FB0 + k] = { pix(2*k+1), pix(2*k) };

        // ================= TEST1: single word ============================
        errors = 0;
        arm_collect;
        start_run(FB0, 11'd1);
        wait (gi == 1);
        expw = { pix(1), pix(0) };
        if (got[0] !== expw) begin
            errors = errors + 1;
            $display("[FB  ] T1 word0: got %04x want %04x", got[0], expw);
        end
        $display("[FB  ] TEST1 single-word: %s", errors ? "FAIL" : "PASS");
        total_errors = total_errors + errors;

        // ================= TEST2: full 640px line + cadence ==============
        errors = 0;
        arm_collect;
        start_run(FB0, LINE_W[10:0]);
        wait (gi == LINE_W);
        for (k = 0; k < LINE_W; k = k + 1) begin
            expw = { pix(2*k+1), pix(2*k) };
            if (got[k] !== expw) begin
                errors = errors + 1;
                if (errors <= 4)
                    $display("[FB  ] T2 word%0d: got %04x want %04x",
                             k, got[k], expw);
            end
        end
        $display("[FB  ] TEST2 line=%0d words, cadence ~%0d fclk/word (%0d..%0d): %s",
                 LINE_W, (last_tick - first_tick) / (LINE_W - 1),
                 first_tick, last_tick, errors ? "FAIL" : "PASS");
        total_errors = total_errors + errors;

        // ================= TEST3: stream + concurrent CPU r/w ============
        // Kick a full-line stream, then hammer the CPU port at a distinct
        // region.  The fb stream must stay correct AND every CPU access must
        // return the right byte (proves the between-words interleave).
        errors = 0;
        arm_collect;
        start_run(FB0, LINE_W[10:0]);
        for (j = 0; j < 8; j = j + 1)               // CPU writes during stream
            cpu_cyc(1'b1, 25'h020000 + j, 8'h70 + j[7:0]);
        for (j = 0; j < 8; j = j + 1) begin         // CPU reads during stream
            cpu_cyc(1'b0, 25'h020000 + j, 8'h00);
            rq = rd;
            if (rq !== (8'h70 + j[7:0])) begin
                errors = errors + 1;
                $display("[FB  ] T3 cpu rd @%0d: got %02x want %02x",
                         j, rq, 8'h70 + j[7:0]);
            end
        end
        wait (gi == LINE_W);
        for (k = 0; k < LINE_W; k = k + 1) begin
            expw = { pix(2*k+1), pix(2*k) };
            if (got[k] !== expw) begin
                errors = errors + 1;
                if (errors <= 4)
                    $display("[FB  ] T3 word%0d: got %04x want %04x",
                             k, got[k], expw);
            end
        end
        $display("[FB  ] TEST3 interleave (fb + CPU r/w): %s",
                 errors ? "FAIL" : "PASS");
        total_errors = total_errors + errors;

        // ================= TEST4: planar WRITE round-trip ================
        // Write pixels the way x16.sv will: byte_addr = { i[0], FB0B+(i>>1) }.
        // Then stream-read them back and check the pairing.
        errors = 0;
        for (i = 0; i < 2*8; i = i + 1) begin
            wi = FB0B + (i >> 1);
            cpu_cyc(1'b1, { i[0], wi[23:0] }, pix(i));
        end
        repeat (40) @(posedge clk);   // let the CPU write FIFO drain to SDRAM.
                                      // fb reads are frame-coherent (they pull
                                      // from SDRAM and don't stall on the write
                                      // FIFO) -- a real scanout reads a line
                                      // written frames earlier, never the same
                                      // instant, so this drain models reality.
        arm_collect;
        start_run(FB0B, 11'd8);
        wait (gi == 8);
        for (k = 0; k < 8; k = k + 1) begin
            expw = { pix(2*k+1), pix(2*k) };
            if (got[k] !== expw) begin
                errors = errors + 1;
                $display("[FB  ] T4 word%0d: got %04x want %04x", k, got[k], expw);
            end
        end
        $display("[FB  ] TEST4 planar write round-trip: %s",
                 errors ? "FAIL" : "PASS");
        total_errors = total_errors + errors;

        // ================= summary =======================================
        if (total_errors == 0) $display("[FB  ] ALL OK");
        else                   $display("[FB  ] FAILED (%0d mismatches)", total_errors);
        $finish;
    end

    initial begin
        #4000000;
        $display("[FB  ] TIMEOUT (gi=%0d, fb_active=%b)", gi, dut.fb_active);
        $finish;
    end
endmodule
