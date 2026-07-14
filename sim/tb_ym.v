`timescale 1ns/1ps
// ============================================================================
// tb_ym.v -- YM2151 (IKAOPM) integration smoke test for the 2026-07-06
// audio fix: EMUCLK = pix_clk 25 MHz with /7 phiM cen (3.5714 MHz), CPU bus
// writes handshaked cpu_clk -> pix_clk, status-read idle with pending-write
// BUSY shadow.  This TB replicates the x16.sv glue VERBATIM, programs a
// max-volume alg-7 voice on ch0 through busy-polled writes (like real X16
// software), and PASSES when o_EMU_L/R produce nonzero audio.
// ============================================================================
module tb_ym;
    reg pix_clk = 0; always #20   pix_clk = ~pix_clk;   // 25 MHz
    reg cpu_clk = 0; always #62.5 cpu_clk = ~cpu_clk;   // 8 MHz
    reg sys_rst_n = 0;

    // ---- CPU-side bus stimulus ----
    reg        ym_cs  = 0;
    reg        rwn    = 1;
    reg        a0     = 0;
    reg  [7:0] wdata  = 8'h00;
    wire       ym_wr  = ym_cs & ~rwn;

    // ======================= x16.sv GLUE (verbatim) ========================
    wire [15:0] ym_emu_r, ym_emu_l;
    wire [7:0]  ym_od;
    wire        ym_irq_n;

    reg [1:0] ymrst_sync = 2'b00;
    always @(posedge pix_clk or negedge sys_rst_n)
        if (!sys_rst_n) ymrst_sync <= 2'b00;
        else            ymrst_sync <= {ymrst_sync[0], 1'b1};
    wire ym_reset_n = ymrst_sync[1];

    reg [2:0] ym_div = 3'd0;
    always @(posedge pix_clk) ym_div <= (ym_div == 3'd6) ? 3'd0 : ym_div + 3'd1;
    wire ym_pcen_n = (ym_div != 3'd0);

    reg        ym_req_t = 1'b0;
    reg        ym_wr_d  = 1'b0;
    reg        ym_wa0   = 1'b0;
    reg  [7:0] ym_wdat  = 8'h00;
    always @(posedge cpu_clk) begin
        ym_wr_d <= ym_wr;
        if (ym_wr & ~ym_wr_d) begin
            ym_wa0   <= a0;
            ym_wdat  <= wdata;
            ym_req_t <= ~ym_req_t;
        end
    end

    reg  [2:0] ym_req_s = 3'b000;
    always @(posedge pix_clk) ym_req_s <= {ym_req_s[1:0], ym_req_t};
    wire ym_req_edge = ym_req_s[2] ^ ym_req_s[1];

    reg        ym_ack_t  = 1'b0;
    reg        ym_bus_wr = 1'b0;
    reg  [4:0] ym_hold   = 5'd0;
    reg        ym_a0_r   = 1'b0;
    reg  [7:0] ym_d_r    = 8'h00;
    reg  [7:0] ym_status = 8'h00;
    // post-write: for DATA writes (A0=1), hold the pending/busy shadow until
    // the OPM's own BUSY flag is seen completing (rise then fall) -- IKAOPM
    // silently drops a data write issued while the previous one is still
    // unconsumed (like the real chip), and the raw delivery-ack window left
    // a gap where a fast busy-poll could sneak the next write in too early.
    reg  [1:0] ym_post   = 2'd0;
    reg  [8:0] ym_tmo    = 9'd0;
    always @(posedge pix_clk) begin
        case (ym_post)
        2'd0: begin
            ym_status <= ym_od;              // idle = continuous status capture
            if (ym_req_edge) begin
                ym_a0_r   <= ym_wa0;         // stable: written 2+ cpu cycles ago
                ym_d_r    <= ym_wdat;
                ym_bus_wr <= 1'b1;
                ym_hold   <= 5'd15;          // 16 pix cycles = 640 ns > 2 phiM
                ym_post   <= 2'd1;
            end
        end
        2'd1: begin
            if (ym_hold != 5'd0) ym_hold <= ym_hold - 5'd1;
            else begin
                ym_bus_wr <= 1'b0;
                if (ym_a0_r) begin           // data write: track OPM busy
                    ym_post <= 2'd2;
                    ym_tmo  <= 9'd200;       // ~8 us guard for the rise
                end else begin               // address write: done
                    ym_ack_t <= ~ym_ack_t;
                    ym_post  <= 2'd0;
                end
            end
        end
        2'd2: begin                          // wait busy rise (or timeout)
            ym_tmo <= ym_tmo - 9'd1;
            if (ym_od[7]) begin
                ym_post <= 2'd3;
                ym_tmo  <= 9'd500;           // ~20 us >= real busy duration
            end
            else if (ym_tmo == 9'd0) begin
                ym_ack_t <= ~ym_ack_t;       // consumed faster than visible
                ym_post  <= 2'd0;
            end
        end
        2'd3: begin                          // wait busy fall (bounded: the
            ym_tmo <= ym_tmo - 9'd1;         // TEST reg can repurpose o_D)
            if (!ym_od[7] || ym_tmo == 9'd0) begin
                ym_ack_t <= ~ym_ack_t;
                ym_post  <= 2'd0;
            end
        end
        endcase
    end

    reg [1:0] ym_ack_s = 2'b00;
    reg [7:0] ym_status_s = 8'h00, ym_status_c = 8'h00;
    always @(posedge cpu_clk) begin
        ym_ack_s    <= {ym_ack_s[0], ym_ack_t};
        ym_status_s <= ym_status;
        ym_status_c <= ym_status_s;
    end
    wire ym_pending = ym_req_t ^ ym_ack_s[1];
    wire [7:0] ym_rd_data = {ym_status_c[7] | ym_pending, ym_status_c[6:0]};

    IKAOPM #(
        .FULLY_SYNCHRONOUS  (1),
        .FAST_RESET         (1),
        .USE_BRAM           (0)
    ) u_ym2151 (
        .i_EMUCLK           (pix_clk),
        .i_phiM_PCEN_n      (ym_pcen_n),
        .i_IC_n             (ym_reset_n),
        .i_CS_n             (1'b0),
        .i_RD_n             (ym_bus_wr),
        .i_WR_n             (~ym_bus_wr),
        .i_A0               (ym_bus_wr ? ym_a0_r : 1'b0),
        .i_D                (ym_d_r),
        .o_D                (ym_od),
        .o_D_OE             (),
        .o_CT1              (), .o_CT2              (),
        .o_IRQ_n            (ym_irq_n),
        .o_SH1              (), .o_SH2              (),
        .o_SO               (),
        .o_EMU_R            (ym_emu_r),
        .o_EMU_L            (ym_emu_l),
        .o_EMU_R_SAMPLE     (), .o_EMU_L_SAMPLE     ()
    );
    // ========================================================================

    // busy-polled register write, like real X16 software
    integer guard;
    task ymreg(input [7:0] addr, input [7:0] val);
        begin
            guard = 0;
            while (ym_rd_data[7]) begin       // busy (incl. pending shadow)
                @(posedge cpu_clk); guard = guard + 1;
                if (guard > 100000) begin
                    $display("[YM  ] FAIL: busy never cleared (addr %02x)", addr);
                    $finish;
                end
            end
            a0 <= 0; wdata <= addr; ym_cs <= 1; rwn <= 0; @(posedge cpu_clk);
            ym_cs <= 0; rwn <= 1;   repeat (4) @(posedge cpu_clk);
            guard = 0;
            while (ym_rd_data[7]) begin
                @(posedge cpu_clk); guard = guard + 1;
                if (guard > 100000) begin
                    $display("[YM  ] FAIL: busy stuck after addr write");
                    $finish;
                end
            end
            a0 <= 1; wdata <= val;  ym_cs <= 1; rwn <= 0; @(posedge cpu_clk);
            ym_cs <= 0; rwn <= 1;   repeat (4) @(posedge cpu_clk);
        end
    endtask

    // debug probes into IKAOPM internals
    always @(posedge pix_clk)
        if (ym_bus_wr && ym_hold == 5'd0)
            $display("[DBG ] burst-end a0=%b d=%02x areq=%b dreq=%b busy=%b t=%0t",
                     ym_a0_r, ym_d_r,
                     u_ym2151.REG.areg_rq_inlatch, u_ym2151.REG.dreg_rq_inlatch,
                     ym_od[7], $time);

    // is phi1 alive?  count edges over the settle period
    integer phi1_edges = 0;
    reg phi1_d = 0;
    always @(posedge pix_clk) begin
        phi1_d <= u_ym2151.o_phi1;
        if (u_ym2151.o_phi1 ^ phi1_d) phi1_edges = phi1_edges + 1;
    end

    integer i, nz;
    initial begin
        repeat (2000) @(posedge pix_clk);   // IC_n low ~285 phiM: full OPM reset
        sys_rst_n = 1;
        repeat (4000) @(posedge pix_clk);   // release + core settle

        // minimal loud voice on ch0, algorithm 7 (all slots carriers)
        ymreg(8'h20, 8'hC7);                // RL=LR, FB=0, CONNECT=7
        ymreg(8'h28, 8'h4A);                // KC ch0 ~ A4
        ymreg(8'h40, 8'h01);                // DT1/MUL slot M1
        ymreg(8'h60, 8'h00);                // TL M1 = max volume
        ymreg(8'h68, 8'h00);
        ymreg(8'h70, 8'h00);
        ymreg(8'h78, 8'h00);
        ymreg(8'h80, 8'h1F);                // AR = 31 (instant)
        ymreg(8'h88, 8'h1F);
        ymreg(8'h90, 8'h1F);
        ymreg(8'h98, 8'h1F);
        ymreg(8'h08, 8'h78);                // KEY ON ch0, all slots
        $display("[YM  ] voice programmed (all writes busy-polled), phi1_edges=%0d",
                 phi1_edges);

        // watch ~4 ms of audio for nonzero samples
        nz = 0;
        for (i = 0; i < 100000; i = i + 1) begin
            @(posedge pix_clk);
            if (ym_emu_l !== 16'h0000 || ym_emu_r !== 16'h0000) nz = nz + 1;
        end
        if (nz > 100)
            $display("[YM  ] *** PASS: audio out (%0d nonzero samples), status=%02x ***",
                     nz, ym_rd_data);
        else
            $display("[YM  ] *** FAIL: no audio (%0d nonzero samples), status=%02x ***",
                     nz, ym_rd_data);
        $finish;
    end

    initial begin
        #60000000;
        $display("[YM  ] TIMEOUT");
        $finish;
    end
endmodule
