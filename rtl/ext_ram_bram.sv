//============================================================================
// ext_ram_bram.sv  -  TEST HiRAM ($A000-$BFFF) in on-chip M10K BRAM.
//
// 16 KB = TWO banks (bank 0 / bank 1), selected by bank[0]; higher banks alias
// down by bank[0].  Bank 1 holds the KERNAL keymap (BANK_KEYBD=$01).
//
// STALL_TEST:
//   0 -> single-cycle, ALWAYS ready (known-good 16K config: 16K + keyboard).
//   1 -> reliable BRAM storage but MIMIC the SDRAM ~10-cycle CPU stall
//        (idle -> acc(LAT) -> done, ready = ~cs | done), i.e. it engages the
//        exact same cpu_rdy stall path the SDRAM uses.  Isolates the stall path
//        from the SDRAM chip:
//          * still 16K + keyboard -> stall path FINE; SDRAM is chip/signal.
//          * back to 8K / no kbd  -> the STALL / cpu_rdy path itself is the bug.
//
// One implementation is selected at compile time (generate) so `mem` keeps a
// single clean write port + single read port (M10K-inferable).
//============================================================================
module ext_ram_bram (
    input  logic        clk,
    input  logic        cs,
    input  logic        we,
    input  logic  [7:0] bank,        // RAM bank; only bank[0] distinguishes (2 banks)
    input  logic [12:0] addr,        // offset within $A000-$BFFF
    input  logic  [7:0] wr_data,
    output logic  [7:0] rd_data,
    output logic        ready
);

    localparam       STALL_TEST = 1'b0;   // 1 = mimic SDRAM stall, 0 = always ready
    // LAT controls the stall length.  SDRAM needs ~9 (CYCLE_LEN); VERA's working
    // stall is only ~2.  Testing LAT=2 (a short, VERA-like stall) vs LAT=9:
    //   * LAT=2 works (16K+kbd) but LAT=9 fails -> the CPU core can't survive a
    //     LONG stall -> fix = make the SDRAM access short (faster SDRAM clock so
    //     ~9 SDRAM cycles = ~1-2 CPU cycles).
    //   * LAT=2 also fails -> the stall MECHANISM is wrong (off-by-one / data
    //     timing), not the length -> fix the stall handshake itself.
    localparam [3:0] LAT        = 4'd2;

    (* ramstyle = "M10K" *) logic [7:0] mem [0:16383];   // 16 KB, zero-init by config
    wire [13:0] a = {bank[0], addr};

    generate
    if (STALL_TEST) begin : g_stall
        // mimic the SDRAM controller's idle -> acc(LAT) -> done timing
        localparam [1:0] ST_IDLE = 2'd0, ST_ACC = 2'd1, ST_DONE = 2'd2;
        logic [1:0] st  = ST_IDLE;
        logic [3:0] cyc = 4'd0;
        logic [7:0] rd_st;
        wire        acc_last = (st == ST_ACC) && (cyc == LAT);

        always_ff @(posedge clk) begin
            case (st)
                ST_IDLE: if (cs) begin cyc <= 4'd0; st <= ST_ACC; end
                ST_ACC: begin
                    cyc <= cyc + 4'd1;
                    if (cyc == LAT) st <= ST_DONE;
                end
                ST_DONE: if (!cs) st <= ST_IDLE;   // hold ready until CPU moves off
                default: st <= ST_IDLE;
            endcase
        end

        // clean single write port + single read port (both posedge)
        always_ff @(posedge clk) if (acc_last && we) mem[a] <= wr_data;
        always_ff @(posedge clk) if (acc_last)       rd_st  <= mem[a];

        assign rd_data = rd_st;
        assign ready   = ~cs | (st == ST_DONE);
    end else begin : g_fast
        // POSEDGE-registered read with a VERA-style 1-cycle stall (BUG3 fix).
        //
        // This used to be a negedge-read, always-ready port like lowram_bram.
        // That works for lowram, but HiRAM's longer decode/mux path (bank
        // compare, bram/sdram select, self-test muxes) made the half-cycle
        // read marginal on silicon: the R49 SAVE path -- the only code that
        // does DENSE SEQUENTIAL HiRAM READS (lda sector_buffer,y, buffer at
        // $B9C6 bank 0) -- read STALE bytes on alternating addresses, which
        // shredded every written sector (see rom/sdcard_corrupted*.img:
        // odd offsets hold the previous byte's value).  RTL sim of the
        // negedge path is clean; this is a silicon/timing artifact, so the
        // fix is structural: full-cycle synchronous read + ready handshake
        // (reads honor RDY; VERA proves the CPU handles short read stalls).
        // WRITES stay single-cycle and NEVER stall -- the r65c02 ignores RDY
        // on write cycles, so a stalled write would be lost, not held.
        logic [7:0] rd_q;
        logic       rd_done;
        always_ff @(posedge clk) if (cs & we) mem[a] <= wr_data;
        always_ff @(posedge clk) begin
            rd_q    <= mem[a];                    // registered on the FULL cycle
            rd_done <= cs & ~we & ~rd_done;       // data valid the cycle after
        end

        assign rd_data = rd_q;
        assign ready   = ~cs | we | rd_done;
    end
    endgenerate

endmodule
