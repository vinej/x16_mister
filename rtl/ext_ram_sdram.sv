//============================================================================
// ext_ram_sdram.sv  -  X16 banked memory backed by SDRAM.
//
// Serves TWO CPU windows through one controller (the parent muxes them into
// the single byte_addr port):
//   * HiRAM  $A000-$BFFF, RAM banks 2..255   -> SDRAM 0x000000-0x1FFFFF
//   * Cart   $C000-$FFFF, ROM banks 32..255  -> SDRAM 0x480000-0x7FFFFF
//     (byte_addr = 0x400000 + bank*16K + offset; banks 32-255 are the X16
//      cartridge space and are READ/WRITE, i.e. cart RAM/flash semantics)
//
// DUAL-CLOCK design:
//   * The MiST byte controller (sdram.v) + the access FSM run on sdram_clk
//     (100 MHz) -- the speed this controller was designed for.  An SDRAM
//     access is ~9 fast cycles (~90 ns), so the CPU stall is only ~3 cpu_clk
//     cycles (VERA-length), instead of ~10 at the old 8 MHz controller clock.
//     The r65c02 core mishandles LONG stalls; short stalls (like VERA's) work.
//   * The CPU interface lives in the cpu_clk (8 MHz) domain.
//
// CDC: a 1-bit toggle req/ack handshake.  The access parameters are CAPTURED
// into cpu-domain registers (lat_*) at the posedge that flips req_tgl, so the
// fast domain never samples the live CPU bus.  Only the req/ack toggles need
// 2-FF synchronizers; lat_* are stable from req until the next req (which
// cannot happen before ack returns).
//
// WRITES: the r65c02 IGNORES rdy during its write-on-bus cycle (the FSM
// write states exit unconditionally), so a write can NEVER be held with
// ready=0 -- and it can even arrive while a previous access is still in
// flight (e.g. `sta abs,x`: the 65C02 issues a dummy READ of the target,
// which starts an SDRAM access, and the write cycle lands during `waiting`).
// So every CPU write cycle is captured unconditionally into a small FIFO
// (same idea as the BUG3 SPI write-FIFO) and drained to SDRAM in order;
// reads are only issued/completed once the FIFO is empty, which also keeps
// read-after-write ordering.  If the FIFO fills up (SDRAM init/refresh),
// ready stalls the CPU GLOBALLY at its next rdy-honoring cycle -- every
// write is followed by an opcode fetch within a few cycles, so the FIFO
// bound holds.
//
// NOTE: requires the MiSTer SDRAM module present.  Needs PLL outclk_3 = 100 MHz.
//============================================================================
module ext_ram_sdram (
    input  logic        clk,          // cpu_clk (8 MHz)  -- CPU interface domain
    input  logic        sdram_clk,    // 100 MHz          -- SDRAM controller domain
    input  logic        reset_n,      // async; released in cpu_clk domain

    // CPU interface  (clk domain)
    input  logic        cs,           // access targets this module (HiRAM or cart)
    input  logic        we,           // 1 = write
    input  logic [24:0] byte_addr,    // full SDRAM byte address (parent muxes windows)
    input  logic  [7:0] wr_data,
    output logic  [7:0] rd_data,
    output logic        ready,        // 1 = CPU may proceed; 0 = stall (cpu_rdy)

    // Loader port (sdram_clk domain = the hps_io ioctl domain): streams a
    // cart image into SDRAM while the CPU is held in reset.  ld_busy is meant
    // for ioctl_wait; an 8-deep FIFO (busy at 4) absorbs the few words the
    // HPS may still push after wait asserts.
    input  logic        ld_wr,        // 1-cycle pulse: write ld_data @ ld_addr
    input  logic [24:0] ld_addr,
    input  logic  [7:0] ld_data,
    output logic        ld_busy,

    // Backup read port (sdram_clk domain) -- jyv 2026-07-07, cart save-back:
    // lowest-priority single-byte reads for rtl/cart_backer.sv's sector
    // prefetch.  bk_rd pulse -> bk_ack pulse with bk_rdata valid.
    input  logic        bk_rd,
    input  logic [24:0] bk_addr,
    output logic  [7:0] bk_rdata,
    output logic        bk_ack,

    // CPU-write snoop (sdram_clk domain): 1-cycle pulse when a CPU write is
    // consumed by the controller, with its address -- cart_backer uses it
    // for sector dirty tracking.  Loader/backup traffic never pulses this.
    output logic        wr_snoop,
    output logic [24:0] wr_snoop_addr,

    // SDRAM chip pins (forwarded from sdram.v)
    output logic [12:0] SDRAM_A,
    inout  wire  [15:0] SDRAM_DQ,
    output logic  [1:0] SDRAM_BA,
    output logic        SDRAM_nCS,
    output logic        SDRAM_nWE,
    output logic        SDRAM_nRAS,
    output logic        SDRAM_nCAS,
    output logic        SDRAM_CKE,
    output logic        SDRAM_CLK,
    output logic        SDRAM_DQML,
    output logic        SDRAM_DQMH
);
    localparam [9:0] REFRESH_INTERVAL = 10'd750;  // < 780 = 7.8 us @ 100 MHz
    localparam [3:0] CYCLE_LEN        = 4'd9;      // > sdram.v's 8-state cycle
    localparam [9:0] INIT_WAIT_LEN    = 10'd400;   // > 31*8 self-init cycles

    // ---- fast-domain reset: async assert, sync deassert into sdram_clk ----
    logic [1:0] rstf_sync;
    always_ff @(posedge sdram_clk or negedge reset_n)
        if (!reset_n) rstf_sync <= 2'b00;
        else          rstf_sync <= {rstf_sync[0], 1'b1};
    wire reset_n_f = rstf_sync[1];

    // ---- CDC toggles + captured read data (cross the clock boundary) ----
    logic       req_tgl;        // cpu_clk -> sdram_clk : flips to request an access
    logic       ack_tgl;        // sdram_clk -> cpu_clk : flips when access is done
    logic [7:0] rd_data_f;      // sdram_clk : captured read byte (stable when ack flips)

    // ---- cpu-domain capture of the access params (see header: write safety) ----
    logic        lat_we;
    logic [24:0] lat_addr;
    logic  [7:0] lat_wdata;

    // ---- fast-domain snapshot of the captured params ----
    logic       sd_ce, sd_we_l, sd_refresh;
    logic [24:0] acc_addr;
    logic  [7:0] acc_wdata;
    wire   [7:0] sd_dout;
    wire   [1:0] sd_dqm;

    sdram u_sdram (
        .sd_addr (SDRAM_A),
        .sd_data (SDRAM_DQ),
        .sd_ba   (SDRAM_BA),
        .sd_cs   (SDRAM_nCS),
        .sd_we   (SDRAM_nWE),
        .sd_ras  (SDRAM_nRAS),
        .sd_cas  (SDRAM_nCAS),
        .sd_clk  (SDRAM_CLK),
        .sd_dqm  (sd_dqm),

        .init    (~reset_n_f),
        .clk     (sdram_clk),
        .addr    (acc_addr),
        .din     (acc_wdata),
        .dout    (sd_dout),
        .refresh (sd_refresh),
        .ce      (sd_ce),
        .we      (sd_we_l)
    );
    assign SDRAM_CKE  = 1'b1;
    assign SDRAM_DQML = sd_dqm[0];
    assign SDRAM_DQMH = sd_dqm[1];

    // ======================================================================
    // CPU domain (clk, 8 MHz).
    //
    // WRITES: captured unconditionally into a 4-deep FIFO at the write-on-bus
    //   posedge (the only edge where the write's params are valid -- the CPU
    //   ignores rdy on writes and may fire one while `waiting` is set, see
    //   header).  Drained to SDRAM in order, ahead of any read.
    //
    // READS: "serve once, deliver once" (consume-clear).  served_valid is set
    //   when the fast domain returns a read and cleared the moment the CPU
    //   consumes it (the single ready=1 delivery cycle), so the NEXT in-window
    //   access always stalls and re-serves -- back-to-back accesses where cs
    //   never falls (sequential fetches when EXECUTING from a cart bank,
    //   page-crossing indexed reads, HiRAM<->cart transitions) are correct by
    //   construction, with NO address compare and NO negedge logic anywhere
    //   near the rdy cone.  This board has a history of punishing clever
    //   rdy-path logic that RTL sim can't see (BUG2/BUG3) -- keep this dumb.
    // ======================================================================
    logic [32:0] wfifo [0:3];                  // {byte_addr, wr_data}
    logic  [1:0] wf_rd, wf_wr;
    logic  [2:0] wf_cnt;
    // wpush: capture a write ONCE, on its committing (unstalled) cycle.
    // The r65c02 blows through rdy on writes, so its write cycle is always
    // exactly one clock and (because every 65C02 write is followed by >=3
    // rdy-honoring cycles) can never coincide with wf_hi=1 -- the gate is a
    // no-op for it.  The P65C816 HONORS rdy on writes: when wf_hi stalls it
    // mid-write it freezes holding cs/we/addr/data, and an ungated push
    // would re-push that same write every clock -- wf_wr laps wf_rd (writes
    // LOST), wf_cnt wraps 7->0 (FIFO declared empty with entries stranded).
    // See sim/tb_wfifo.v for the repro.  During a write, ready == ~wf_hi,
    // so gating on ~wf_hi is exactly "push at the commit edge".
    wire         wf_nonempty = (wf_cnt != 3'd0);
    wire         wf_hi       = (wf_cnt >= 3'd2);   // headroom for 2 more
    wire         wpush       = cs & we & ~wf_hi;

    logic [1:0]  ack_s;
    logic        ack_d, waiting, served_valid, wr_since_issue;
    wire         ack_edge  = (ack_s[1] != ack_d);
    wire         need_read = cs & ~we & ~served_valid;
    wire         wpop      = ~waiting & wf_nonempty;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            req_tgl <= 1'b0; ack_s <= 2'b00; ack_d <= 1'b0;
            waiting <= 1'b0; served_valid <= 1'b0; wr_since_issue <= 1'b0;
            lat_we <= 1'b0; lat_addr <= 25'h0; lat_wdata <= 8'h00;
            wf_rd <= 2'd0; wf_wr <= 2'd0; wf_cnt <= 3'd0;
            rd_data <= 8'h00;
        end else begin
            ack_s <= {ack_s[0], ack_tgl};
            ack_d <= ack_s[1];

            // consume-clear: during the delivery cycle (served_valid=1, CPU
            // unstalled on a read) the CPU takes rd_data at this posedge; the
            // next in-window access must re-serve.  (If some OTHER stall held
            // the CPU this cycle, the access is simply served again -- a few
            // wasted fast cycles, never wrong data.)
            if (cs & ~we & served_valid) served_valid <= 1'b0;
            if (!cs)                     served_valid <= 1'b0;

            // issue the next access: FIFO writes first, then the bus read
            if (!waiting) begin
                if (wf_nonempty) begin
                    lat_we               <= 1'b1;
                    {lat_addr, lat_wdata} <= wfifo[wf_rd];
                    wf_rd                <= wf_rd + 2'd1;
                    req_tgl              <= ~req_tgl;
                    waiting              <= 1'b1;
                    wr_since_issue       <= 1'b0;
                end else if (need_read) begin
                    lat_we         <= 1'b0;
                    lat_addr       <= byte_addr;
                    req_tgl        <= ~req_tgl;
                    waiting        <= 1'b1;
                    wr_since_issue <= 1'b0;
                end
            end

            // capture a CPU write cycle (this edge is the only chance -- the
            // r65c02 ignores rdy on writes)
            if (wpush) begin
                wfifo[wf_wr]   <= {byte_addr, wr_data};
                wf_wr          <= wf_wr + 2'd1;
                served_valid   <= 1'b0;                // write shadows any delivery
                wr_since_issue <= 1'b1;
            end

            wf_cnt <= wf_cnt + (wpush ? 3'd1 : 3'd0) - (wpop ? 3'd1 : 3'd0);

            // Fast domain finished the in-flight access (set LAST: a read
            // completing this edge opens next cycle's delivery window).
            //
            // DELIVERY GATE (the "136K HIGH RAM" bug): the r65c02 blows
            // through its DUMMY-read cycles ignoring rdy (`sta abs,x` reads
            // the target before writing it), so a read can complete AFTER the
            // CPU has moved on.  Blindly caching that data poisons the next
            // read: ramtas' `sta $a000,x / cmp $a000,x` consumed the dummy
            // read's PRE-WRITE value and undercounted the banks.  Deliver
            // ONLY if (a) no write was pushed while this read was in flight,
            // and (b) the CPU is still parked on this exact read.  Otherwise
            // drop the data -- the read (if still wanted) simply re-serves.
            // Both checks feed a register, never the combinational rdy cone.
            if (waiting && ack_edge) begin
                waiting <= 1'b0;
                if (!lat_we && !wr_since_issue
                    && cs && !we && (byte_addr == lat_addr)) begin
                    served_valid <= 1'b1;
                    rd_data      <= rd_data_f;
                end
            end
        end
    end

    // ---- stall generation ----
    //
    // COMBINATIONAL-LOOP CONSTRAINT: ready must NOT compare live CPU address
    // bits.  The pre-BUG3 form did, closing a loop cpu_a -> ready -> cpu_rdy
    // -> CPU rdy_i -> a_o -> cpu_a ("105-node combinational loop" /
    // metastable 2-try boot).  Here ready = the same proven shape as phase f:
    // a plain ~cs term plus REGISTERED flags only (served_valid, waiting,
    // wf_cnt) -- consume-clear needs no compare at all.
    //
    // ~wf_hi stalls the CPU GLOBALLY (even with cs low) when the write FIFO
    // gets 2+ deep, so it can never overflow: the CPU adds at most one more
    // write before its next rdy-honoring (read/fetch) cycle parks it.
    //
    // `| we`: a WRITE cycle is always ready -- the FIFO captures it at the
    // bus edge, so there is nothing to wait for.  The r65c02 ignored rdy on
    // writes anyway (identical behavior), but a CPU that HONORS rdy on
    // writes (the P65C816's RDY_IN is a global clock enable) would deadlock
    // without this: the held write re-pushes wpush each cycle, keeps
    // served_valid cleared, and ready would stay 0 forever.
    assign ready = (~cs | we | (served_valid & ~waiting)) & ~wf_hi;

    // ======================================================================
    // SDRAM domain (sdram_clk, 100 MHz) -- init / refresh / loader / access.
    //
    // Loader FIFO: ioctl bytes land here (same clock as hps_io).  Service
    // rate is ~1 byte / 10-19 cycles (~8 MB/s), faster than the HPS file
    // stream, so ld_busy (-> ioctl_wait) rarely asserts.  REFRESH has top
    // priority in S_IDLE so a seconds-long download cannot starve it and
    // lose the HiRAM contents.  Loader completions do NOT flip ack_tgl --
    // that toggle belongs to the CPU handshake.
    // ======================================================================
    typedef enum logic [1:0] {S_INIT, S_IDLE, S_ACC, S_RFSH} st_t;
    st_t        state;
    logic [9:0] init_cnt, rfsh_cnt;
    logic [3:0] cyc;
    logic [1:0] req_s;
    logic       req_d, req_pending;
    logic       acc_is_ld;
    logic       acc_is_bk, bk_pending;

    logic [32:0] ldfifo [0:7];                 // {ld_addr, ld_data}
    logic  [2:0] ldf_rd, ldf_wr;
    logic  [3:0] ldf_cnt;
    wire         ldf_nonempty = (ldf_cnt != 4'd0);
    assign       ld_busy      = (ldf_cnt >= 4'd4);   // headroom for in-flight words

    wire refresh_due = (rfsh_cnt == REFRESH_INTERVAL);
    wire ld_pop      = (state == S_IDLE) & ~refresh_due & ldf_nonempty;

    always_ff @(posedge sdram_clk or negedge reset_n_f) begin
        if (!reset_n_f) begin
            state <= S_INIT; init_cnt <= 10'd0; rfsh_cnt <= 10'd0; cyc <= 4'd0;
            sd_ce <= 1'b0; sd_refresh <= 1'b0; sd_we_l <= 1'b0; rd_data_f <= 8'h00;
            req_s <= 2'b00; req_d <= 1'b0; req_pending <= 1'b0; ack_tgl <= 1'b0;
            acc_addr <= 25'h0; acc_wdata <= 8'h0; acc_is_ld <= 1'b0;
            acc_is_bk <= 1'b0; bk_pending <= 1'b0; bk_ack <= 1'b0;
            bk_rdata <= 8'h00; wr_snoop <= 1'b0; wr_snoop_addr <= 25'h0;
            ldf_rd <= 3'd0; ldf_wr <= 3'd0; ldf_cnt <= 4'd0;
        end else begin
            sd_ce      <= 1'b0;   // ce/refresh are single-cycle triggers
            sd_refresh <= 1'b0;
            bk_ack     <= 1'b0;   // 1-cycle pulses
            wr_snoop   <= 1'b0;
            if (bk_rd) bk_pending <= 1'b1;

            // synchronize the CPU's request toggle, latch a pending request
            req_s <= {req_s[0], req_tgl};
            req_d <= req_s[1];
            if (req_s[1] != req_d) req_pending <= 1'b1;

            if (!refresh_due) rfsh_cnt <= rfsh_cnt + 10'd1;

            // loader FIFO push (hps_io ioctl domain = this domain)
            if (ld_wr) begin
                ldfifo[ldf_wr] <= {ld_addr, ld_data};
                ldf_wr         <= ldf_wr + 3'd1;
            end
            ldf_cnt <= ldf_cnt + (ld_wr ? 4'd1 : 4'd0) - (ld_pop ? 4'd1 : 4'd0);

            case (state)
                S_INIT: begin
                    init_cnt <= init_cnt + 10'd1;
                    if (init_cnt == INIT_WAIT_LEN) state <= S_IDLE;
                end
                S_IDLE: begin
                    if (refresh_due) begin
                        sd_refresh <= 1'b1; rfsh_cnt <= 10'd0; cyc <= 4'd0; state <= S_RFSH;
                    end else if (ldf_nonempty) begin
                        sd_ce                 <= 1'b1;
                        sd_we_l               <= 1'b1;
                        {acc_addr, acc_wdata} <= ldfifo[ldf_rd];
                        ldf_rd                <= ldf_rd + 3'd1;
                        acc_is_ld             <= 1'b1;
                        acc_is_bk             <= 1'b0;
                        cyc                   <= 4'd0;
                        state                 <= S_ACC;
                    end else if (req_pending) begin
                        // snapshot the CAPTURED (cpu-domain, stable) params
                        sd_ce       <= 1'b1;
                        sd_we_l     <= lat_we;
                        acc_addr    <= lat_addr;
                        acc_wdata   <= lat_wdata;
                        acc_is_ld   <= 1'b0;
                        acc_is_bk   <= 1'b0;
                        cyc         <= 4'd0;
                        req_pending <= 1'b0;
                        state       <= S_ACC;
                        // dirty snoop for cart_backer (CPU writes only)
                        wr_snoop      <= lat_we;
                        wr_snoop_addr <= lat_addr;
                    end else if (bk_pending) begin
                        // backup read: lowest priority, single byte
                        sd_ce      <= 1'b1;
                        sd_we_l    <= 1'b0;
                        acc_addr   <= bk_addr;
                        acc_is_ld  <= 1'b0;
                        acc_is_bk  <= 1'b1;
                        cyc        <= 4'd0;
                        bk_pending <= 1'b0;
                        state      <= S_ACC;
                    end
                end
                S_ACC: begin
                    cyc <= cyc + 4'd1;
                    if (cyc == CYCLE_LEN) begin
                        if (acc_is_bk) begin
                            bk_rdata <= sd_dout;
                            bk_ack   <= 1'b1;
                        end else if (!acc_is_ld) begin
                            rd_data_f <= sd_dout;
                            ack_tgl   <= ~ack_tgl;   // completion to CPU domain
                        end
                        state <= S_IDLE;
                    end
                end
                S_RFSH: begin
                    cyc <= cyc + 4'd1;
                    if (cyc == CYCLE_LEN) state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
