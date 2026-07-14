//============================================================================
// rtc_x16.sv  -  MCP7940N RTC + 64-byte SRAM, I2C slave at $6F.
//
// Fills the X16's second I2C device (the first is the SMC at $42): the
// KERNAL reads registers 0-6 (BCD sec/min/hour/wkday/date/month/year) at
// boot for DATE/TIME, and keeps boot settings (keymap, screen mode) with a
// checksum in the chip's SRAM -- r49 uses $40-$5F of the $20-$5F SRAM.
//
// Time source: hps_io's RTC[64:0] (the MiSTer/Linux wall clock, "MSM6242B
// layout": BCD bytes {ctrl,wday,year,month,date,hour,min,sec}, bit 64
// toggles on each ~60 s update).  Between HPS updates a local 1 Hz tick
// keeps the seconds running (full BCD calendar cascade incl. leap years,
// valid 2000-2099).  A CPU write to any time register (KERNAL
// clock_set_date_time) takes over: HPS re-syncs are ignored from then on
// so the user's clock is not stomped a minute later.
//
// Register map (pointer auto-increments, wraps $5F -> $00, MCP behavior):
//   $00 RTCSEC   {ST, sec BCD}   ST=1 -> oscillator (our tick) runs
//   $01 RTCMIN   min BCD
//   $02 RTCHOUR  hour BCD (24 h)
//   $03 RTCWKDAY {OSCRUN, 0, VBATEN=1, wkday 1-7}
//   $04 RTCDATE  date BCD
//   $05 RTCMTH   month BCD
//   $06 RTCYEAR  year BCD (00-99 = 2000-2099)
//   $07-$1F      control/trim scratch (stored, no function)
//   $20-$5F      SRAM, battery-backed on real HW; here volatile, zeroed --
//                except $5F (the KERNAL's settings checksum) = $FF so the
//                all-zero contents can never pass the checksum and the
//                KERNAL falls back to defaults until real settings are saved.
//
// I2C engine: same synchronizer/START/STOP/ACK scheme as smc_i2c_slave.sv,
// plus data storage, read auto-increment, and proper master-NACK handling
// (release SDA so the master's STOP can complete).
//============================================================================
module rtc_x16 #(
    parameter logic [6:0] SLAVE_ADDR = 7'h6F,
    parameter integer     CLK_HZ     = 8_000_000   // clk ticks per RTC second
) (
    input  logic        clk,            // cpu_clk (8 MHz)
    input  logic        reset_n,
    input  logic        sda_bus,        // bus state (post-OR of all drivers)
    input  logic        scl_bus,
    output logic        sda_drive_low,  // 1 = we pull SDA low

    input  logic [64:0] hps_rtc,        // hps_io RTC (sdram_clk domain; [64]
                                        // toggles per update, payload static)

    // NVRAM backer port (jyv 2026-07-07, reworked after Quartus 276001:
    // a second write port in another clock domain is not synthesizable).
    // Everything here is cpu_clk domain; rtl/nvram_backer.sv owns the
    // cross-domain plumbing with its own dual-clock buffers.
    //   nv_we/addr/wdata : restore-time writes into the store (muxed into
    //                      the single write port; the I2C FSM write wins,
    //                      the backer sees nv_snoop_we and retries)
    //   nv_snoop_*       : mirror of every I2C-FSM write to the store, so
    //                      the backer keeps its save shadow coherent
    //   nv_dirty_toggle  : flips on every I2C-FSM write -> autosave timer
    // TBs that don't care may leave these unconnected (inputs float ->
    // nv_we never true, port inert).
    input  logic        nv_we,
    input  logic  [6:0] nv_addr,
    input  logic  [7:0] nv_wdata,
    output logic        nv_snoop_we,
    output logic  [6:0] nv_snoop_addr,
    output logic  [7:0] nv_snoop_data,
    output logic        nv_dirty_toggle
);
    // ======================================================================
    // Time registers + 1 Hz tick + HPS sync
    // ======================================================================
    logic [7:0] r_sec, r_min, r_hour, r_date, r_month, r_year;  // BCD
    logic [2:0] r_wkday;                                        // 1-7
    logic       st = 1'b1;              // RTCSEC bit 7 (oscillator enable)
    logic       cpu_set = 1'b0;         // CPU owns the clock; ignore HPS

    // hps_rtc toggle: 2-FF sync, payload sampled after the edge (static
    // for ~60 s around each toggle -- same pattern as ps2_key/ps2_mouse).
    logic [2:0] rtctgl_s = 3'b000;
    always_ff @(posedge clk) rtctgl_s <= {rtctgl_s[1:0], hps_rtc[64]};
    wire hps_update = rtctgl_s[2] != rtctgl_s[1];

    // BCD helpers
    function automatic logic [7:0] bcd_inc(input logic [7:0] v);
        bcd_inc = (v[3:0] == 4'd9) ? {v[7:4] + 4'd1, 4'd0} : v + 8'd1;
    endfunction
    // year % 4 == 0 (2000-2099: every %4 year is a leap year)
    wire leap = (({1'b0, r_year[5:4]} + r_year[1:0]) & 2'b11) == 2'b00;
    // days in current month, BCD
    logic [7:0] dim;
    always_comb begin
        case (r_month)
            8'h04, 8'h06, 8'h09, 8'h11: dim = 8'h30;
            8'h02:                      dim = leap ? 8'h29 : 8'h28;
            default:                    dim = 8'h31;
        endcase
    end

    logic [22:0] div1hz = 23'd0;
    wire         sec_tick = (div1hz == CLK_HZ - 1);

    // I2C write strobes into the time registers (from the engine below)
    logic       twr;            // 1-cycle: write t_wdata to time reg twaddr
    logic [2:0] twaddr;
    logic [7:0] twdata;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // (power-up defaults; a HPS sync normally lands within a minute)
            r_sec <= 8'h00; r_min <= 8'h00; r_hour <= 8'h00; r_wkday <= 3'd1;
            r_date <= 8'h01; r_month <= 8'h01; r_year <= 8'h00;
            st <= 1'b1; cpu_set <= 1'b0; div1hz <= 23'd0;
        end else begin
            div1hz <= sec_tick ? 23'd0 : div1hz + 23'd1;

            if (twr) begin
                cpu_set <= 1'b1;
                case (twaddr)
                    3'd0: begin
                        r_sec  <= {1'b0, twdata[6:0]};
                        st     <= twdata[7];
                        div1hz <= 23'd0;   // MCP-like: writing SEC restarts
                    end                    // the divider chain
                    3'd1: r_min   <= twdata;
                    3'd2: r_hour  <= twdata & 8'h3F;
                    3'd3: r_wkday <= twdata[2:0];
                    3'd4: r_date  <= twdata;
                    3'd5: r_month <= twdata & 8'h1F;
                    default: r_year <= twdata;
                endcase
            end else if (hps_update && !cpu_set) begin
                r_sec   <= hps_rtc[7:0];
                r_min   <= hps_rtc[15:8];
                r_hour  <= hps_rtc[23:16];
                r_date  <= hps_rtc[31:24];
                r_month <= hps_rtc[39:32];
                r_year  <= hps_rtc[47:40];
                r_wkday <= hps_rtc[50:48] + 3'd1;     // Linux 0-6 -> 1-7
                st      <= 1'b1;
            end else if (sec_tick && st) begin
                if (r_sec != 8'h59) r_sec <= bcd_inc(r_sec);
                else begin
                    r_sec <= 8'h00;
                    if (r_min != 8'h59) r_min <= bcd_inc(r_min);
                    else begin
                        r_min <= 8'h00;
                        if (r_hour != 8'h23) r_hour <= bcd_inc(r_hour);
                        else begin
                            r_hour  <= 8'h00;
                            r_wkday <= (r_wkday == 3'd7) ? 3'd1 : r_wkday + 3'd1;
                            if (r_date != dim) r_date <= bcd_inc(r_date);
                            else begin
                                r_date <= 8'h01;
                                if (r_month != 8'h12) r_month <= bcd_inc(r_month);
                                else begin
                                    r_month <= 8'h01;
                                    r_year  <= (r_year == 8'h99) ? 8'h00
                                                                 : bcd_inc(r_year);
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    // ======================================================================
    // Control scratch + SRAM backing store ($07-$5F -> mem[0..88])
    //
    // TEXTBOOK sync-write/sync-read RAM in its OWN always block.  The first
    // version wrote mem[] from inside the async-reset FSM block below;
    // Quartus RAM-extracted it anyway (Critical Warning 127005 on the
    // build) -- a pattern where synthesis and RTL sim can legitimately
    // diverge.  The registered read costs one cpu_clk cycle, invisible next
    // to the multi-microsecond I2C bit times (reg_ptr is stable for at
    // least a full SCL period before any byte is served).
    // ======================================================================
    logic [7:0] mem [0:88];
    initial begin
        integer i;
        for (i = 0; i < 89; i = i + 1) mem[i] = 8'h00;
        mem[88] = 8'hFF;   // reg $5F: settings checksum -> force mismatch
    end

    logic [7:0] reg_ptr;
    logic       mem_we;          // driven combinationally by the FSM below
    logic [7:0] mem_wdata;
    logic [7:0] mem_rdata;
    wire  [6:0] mem_addr = reg_ptr[6:0] - 7'd7;

    // plain `always`, not always_ff: mem also has an `initial` preload.
    // Single write port, single clock: the I2C FSM write wins over a
    // same-cycle backer restore write (the backer watches nv_snoop_we and
    // retries -- see nvram_backer.sv).
    always @(posedge clk) begin
        if (mem_we)      mem[mem_addr] <= mem_wdata;
        else if (nv_we)  mem[nv_addr]  <= nv_wdata;
        mem_rdata <= mem[mem_addr];
    end

    assign nv_snoop_we   = mem_we;
    assign nv_snoop_addr = mem_addr;
    assign nv_snoop_data = mem_wdata;

    initial nv_dirty_toggle = 1'b0;
    always @(posedge clk)
        if (mem_we) nv_dirty_toggle <= ~nv_dirty_toggle;

    // read mux for the current register pointer
    logic [7:0] rd_data;
    always_comb begin
        case (reg_ptr)
            8'h00:   rd_data = {st, r_sec[6:0]};
            8'h01:   rd_data = r_min;
            8'h02:   rd_data = r_hour;
            8'h03:   rd_data = {2'b00, st, 1'b0, 1'b1, r_wkday};  // OSCRUN,VBATEN
            8'h04:   rd_data = r_date;
            8'h05:   rd_data = r_month;
            8'h06:   rd_data = r_year;
            default: rd_data = (reg_ptr <= 8'h5F) ? mem_rdata : 8'h00;
        endcase
    end

    wire [7:0] ptr_next = (reg_ptr == 8'h5F) ? 8'h00 : reg_ptr + 8'd1;

    // ======================================================================
    // I2C slave engine (per smc_i2c_slave.sv, + storage and NACK handling)
    // ======================================================================
    logic sda_s1, sda_s2, sda_s3;
    logic scl_s1, scl_s2, scl_s3;
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sda_s1 <= 1'b1; sda_s2 <= 1'b1; sda_s3 <= 1'b1;
            scl_s1 <= 1'b1; scl_s2 <= 1'b1; scl_s3 <= 1'b1;
        end else begin
            sda_s1 <= sda_bus; sda_s2 <= sda_s1; sda_s3 <= sda_s2;
            scl_s1 <= scl_bus; scl_s2 <= scl_s1; scl_s3 <= scl_s2;
        end
    end
    wire sda      = sda_s2;
    wire scl      = scl_s2;
    wire scl_rise = scl_s2 & ~scl_s3;
    wire scl_fall = ~scl_s2 & scl_s3;
    wire start_cond = (~sda_s2 & sda_s3) & scl;   // SDA falls, SCL high
    wire stop_cond  = (sda_s2 & ~sda_s3) & scl;   // SDA rises, SCL high

    typedef enum logic [3:0] {
        S_IDLE, S_ADDR, S_ADDR_ACK,
        S_REG_RX, S_REG_ACK,
        S_WRDATA, S_WRDATA_ACK,
        S_RDDATA, S_RDDATA_ACK
    } state_t;

    state_t     state;
    logic [7:0] shift, rd_shift;
    logic [3:0] bitcnt;
    logic       rw_bit;

    // plain `always`, not always_ff: `mem` also has an `initial` preload
    // (the $5F=$FF checksum guard), and always_ff forbids a second driver.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_IDLE; shift <= 8'h00; rd_shift <= 8'h00;
            bitcnt <= 4'h0; rw_bit <= 1'b0; reg_ptr <= 8'h00;
            sda_drive_low <= 1'b0;
            twr <= 1'b0; twaddr <= 3'd0; twdata <= 8'h00;
        end else begin
            twr <= 1'b0;                    // 1-cycle strobe

            if (stop_cond) begin
                state <= S_IDLE;
                sda_drive_low <= 1'b0;
            end
            else if (start_cond) begin      // START / repeated START
                state  <= S_ADDR;
                bitcnt <= 4'h0;
                shift  <= 8'h00;
                sda_drive_low <= 1'b0;
            end
            else case (state)
                S_IDLE: ;

                S_ADDR: if (scl_rise) begin
                    shift  <= {shift[6:0], sda};
                    bitcnt <= bitcnt + 4'h1;
                    if (bitcnt == 4'h7) begin
                        if (shift[6:0] == SLAVE_ADDR) begin
                            rw_bit <= sda;
                            state  <= S_ADDR_ACK;
                        end else state <= S_IDLE;
                    end
                end

                S_ADDR_ACK: begin
                    if (scl_fall && !sda_drive_low) sda_drive_low <= 1'b1;
                    else if (scl_fall && sda_drive_low) begin
                        sda_drive_low <= 1'b0;
                        bitcnt <= 4'h0;
                        shift  <= 8'h00;
                        if (rw_bit) begin
                            // first read byte: present it now (this IS the
                            // SCL-low window before its first bit)
                            rd_shift      <= rd_data;
                            reg_ptr       <= ptr_next;
                            sda_drive_low <= ~rd_data[7];
                            bitcnt        <= 4'h1;
                            state         <= S_RDDATA;
                        end else state <= S_REG_RX;
                    end
                end

                S_REG_RX: if (scl_rise) begin
                    shift  <= {shift[6:0], sda};
                    bitcnt <= bitcnt + 4'h1;
                    if (bitcnt == 4'h7) begin
                        reg_ptr <= {shift[6:0], sda};
                        state   <= S_REG_ACK;
                    end
                end

                S_REG_ACK: begin
                    if (scl_fall && !sda_drive_low) sda_drive_low <= 1'b1;
                    else if (scl_fall && sda_drive_low) begin
                        sda_drive_low <= 1'b0;
                        bitcnt <= 4'h0;
                        shift  <= 8'h00;
                        state  <= S_WRDATA;
                    end
                end

                S_WRDATA: if (scl_rise) begin
                    shift  <= {shift[6:0], sda};
                    bitcnt <= bitcnt + 4'h1;
                    if (bitcnt == 4'h7) begin
                        // commit {shift[6:0], sda} to reg_ptr, advance ptr
                        // (mem writes go through the mem_we strobe below --
                        //  the RAM block must stay free of this async-reset
                        //  process for clean inference)
                        if (reg_ptr <= 8'h06) begin
                            twr    <= 1'b1;
                            twaddr <= reg_ptr[2:0];
                            twdata <= {shift[6:0], sda};
                        end
                        reg_ptr <= ptr_next;
                        state   <= S_WRDATA_ACK;
                    end
                end

                S_WRDATA_ACK: begin
                    if (scl_fall && !sda_drive_low) sda_drive_low <= 1'b1;
                    else if (scl_fall && sda_drive_low) begin
                        sda_drive_low <= 1'b0;
                        bitcnt <= 4'h0;
                        state  <= S_WRDATA;
                    end
                end

                // shift out rd_shift, MSB first; bits change on SCL low
                S_RDDATA: if (scl_fall) begin
                    if (bitcnt == 4'h8) begin
                        sda_drive_low <= 1'b0;   // free the master's ACK slot
                        state         <= S_RDDATA_ACK;
                    end else begin
                        sda_drive_low <= ~rd_shift[7 - bitcnt[2:0]];
                        bitcnt        <= bitcnt + 4'h1;
                    end
                end

                S_RDDATA_ACK: begin
                    if (scl_fall) sda_drive_low <= 1'b0;  // release for master
                    if (scl_rise) begin
                        if (sda) state <= S_IDLE;         // master NACK: done
                        else begin                        // ACK: next byte
                            rd_shift <= rd_data;
                            reg_ptr  <= ptr_next;
                            bitcnt   <= 4'h0;
                            state    <= S_RDDATA;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // SRAM write strobe: fires on the same edge the FSM commits a data byte
    // for pointers $07-$5F (time regs $00-$06 go through twr instead).
    // Every operand is a register or a synchronized bus bit, so this is a
    // clean synchronous write-enable for the RAM block above.
    assign mem_we    = (state == S_WRDATA) && scl_rise && (bitcnt == 4'h7)
                     && (reg_ptr > 8'h06) && (reg_ptr <= 8'h5F);
    assign mem_wdata = {shift[6:0], sda};

endmodule
