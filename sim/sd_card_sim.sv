// 
// sd_card.v
//
// Copyright (c) 2014 Till Harbaum <till@harbaum.org>
// Copyright (c) 2015-2018 Sorgelig
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the Lesser GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// http://elm-chan.org/docs/mmc/mmc_e.html
//
/////////////////////////////////////////////////////////////////////////

//
// Made module syncrhronous. Total code refactoring. (Sorgelig)
// clk_spi must be at least 2 x sck for proper work.

module sd_card #(parameter WIDE = 0, OCTAL=0, AW=8, DW=7, SZ=1, SW=0)
(
	input             clk_sys,
	input             reset,

	input             sdhc,
	input             img_mounted,
	input      [63:0] img_size,

	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg        sd_wr,
	input             sd_ack,

	input      [AW:0] sd_buff_addr,
	input      [DW:0] sd_buff_dout,
	output     [DW:0] sd_buff_din,
	input             sd_buff_wr,

	// SPI interface
	input             clk_spi,

	input             ss,
	input             sck,
	input      [SW:0] mosi,
	output reg [SW:0] miso,

	// --- debug taps for the x16 SD read-path probe (spare outputs, harmless) ---
	output     [DW:0] dbg_bufdout,   // byte being serialised onto MISO (port-B read)
	output      [8:0] dbg_bufptr,    // its offset within the 512-byte block
	output      [1:0] dbg_spibuf,    // which of the 4 prefetch banks is being read
	output      [1:0] dbg_sdbuf,     // which bank the HPS is currently writing
	output reg  [2:0] dbg_wrstate,   // write state machine (6 = WR_STATE_BUSY zeros MISO)
	output reg  [2:0] dbg_rdstate,   // read state machine
	output reg  [7:0] dbg_mem510     // DIRECT read of buffer RAM[510] (fixed addr)
);


localparam DATA_TOKEN_CMD25    = 8'hfc;
localparam STOP_TRAN           = 8'hfd;
localparam DATA_TOKEN          = 8'hfe;
localparam WRITE_DATA_RESPONSE = 8'he5;

// number of bytes to wait after a command before sending the reply
localparam NCR = 5+3; // 5 bytes are required (command length)

localparam RD_STATE_IDLE       = 0;
localparam RD_STATE_START      = 1;
localparam RD_STATE_WAIT_IO    = 2;
localparam RD_STATE_SEND_TOKEN = 3;
localparam RD_STATE_SEND_DATA  = 4;
localparam RD_STATE_WAIT_M     = 5;

localparam WR_STATE_IDLE       = 0;
localparam WR_STATE_EXP_DTOKEN = 1;
localparam WR_STATE_RECV_DATA  = 2;
localparam WR_STATE_RECV_CRC0  = 3;
localparam WR_STATE_RECV_CRC1  = 4;
localparam WR_STATE_SEND_DRESP = 5;
localparam WR_STATE_BUSY       = 6;

localparam PREF_STATE_IDLE     = 0;
localparam PREF_STATE_RD       = 1;
localparam PREF_STATE_FINISH   = 2;

	// -----------------------------------------------------------------------
	// X16 fix: replaced the altsyncram with a plain true-dual-port RAM.  In this
	// core clk_sys and clk_spi are the SAME clock (cpu_clk), and the altsyncram's
	// registered dual-port read was serving stale/zero data at the sector tail
	// (byte 510/511 -> 0x00) despite the buffer being filled correctly.  Single
	// 512-byte bank (the X16 reads one sector at a time), 1-cycle registered read.
	// Port A = HPS fill/read (clk_sys); Port B = SPI serialise/RECV (clk_spi).
	// -----------------------------------------------------------------------
	// ONE always block / ONE shared memory (clk_sys == clk_spi == cpu_clk here).  MULTI-
	// bank (2048 = 4 x 512): the HPS fill writes bank sd_buf while the SPI read serialises
	// bank spi_buf -- separate banks, so a concurrent prefetch can't clobber the byte
	// being read (the single-bank version collided -> timing-random zeros at the tail).
	// (RAM block moved below the buffer_ptr/sd_buf declarations for the strict sim parser)
reg [26:0] csd_size;
reg        csd_sdhc;
always @(posedge clk_sys) begin
	if (img_mounted) begin
		csd_sdhc <= sdhc;
		if (sdhc) begin
			csd_size[0]     <= 0;
			csd_size[22:1]  <= img_size[40:19]; // in 512K units
			csd_size[26:23] <= 0;
		end
		else begin
			csd_size[2:0]   <= 7; // C_SIZE_MULT
			csd_size[14:3]  <= 12'b101101101101;
			csd_size[26:15] <= img_size[29:18]; // in 256K units ((2**(C_SIZE_MULT+2))*512)
		end
	end
end

wire [127:0] CSD = {1'b0,csd_sdhc,6'h00,8'h0e,8'h00,8'h32,8'h5b,8'h59,6'h00,csd_size,7'h7f,8'h80,8'h0a,8'h40,8'h40,8'hf1};
wire [127:0] CID = {8'hcd,8'hc7,8'h00,8'h93,8'h6f,8'h2f,8'h73,8'h00,8'h00,8'h44,8'h32,8'h38,8'h34,8'h00,8'h00,8'h3e};

reg  [8:0] buffer_ptr;
reg  [7:0] buffer_din;
reg  [7:0] buffer_dout;
reg        buffer_wr;

reg  [1:0] sd_buf, spi_buf;

reg [7:0] sdbuf_mem [0:2047];
reg [7:0] sd_buff_din_r;
assign    sd_buff_din = sd_buff_din_r;
always @(posedge clk_sys) begin
	if      (sd_ack & sd_buff_wr) sdbuf_mem[{sd_buf, sd_buff_addr[8:0]}] <= sd_buff_dout;
	else if (buffer_wr)           sdbuf_mem[{spi_buf, buffer_ptr}]       <= buffer_din;
	sd_buff_din_r <= sdbuf_mem[{sd_buf,  sd_buff_addr[8:0]}];
	buffer_dout   <= sdbuf_mem[{spi_buf, buffer_ptr}];
	dbg_mem510    <= sdbuf_mem[{spi_buf, 9'd510}];
end

// debug taps (see port list)
assign dbg_bufdout = buffer_dout;
assign dbg_bufptr  = buffer_ptr;
assign dbg_spibuf  = spi_buf;
assign dbg_sdbuf   = sd_buf;

reg  [6:0] sbuf; integer dbg_nrx = 0;
reg  [2:0] bit_cnt;

wire       last_bit = &bit_cnt || OCTAL;
wire [7:0] ibuf = OCTAL ? mosi : {sbuf,mosi[0]};

always @(posedge clk_spi) begin
	reg  [2:0] read_state;
	reg  [2:0] write_state;
	reg  [1:0] pref_state;
	reg  [5:0] cmd;
	reg        cmd55;
	reg [39:0] reply;
	reg  [3:0] byte_cnt;
	reg        old_sck;
	reg  [2:0] ack;
	reg  [2:0] wait_m_cnt;
	reg [31:0] arg;

	ack[1:0] <= {ack[0],sd_ack};
	if(ack[1] == ack[0]) ack[2] <= ack[1];

	if(~ack[2] &  ack[1]) {sd_rd,sd_wr} <= 0;
	if( ack[2] & ~ack[1]) begin
		sd_buf <= sd_buf + 1'd1;
		sd_lba <= sd_lba + 1;
	end

	buffer_wr <= 0;

	if(reset) begin
		bit_cnt     <= 0;
		byte_cnt    <= '1;
		miso        <= '1;
		cmd         <= 0;
		sd_wr 		<= 0;
		sd_rd 		<= 0;
		read_state  <= RD_STATE_IDLE;
		write_state <= WR_STATE_IDLE;
		pref_state  <= PREF_STATE_IDLE;
		reply <= 0; arg <= 0; ack <= 0; wait_m_cnt <= 0; old_sck <= 0; cmd55 <= 0;
	end
	else begin

		if(buffer_wr) begin
			if(~&buffer_ptr) buffer_ptr <= buffer_ptr + 1'd1;
			else begin
				spi_buf <= spi_buf + 1'd1;
				sd_wr <= 1;
			end
		end

		case(pref_state)
			PREF_STATE_IDLE:
				if(((sd_buf - spi_buf) <= 1) && (read_state != RD_STATE_IDLE) && (cmd == 17 || cmd == 18)) begin
					sd_rd <= 1;
					pref_state <= PREF_STATE_RD;
				end

			PREF_STATE_RD:
				if(read_state == RD_STATE_IDLE) begin
					pref_state <= PREF_STATE_IDLE;
				end
				else if(ack[2] & ~ack[1]) begin
					pref_state <= (cmd == 18) ? PREF_STATE_IDLE : PREF_STATE_FINISH;
				end

			PREF_STATE_FINISH:
				if(read_state == RD_STATE_IDLE) begin
					pref_state <= PREF_STATE_IDLE;
				end
		endcase

		old_sck <= sck;
		if(ss) begin
			bit_cnt  <= 0;
			byte_cnt <= '1;
			miso     <= '1;
		end
		else if(old_sck & ~sck) begin
			miso <= '1;				// default: send 1's (busy/wait)
			if(byte_cnt >= NCR) {miso,reply} <= {reply, {SZ{1'b1}}};

			// ---------- read state machine processing -------------
			case(read_state)
				RD_STATE_IDLE: ;

				RD_STATE_START: begin
					if(byte_cnt == NCR && last_bit) read_state <= (cmd == 9 || cmd == 10) ? RD_STATE_SEND_TOKEN : RD_STATE_WAIT_IO;
				end

				// waiting for io controller to return data
				RD_STATE_WAIT_IO: begin
					if(sd_buf != spi_buf && last_bit) read_state <= RD_STATE_SEND_TOKEN;
				end

				// send data token
				RD_STATE_SEND_TOKEN: begin
					miso <= DATA_TOKEN[~bit_cnt -:SZ];
					if(last_bit) begin
						read_state <= RD_STATE_SEND_DATA;   // next: send data
						buffer_ptr <= 0;
					end
				end

				// send data
				RD_STATE_SEND_DATA: begin

					miso <= (cmd == 9) ? CSD[{buffer_ptr[3:0],~bit_cnt} -:SZ] : (cmd == 10) ? CID[{buffer_ptr[3:0],~bit_cnt} -:SZ] : buffer_dout[~bit_cnt -:SZ];

					if(last_bit) begin

						// sent 512 sector data bytes?
						if(cmd == 17 && &buffer_ptr) read_state <= RD_STATE_IDLE;
						else if(cmd == 18 && &buffer_ptr) begin
							read_state <= RD_STATE_WAIT_M;
							wait_m_cnt <= 0;
						end

						// sent 16 cid/csd data bytes?
						else if((cmd == 9 || cmd == 10) && &buffer_ptr[3:0]) read_state <= RD_STATE_IDLE;

						// not done yet -> trigger read of next data byte
						else buffer_ptr <= buffer_ptr + 1'd1;
					end
				end

				RD_STATE_WAIT_M: begin
					if(last_bit) begin
						wait_m_cnt <= wait_m_cnt + 1'd1;
						if(&wait_m_cnt) begin
							spi_buf <= spi_buf + 1'd1;
							read_state <= RD_STATE_WAIT_IO;
						end
					end
				end
			endcase

			// ------------------ write support ----------------------
			// send write data response
			if(write_state == WR_STATE_SEND_DRESP) miso <= WRITE_DATA_RESPONSE[~bit_cnt -:SZ];

			// busy after write until the io controller sends ack
			if(write_state == WR_STATE_BUSY) miso <= 0;
		end
		else if(~old_sck & sck) begin

			sbuf[6:0] <= {sbuf[5:0],mosi[0]};
			bit_cnt <= bit_cnt + SZ[2:0];

			if(last_bit) begin
				if (dbg_nrx < 20) begin
					$display("[SD] rx ibuf=%02x byte_cnt=%0d cmd=%0d rdstate=%0d ss=%b", ibuf, byte_cnt, cmd, read_state, ss);
					dbg_nrx <= dbg_nrx + 1;
				end
				// finished reading one byte
				// byte counter runs against 15 byte boundary
				if(~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;

				// byte_cnt > 6 -> complete command received
				// first byte of valid command is 01xxxxxx
				// don't accept new commands once a write or read command has been accepted
				if(byte_cnt > 5 &&
						((write_state == WR_STATE_IDLE && read_state == RD_STATE_IDLE && ibuf[7:6] == 1) || 
						(read_state != RD_STATE_IDLE && ibuf == 8'h4c))) begin
					byte_cnt <= 0;
					cmd      <= ibuf[5:0];
					cmd55    <= (cmd == 55); // set cmd55 flag if previous command was 55
					if(ibuf[5:0] == 12) read_state <= RD_STATE_IDLE;
				end

				// parse additional command bytes
				if(byte_cnt == 0) arg[31:24] <= ibuf;
				if(byte_cnt == 1) arg[23:16] <= ibuf;
				if(byte_cnt == 2) arg[15:8]  <= ibuf;
				if(byte_cnt == 3) arg[7:0]   <= ibuf;

				// last byte (crc) received, evaluate
				if(byte_cnt == 4) begin

					// default:
					reply <= 40'h04FFFFFFFF;        // illegal command

					case(cmd)
						// CMD0: GO_IDLE_STATE
						 0: reply[39:32] <= 1;       // ok, busy

						// CMD1: SEND_OP_COND
						 1: reply[39:32] <= 0;

						// CMD8: SEND_IF_COND (V2 only)
						 8: reply <= 40'h01000001AA; // ok, busy

						// CMD9: SEND_CSD
						 9,
						// CMD10: SEND_CID
						10: begin
								reply[39:32] <= 0;
								read_state <= RD_STATE_START;
							end

						// CMD12: STOP_TRANSMISSION 
						12: reply[39:32] <= 0;
						
						// CMD13: SEND_STATUS 
						13: reply[39:24] <= 16'h0000;

						// CMD16: SET_BLOCKLEN
						16: reply[39:32] <= (arg == 512) ? 8'h00 : 8'h40; // we only support a block size of 512

						// CMD17: READ_SINGLE_BLOCK
						17,
						// CMD18: READ_MULTIPLE
						18: begin
								reply[39:32] <= 0;
								read_state <= RD_STATE_START;
								spi_buf <= 0;
								sd_buf <= 0;
								sd_lba <= csd_sdhc ? arg : {9'd0, arg[31:9]};
							end

						// ACMD23:  SET_WR_BLK_ERASE_COUNT
						23: reply[39:32] <= 0;

						// CMD24: WRITE_BLOCK
						24,
						// CMD25: WRITE_MULTIPLE
						25: begin
								reply[39:32] <= 0;
								write_state <= WR_STATE_EXP_DTOKEN;  // expect data token
								spi_buf <= 0;
								sd_buf <= 0;
								sd_lba <= csd_sdhc ? arg : {9'd0, arg[31:9]};
							end

						// ACMD41: APP_SEND_OP_COND
						41: if(cmd55) reply[39:32] <= 0;    // ok, not busy

						// CMD55: APP_COND
						55: reply[39:32] <= 1;    // ok, busy

						// CMD58: READ_OCR
						58: reply <= { 8'h00, 1'b1, csd_sdhc, 30'd0 }; // bit 30 = 1 -> high capacity card

						// CMD59: CRC_ON_OFF
						59: reply[39:32] <= 0;
					endcase
				end

					// ---------- handle write -----------
				case(write_state)
					// do nothing in idle state
					WR_STATE_IDLE: ;

					// waiting for data token
					WR_STATE_EXP_DTOKEN: begin
						buffer_ptr <= 0;
						if(cmd == 24) begin
							if(ibuf == DATA_TOKEN)       write_state <= WR_STATE_RECV_DATA;
						end
						else begin
							if(ibuf == DATA_TOKEN_CMD25) write_state <= WR_STATE_RECV_DATA;
							if(ibuf == STOP_TRAN)        write_state <= WR_STATE_IDLE;
						end
					end

					// transfer 512 bytes
					WR_STATE_RECV_DATA: begin
						// push one byte into local buffer
						buffer_wr  <= 1;
						buffer_din <= ibuf;

						// all bytes written?
						if(&buffer_ptr) write_state <= WR_STATE_RECV_CRC0;
					end

					// transfer 1st crc byte
					WR_STATE_RECV_CRC0:
						write_state <= WR_STATE_RECV_CRC1;

					// transfer 2nd crc byte
					WR_STATE_RECV_CRC1:
						write_state <= WR_STATE_SEND_DRESP;

					// send data response
					WR_STATE_SEND_DRESP:
						write_state <= WR_STATE_BUSY;

					// wait for io controller to accept data
					WR_STATE_BUSY:
						if(spi_buf == sd_buf) write_state <= (cmd == 25) ? WR_STATE_EXP_DTOKEN : WR_STATE_IDLE;
				endcase
			end
		end
	end

	// debug mirrors of the (block-local) state machines
	dbg_wrstate <= write_state;
	dbg_rdstate <= read_state;
end

integer zzi;
initial begin
    buffer_ptr=0; sd_buf=0; spi_buf=0; sbuf=0; sd_buff_din_r=0;
    for (zzi=0; zzi<2048; zzi=zzi+1) sdbuf_mem[zzi]=0;
end
endmodule
