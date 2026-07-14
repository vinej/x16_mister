# X16 SD-read-path ModelSim testbenches

## Peripheral testbench (`./run.sh periph`, 2026-07-05)
`tb_periph.v`: unit tests for the peripheral batch using the exact shipped
modules — `i2s_rx` (rtl/x16_periph.sv) against VERA's real `dacif.v`
serializer (recovers exact 16-bit samples incl. $8000/$7FFF), `snes_pad`
driven with the KERNAL r49 `joystick_scan` protocol, and
`ps2_to_smc_bridge` key+mouse interleaving (atomic FF S DX DY packet).
Result 2026-07-05: **ALL PASS** (caught a 1-bit I2S frame-alignment error:
the resync detection consumes the pad bit, so capture is at cnt==23).
Also `rtc_x16` (MCP7940N @ $6F) via a bit-banged I2C master doing
KERNAL-shaped transactions: time regs from a fed hps_rtc value, NVRAM
multi-byte write + auto-increment read, $5F checksum guard, clock set via
I2C (stops HPS syncs), and the 23:59:59 Feb-28 (non-leap) -> Mar-1 00:00:00
rollover cascade.  (`CLK_HZ` is a parameter so the TB ticks fast.)

## Bank / cart testbench (`./run.sh bank`, 2026-07-05)
`tb_bank.v` + `banktest.s` + `sdram_sim.v`: real r65c02 + REAL
`rtl/ext_ram_sdram.sv` (behavioral `sdram` byte controller) + real
`lowram_bram`/`ext_ram_bram`, glued with a replica of x16.sv's decode.
Verifies the 256-ROM-bank / cart feature: HiRAM SDRAM banks (phase-f
regression), cart banks 32-255 write/read/isolation ($C000/$FFF0), indexed
fill+verify, EXECUTING code from a cart bank, banks 16-31 = $FF, bank-0 ROM
intact, plus a direct dump of the SDRAM model's cart region.  Also drives
the **cart loader port** (`ld_*` = the hps_io ioctl stream): the TB streams a
"CX16" signature + patterns into bank 32 while the CPU is held in reset
(honoring `ld_busy` like `ioctl_wait`), then the program verifies it from the
CPU side (t7).  Result 2026-07-05: **ALL PASS**.  This TB also caught the
dropped `sta abs,x` write (dummy-read puts the handshake into `waiting`, the
write lands mid-flight) that motivated the write FIFO in ext_ram_sdram.sv.

CPU-in-the-loop simulation of the X16 guest-SD read path (Phase h, BUG 1 =
"SD read returns 0x00"). Real `r65c02` VHDL core + the `x16.sv` cpu_clk SPI
master + the real `sd_card.sv` register/prefetch logic, running a ca65 65C02
program that issues CMD17 and reads a 512-byte sector.

## Files
- `sdtest.s` / `sdtest.cfg` -- ca65 program: select card, CMD17 block 0, wait
  0xFE token (generous ~200*256 poll like the real ROM), read 512B to
  $0400..$05FF, set `done`($00)=0xAA. Zero page: `done`=$00, `ptr`=$02/$03,
  `cnt`=$04/$05 (ptr deliberately NOT at $00 -- see gotcha below).
- `sd_card_sim.sv` -- sim-friendly copy of `sys/sd_card.sv` (params hoisted,
  altsyncram -> plain reg-array, reset/`initial` zeroing to avoid X-prop).
- `tb_cpu.v` -- baseline: **mock** hps_io (dense 1-byte/clk fill).
- `tb_cpu_hps.v` -- **accurate** hps_io fill: faithful `b_wr[2:0]` pipeline
  (write pulse +2 cyc, addr +1 cyc after write), `sd_ack` asserted at the
  command byte and HELD for the whole block, bytes spaced `BYTE_SPACING` cyc
  apart, `sd_ack` dropped `ACK_DROP` cyc after the last byte. Params overridable
  with `vsim -gBYTE_SPACING=.. -gACK_DROP=..`.
- `run.sh` -- build core + assemble + run (`./run.sh both|hps|mock|sweep`).

## Result (2026-07-03)
Both TBs read **all 512 bytes correctly**, including bytes 450/510/511
(0C/55/AA). The accurate-hps_io model passes for every realistic parameter:
`BYTE_SPACING >= 2` and any `ACK_DROP` (0..8).

**Conclusions:**
1. The SD-read RTL (sd_card + prefetch + register interface + SPI master + CPU)
   is functionally correct. Confirmed independently of the fill model.
2. The theory that the real hps_io held-ack + spaced `b_wr` fill misaligns the
   sd_card prefetch banks (fill bank N vs read bank N-1) is **DISPROVEN in sim**:
   `sd_buf` increments 0->1 once after ack falls; the read always serialises the
   filled bank 0.
3. The tail-truncation theory (ack drops before the last `b_wr` flushes ->
   510/511 lost) does **not** reproduce: after the last byte there are always
   >= 2 cycles before ack can drop, and on real HW `io_enable` (which clears
   `sd_ack`) drops only after the SPI frame, i.e. many cycles later.
4. `BYTE_SPACING=1` (a byte every 2 cyc) is the ONLY failing case -- it violates
   the b_wr pipeline's 3-cycle minimum and is non-physical (HPS delivers bytes
   ~16+ cyc apart over SPI). Not a real-HW scenario.

=> The HW `0x00` (BUG 1) is NOT an RTL-logic bug. Like BUG 2 it must be a
synthesis/timing artifact invisible to behavioral RTL sim. Behavioral sim has
now exhausted its usefulness for BUG 1.

## Gotchas found while building this (were masking as "failures")
- The reader's `ptr` must NOT live at zero-page $00: the `done` marker also
  writes $00, and as `ptr` low byte counts up it transiently hits 0xAA at
  offset 170, tripping the TB's `while (ram[0]!=0xAA)` early -> premature dump
  after only 170 bytes. Fixed: `done`=$00, `ptr`=$02.
- Byte-450 lives at $05C2 ($0400+450), not $05D2. The old verify used the wrong
  address. Fixed; the TB now verifies all 512 bytes against `secbyte()`.
- The token-wait must be generous. With realistic fill latency (~8700 cyc at
  SPACING=16) a 40-try token poll expires before the token, so the CPU reads
  garbage. Fixed: ~200*256 poll retries.
