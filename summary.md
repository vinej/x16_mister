# X16 MiSTer Migration — Status & Next Steps

**Source project:** `c:\quartus\projects\x16_monitor` (Terasic C5G — working baseline).
**Target project:** this folder (`c:\quartus\projects\x16_mister`) — DE10-Nano, MiSTer, `5CSEBA6U23I7`.
**Goal:** port the working X16 implementation to MiSTer.

Original full plan: `C:\Users\jyv\.claude\plans\enchanted-orbiting-pinwheel.md`.

---

## Completed steps (a–d)

### a. Cleaned the MiSTer C64 tree to framework-only
Removed all C64-specific RTL, kept the MiSTer framework + utility RAM/SDRAM modules.

- **Deleted from `rtl/`:** `am29f040.sv`, `c1351.v`, `c1530.vhd`, `cartridge.v`, `cpu_6510.vhd`, `drv_overlay.sv`, `fpga64_*.vhd` (4 files), `mos6526.v`, `reu.v`, `rtcF83.sv`, `video_sync.vhd`, `video_vicII_656x.vhd`.
- **Deleted directories:** `rtl/iec_drive/`, `rtl/opl3/`, `rtl/roms/`, `rtl/sid/`, `rtl/t65/`.
- **Deleted root cruft:** `C64_Q13.{qpf,qsf,srf}`, `keymap.gif`, `releases/` (85 old C64 bitstreams).
- **Kept:** `sys/` (whole MiSTer framework), `rtl/sdram.v`, `rtl/dprom.vhd`, `rtl/spram.vhd`, `rtl/pll.qip` + `rtl/pll.v` + `rtl/pll/` (PLL IP — required by `sys/pll_q17.qip`, **frequencies need regeneration**).
- **Stub `c64.sv`** replaced full C64 top with empty `emu` module driving safe defaults.

### b. Renamed C64 → X16
- `C64.qpf` → `X16.qpf` (and `PROJECT_REVISION = "X16"`)
- `C64.qsf` → `X16.qsf`
- `C64.srf` → `X16.srf`
- `c64.sv`  → `x16.sv`
- `files.qip` updated to reference `x16.sv`
- `README.md` rewritten as a thin pointer to this `summary.md` + the plan.

### c. Copied 49 portable RTL files from x16_monitor
All wired via `files.qip` with library settings preserved (`r65c02_tc`) and the IKAOPM_pg shift-register quirk fix included.

| Group | Count | Path |
|---|---|---|
| R65C02 CPU (VHDL) | 8 | `rtl/cpu/r65c02/` |
| X16 glue (SV) | 7 | `rtl/rom_banks.sv`, `lowram_bram.sv`, `via65c22.sv`, `smc_x16.sv`, `smc_i2c_slave.sv`, `i2c_bb.sv`, `i2c_open_drain.sv` |
| IKAOPM YM2151 | 11 | `IKAOPM-main/src/` + `src/IKAOPM_modules/` |
| VERA | 23 | `vera/fpga/source/{top,main_ram,dpram,reset_sync,vram_if}.v` + `sim/`, `graphics/`, `video/`, `audio/`, `spi/` |

VERA files already have the Lattice→Quartus patches applied (per the memory note `project_vera_quartus_strip.md`).

### d. Created `x16.sv` (~430 lines)
Mirrors the structure of `c:\quartus\projects\x16_monitor\rtl\C5G_x16.sv` but drives MiSTer framework signals instead of C5G pins:

- PLL instantiation (existing `pll` module — **wrong frequencies**, see Issue #1)
- Per-domain reset synchronizers (`pix_reset_n`, `cpu_reset_n`, `aud_reset_n`)
- `hps_io` minimal instantiation (PS/2, ioctl, status)
- `r65c02_wrap` CPU with IRQ from VERA (sync'd to cpu_clk)
- Address decoder identical to C5G: kernal/io/vera/ym/via1/via2/hi_ram/lowram chip-selects
- `rom_banks` with `ROM_BANK` register at `$0001` + vector-page redirect to bank 0
- `lowram_bram` for `$0000-$9EFF`
- HiRAM stub (`ext_ram_data = 0`) — replaced in step f
- VERA `top` with full 4-cycle write / 2-cycle read-stall pipeline
- `IKAOPM` (YM2151) at `aud_mclk`
- Two `via65c22` instances; VIA1 PA bit-bangs internal I2C to SMC
- `smc_x16` with internal I2C bus; **uart_byte tied to 0** (step e wires PS/2)
- CPU data-in priority mux
- Video out → `VGA_R/G/B/HS/VS/DE`
- Audio out → `AUDIO_L/R` (IKAOPM only — VERA I2S deserializer deferred)
- Safe defaults for `DDRAM_*`, `SDRAM_*`, `SD_*`, `USER_OUT`, `LED_*`, `BUTTONS`, `ADC_BUS`

---

## Known issues to address before the bitstream will work

### Issue 1 — PLL frequencies are wrong (BLOCKER)
`rtl/pll.v` is the C64's PLL: outclk_0 ≈ 63 MHz, outclk_1 ≈ 63 MHz, outclk_2 ≈ 31.5 MHz.

X16 needs:
- `cpu_clk` ≈ 8 MHz (R65C02 + VIA + I2C)
- `pix_clk` ≈ 25.175 MHz (VERA VGA 640x480)
- `aud_mclk` ≈ 49.152 MHz (IKAOPM + I2S)

**Action:** open Quartus, Tools → IP Catalog → search "Altera PLL", open the existing `rtl/pll.qip`, edit the wizard to set the three output frequencies above, regenerate. Or create a new `pll_x16` and rewire `x16.sv` accordingly.

### Issue 2 — ROM image not present
`rtl/rom_banks.sv` line 48: `initial $readmemh("x16-rom-r49/build/x16/rom_compact.hex", mem);`

That file doesn't exist in this project. **Two options:**

**Option A (quick):** copy the hex over so we can boot before step g.
```powershell
$src = 'c:\quartus\projects\x16_monitor\x16-rom-r49\build\x16'
$dst = 'c:\quartus\projects\x16_mister\x16-rom-r49\build\x16'
New-Item -ItemType Directory -Force -Path $dst | Out-Null
Copy-Item -Force (Join-Path $src 'rom_compact.hex') (Join-Path $dst 'rom_compact.hex')
```

**Option B (preferred):** jump to step g and wire the HPS ioctl loader directly. Remove `$readmemh` from `rom_banks.sv` and add a write-port driven by `ioctl_*` from `hps_io`. Then load ROM from the MiSTer menu at runtime.

### Issue 3 — `hps_io` may need more tie-offs
First compile may complain about unconnected required inputs (`sd_lba`, `sd_blk_cnt`, `joystick_*_rumble`, `ps2_*_in`, `ps2_kbd_led_*`). Add empty tie-offs as needed.

### Issue 4 — VERA audio not in mix
Only IKAOPM is routed to `AUDIO_L/R`. VERA outputs serial I2S (`audio_lrck`, `audio_bck`, `audio_data`); a deserializer is needed to recover the 16-bit L/R samples and sum them with IKAOPM before driving `AUDIO_L/R`. Deferred; not blocking compile.

---

## Remaining steps (e, f, g, h)

### Step e — Bridge HPS PS/2 keyboard to `smc_x16`  ✅ DONE
- `rtl/ps2_to_smc_bridge.sv` watches `ps2_key[10:0]` from `hps_io` (strobe-toggle
  on `[10]`, pressed `[9]`, extended `[8]`, Set-2 scancode `[7:0]`) and emits the
  byte stream `smc_x16.uart_byte` expects (make: `{E0,}code`; break: `{E0,}F0,code`),
  one byte per cpu_clk cycle.  Matches `Scripts/send_input.py` (make/break at
  send_input.py:195-201).
- `hps_io` is clocked by `cpu_clk`, so `ps2_key` is already in-domain (no CDC).
- Wired `bridge → smc_x16.uart_byte/uart_byte_valid` in `x16.sv`; registered in `files.qip`.
- **TODO: verify** with a USB keyboard on the DE10-Nano (boot to `READY.`, type).

The X16 SMC byte protocol is documented in the header comment of `rtl/smc_x16.sv`.

### Step f — Replace SRAM with SDRAM (HiRAM)  ✅ DONE (needs HW verify)
- `rtl/ext_ram_sdram.sv` wraps `rtl/sdram.v` (MiST byte-access controller), clocked
  by **cpu_clk (8 MHz)** -- same domain as the CPU so there is NO CDC and NO extra
  PLL output.  HiRAM bank N, offset A -> SDRAM byte `{bank[7:0], addr[12:0]}` (low 2 MB).
- An access is a fixed ~9-cycle window; the controller's `ready` is ANDed into
  `cpu_rdy` to stall the CPU.  Auto-refresh issued every ~58 cpu_clk when idle.
- `ram_bank_r` widened to 8 bits (full 256 banks).  SDRAM_* driven from the wrapper.
- **REQUIRES the MiSTer SDRAM module to be physically present.**
- **Why this mattered for the keyboard:** the KERNAL decompresses the keyboard
  keymap into banked HiRAM (`ps2kbd.s`), then looks up every keystroke there.
  With HiRAM stubbed to 0, keycodes were delivered correctly (proven on hardware)
  but mapped to nothing -> no characters.  This is the real fix for Phase e's
  "keyboard does nothing" symptom, not a keyboard bug.

### Step g — HPS ROM loader + final MiSTer integration
- Modify `rom_banks.sv`: remove `$readmemh`, add an HPS-driven write port (`ioctl_addr`, `ioctl_data`, `ioctl_wr`, gated by `ioctl_download && ioctl_index == 1`).
- Hold CPU in reset while `ioctl_download` is asserted.
- Create an MRA file so the MiSTer menu can pick `rom.bin` from `/games/X16/`.
- Set `CONF_STR` core name to "X16" in `x16.sv`.

### Step h — Wire up the X16 guest SD card
The X16 talks to its SD card over SPI (the `vera/.../spi/` files).  Two possible
homes for that guest SD, depending on hardware:

**Hardware note:** the user's board is a **GMTech with TWO physical SD slots**
(not a stock single-slot DE10-Nano).  If the 2nd slot is wired to FPGA fabric,
**prefer routing VERA's SPI SD master directly to that slot's pins** so the X16
sees a *real* SD card — simplest and most authentic.  Verify the slot's wiring
(FPGA vs HPS) from the GMTech schematic/pinout before relying on this.

**Option A (real 2nd card, if FPGA-wired):** connect VERA SPI CS/SCK/MOSI/MISO to
the second slot's FPGA pins; add pin assignments in `X16.qsf`.  No HPS plumbing.

**Option B (virtual image, the portable fallback):** back the guest SD with a
disk-image file on the main microSD, exposed via `hps_io`'s virtual block device.
- Create `rtl/sd_spi_to_hps.sv`: terminate the VERA SPI master (CS/SCK/MOSI/MISO)
  and translate 512-byte block reads/writes into the `hps_io` `sd_*` protocol
  (`sd_lba`, `sd_rd[1:0]`, `sd_wr[1:0]`, `sd_buff_addr/dout/din/wr`, `sd_ack`).
- Honour `img_mounted` / `img_size`; report no-card when nothing is mounted.
- Add the disk image to `CONF_STR` (an `S0` mount slot) so the MiSTer menu can
  mount an X16 SD image (read **and** write, for SAVE).
- Alternative if SPI bridging is too fiddly: emulate the SD at the X16's SPI
  register level instead of bit-banging the wire protocol.
- Verify: `DOS"$"`, `LOAD`, and `SAVE` against a mounted image.

---

## Key file paths

| Purpose | Path |
|---|---|
| Top-level (Phase d) | `x16.sv` |
| Project | `X16.qsf`, `X16.qpf` |
| Source list | `files.qip` |
| Stub PLL (needs regen) | `rtl/pll.qip`, `rtl/pll.v`, `rtl/pll/pll_0002.v` |
| Reference top (read-only) | `c:\quartus\projects\x16_monitor\rtl\C5G_x16.sv` |
| ROM source (hex/bin) | `c:\quartus\projects\x16_monitor\x16-rom-r49\build\x16\rom_compact.hex` |
| VERA patches memo | `C:\Users\jyv\.claude\projects\c--quartus-projects-x16-monitor\memory\project_vera_quartus_strip.md` |
| Full plan | `C:\Users\jyv\.claude\plans\enchanted-orbiting-pinwheel.md` |

---

## First-compile checklist (what to do when you reopen this folder)

1. **Pick a strategy for the ROM** (Issue #2 above) — copy the hex over, OR jump to step g now.
2. **Regenerate the PLL** (Issue #1) — Tools → IP Catalog in Quartus.
3. **Attempt full compile:** `quartus_sh --flow compile X16.qpf` (or use Quartus GUI).
4. **Read the error log.** Most likely categories:
   - Missing `hps_io` input tie-offs → add them in `x16.sv`
   - `r65c02_tc` library missing → check `files.qip` library declarations
   - VERA pixel-clock timing slack negative → tune PLL output frequencies / phase
   - `$readmemh` file-not-found → ROM issue (see #2)
5. **Iterate.** Each fix should be one targeted edit, recompile, next.

When the bitstream compiles and loads on the DE10-Nano, the X16 should show its splash + `READY.` on HDMI but ignore keyboard input (that's step e). HiRAM POKEs will silently fail (that's step f).
