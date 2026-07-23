# Commander X16 for [MiSTer](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

Note: 128M SDRAM module is needed for the X16 core.

## version 1.3 :

**EXPERIMENTAL — "VERA2" bitmap layer.** An optional SDRAM-backed **640×480
4bpp / 8bpp** linear bitmap layer — the high-resolution mode VERA advertises but
its 128 KB of VRAM can't hold — mapped at `$9F60`–`$9F6F` and composited over
VERA. It is **off by default**; nothing changes for existing software unless you
turn it on.

**How to activate:** in the OSD, set **VERA2 Bitmap Layer → On**. A program then
drives the registers at `$9F60`–`$9F6F` (write pixels, load the palette, use the
SDRAM→SDRAM blit for fast fills / save-under). With the layer **Off** the core
behaves exactly as before.

* **Documentation / register spec** — [vera_2.md](vera_2.md)
* **Demos** — source (`.s`) + `.PRG` + a ready-to-mount SD image — [demo/](demo/)
* **Emulator that supports v1.3** — a fork of the X16 emulator adding the same
  layer via the `-bitmap2` flag, so you can develop and test VERA2 programs on a
  PC: **[github.com/vinej/x16-emulator](https://github.com/vinej/x16-emulator)**

**Fixed — cartridge banks 32–255 were writable with no cartridge present.** On
real X16 hardware those banks are whatever a physical cartridge provides (ROM /
RAM / flash); with nothing plugged in there is no RAM there to write. The core
previously treated all of banks 32–255 as writable RAM unconditionally, which
was wrong. They now default to **ROM** (CPU writes ignored), matching the
hardware and the emulator. If you want the old behavior — using the cartridge
space as extra writable RAM, or with **Mount Cart RAM** — set **OSD → Cart
Banks 32+ → RAM**. (The same behavior is available in the emulator fork with the
`-cartram` flag.)

## version 1.2 :

Add serial card emulation done by jnngill

## version 1.1 :

Fixed a lock-up (frozen boot, no flashing prompt) when relaunching the core with an SD image still mounted. The auto-remounted image could collide with the ROM's boot-time SD probe and wedge the virtual SD card; the guest SD is now kept deselected for ~1.3 s after every reset so the probe always runs cleanly. If you were working around this by remounting the image or toggling the CPU after each launch, that is no longer needed.

## version 1.0 : 
The published `.rbf` contains **no ROM** — the X16 ROM is not ours to
redistribute — so you must supply your own `boot1.rom` at runtime.

Get a 256 KB X16 ROM (all 16 banks) — build it from X16Community/x16-rom, or use the rom.bin / *.BIN from a Commander X16 / emulator release.

This core was coded by Claude Opus 4.8 and Claude Fable 5 My participation was
testing, debugging, thinking, analysing... but not coding.

This port of the Commander X16 to the MiSTer FPGA platform (DE10-Nano,
Cyclone V SoC) is derived from a first version developed for the Cyclone V GX
(Terasic C5G) starter kit — an old board bought a long, long time ago.

Even with AI, it took around 3-4 weeks to achieve the goal because we had many
strange bugs to resolve. It worked only after switching to Claude MAX with Fable 5. 

## Installation

| File                     | Description                   | Where to put it |
|--------------------------|-------------------------------|-----------------|
| `/rom/X16_20260722.rbf`  | version 1.3 no baked ROM      | `_Computer/`    |
| your own boot1.rom       | tested with R49               | `games/X16/`    |

(The published `.rbf` contains **no ROM** — the X16 ROM is not ours to
redistribute — so you must supply your own `boot1.rom` at runtime, see above.
`rom/rom.hex` is **not** included in this repo.)

### Rebuilding `rom.hex` (optional — personal baked-in ROM build)

If you would rather bake the ROM into the bitstream for your **own personal
build** (so the core boots with no `boot1.rom` on the SD), regenerate
`rom/rom.hex` from a 256 KB X16 ROM binary and recompile:

1. Get a **256 KB X16 ROM** (all 16 banks) — build it from
   [X16Community/x16-rom](https://github.com/X16Community/x16-rom), or use the
   `rom.bin` / `*.BIN` from a Commander X16 / emulator release.
2. Convert it to the `$readmemh` format the core expects (262144 lines, one
   byte per line):

   ```
   python scripts/rom_bin2hex.py path/to/rom.bin rom/rom.hex
   ```

   (Run it with no output path to write `rom/rom.hex` automatically.)
3. Enable the baked-in ROM: in `X16.qsf`, uncomment
   `set_global_assignment -name VERILOG_MACRO "BAKE_ROM=1"` — this defines the
   `BAKE_ROM` switch that gates the `$readmemh` in `rtl/rom_banks.sv`.
4. Recompile in Quartus. The resulting `.rbf` now has the ROM baked in.

Keep `BAKE_ROM` commented and `rom/rom.hex` out of any repo you publish — a
bitstream with the ROM baked in redistributes the ROM indirectly.

* Bonus: `scripts/mister_send_input.py <mister-ip>` streams your PC keyboard
  to the MiSTer over the network (nothing to install on the MiSTer;
  PAUSE/BREAK quits, F12 is forwarded to the OSD).

## Peripherals

* **Audio**: YM2151 (FM) and VERA PSG + PCM are both mixed into the MiSTer
  audio output (HDMI and analog).
* **Game controllers**: two MiSTer gamepads appear as SNES controllers on
  the X16 joystick ports 1 and 2 (KERNAL `joystick_get`); map the buttons in
  the OSD ("Define joystick buttons": A, B, X, Y, L, R, Select, Start).
* **Mouse**: a mouse connected to the MiSTer works as the X16 PS/2 mouse
  (`MOUSE ON` in BASIC shows the pointer).
* **Interrupts**: VERA, VIA1, VIA2 and YM2151 IRQs are all wired to the CPU,
  so software using VIA timer or YM timer interrupts runs correctly.
* **SMC**: software reset ($02) and NMI ($03) requests over I2C work, and
  the X16 activity LED drives the MiSTer USER LED.
* **RTC + NVRAM**: the MCP7940N (I2C $6F) is implemented, fed by the
  MiSTer's Linux wall clock — `DATE`/`TIME` are correct at boot, the clock
  ticks on its own, and it can be set with `clock_set_date_time` (a set
  clock is not overwritten by the next MiSTer sync). The 64-byte NVRAM
  (KERNAL settings + checksum) works but is volatile: it survives resets,
  not power cycles.

## Banking

* **RAM banks** ($0000): 256 banks of 8 KB at $A000–$BFFF — 2 MB HiRAM
  (banks 0–1 in on-chip BRAM, the rest in SDRAM).
* **ROM banks** ($0001): the full 8-bit register is implemented, giving 256
  banks of 16 KB at $C000–$FFFF:
  * banks **0–15** — the system ROM (`boot1.rom` / baked-in image);
  * banks **16–31** — unpopulated system-ROM space, reads `$FF` like empty
    flash, writes are ignored;
  * banks **32–255** — the X16 **cartridge space** (SDRAM-backed, 3.5 MB).
    By default these read/execute as **ROM** (CPU writes ignored), matching
    real hardware where nothing is present without a cartridge. Set **OSD →
    Cart Banks 32+ → RAM** to make them CPU-writable — then software can use
    the space as extra RAM, self-modify loaded images, or persist it with
    **Mount Cart RAM**. Either way you can fill them from the OSD with **Load
    Cart** or at core start with `boot2.rom` (the loader writes regardless of
    the ROM/RAM setting).

The vector page $FFFA–$FFFF always reads from ROM bank 0, so interrupts keep
hitting the KERNAL handlers whatever bank is selected.


## License

MIT — see [LICENSE](LICENSE). Third-party components (MiSTer framework, VERA,
IKAOPM, the R65C02 core, and the X16 ROM) remain under their own licenses, as
listed in the LICENSE file.

Thanks!
