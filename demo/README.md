# VERA_2 bitmap-layer demos

Example programs for the SDRAM bitmap layer (`$9F60`–`$9F6F`). See
[`../vera_2.md`](../vera_2.md) for the full register spec. Both demos
feature-detect `$9F61`, so the same `.PRG` runs on the emulator **and** on real
hardware.

| Source | PRG | What it shows |
|---|---|---|
| `vera2fill.s` | `VERA2FILL.PRG` | Switch to 8bpp, fill the whole screen fast with the **blit** (doubling a 16-colour seed), wait for a key, return to BASIC. |
| `vera2blit.s` | `VERA2BLIT.PRG` | 8bpp gradient + 16 random **VERA sprites** + the **mouse** over it (passthru); **left-click** the gradient drops a message box (band saved to scratch via the blit), **click the box** to restore it exactly. |

`vera2demo.cfg` is the cc65 linker config both use (a minimal `$0801` PRG with a
BASIC `SYS` stub).

## Build

Needs [cc65](https://cc65.github.io/):

```
ca65 --cpu 65C02 vera2fill.s -o vera2fill.o
ld65 -C vera2demo.cfg vera2fill.o -o VERA2FILL.PRG

ca65 --cpu 65C02 vera2blit.s -o vera2blit.o
ld65 -C vera2demo.cfg vera2blit.o -o VERA2BLIT.PRG
```

## Run

**Emulator** (must be built with the `-bitmap2` device — see the core repo):

```
x16emu -bitmap2 -prg VERA2FILL.PRG -run
```

**Hardware** (X16-MiSTer): turn on **Bitmap Layer** in the OSD, copy the `.PRG`
to the SD card, then `LOAD"VERA2FILL.PRG"` / `RUN` (or load it however you
normally load programs).
