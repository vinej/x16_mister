#!/usr/bin/env bash
# X16 SD-read-path ModelSim testbench runner.
# Builds the r65c02 VHDL core, assembles the ca65 SD-read program, and runs
# either the mock-hps_io baseline (tb_cpu) or the accurate-hps_io TB (tb_cpu_hps).
#
# Usage:
#   ./run.sh            # build everything, run both TBs
#   ./run.sh hps        # run only the accurate-hps_io TB
#   ./run.sh sweep      # accurate TB parameter sweep (BYTE_SPACING / ACK_DROP)
set -e

MS=/c/intelFPGA_lite/17.0/modelsim_ase/win32aloem
CC65=/c/Emulator/cc65/bin
RTL="$(cd "$(dirname "$0")/../rtl/cpu/r65c02" && pwd)"
export PATH="$MS:$CC65:$PATH"
cd "$(dirname "$0")"

echo "=== assemble sdtest.s + sdwedge.s (ca65) ==="
ca65 --cpu 65C02 sdtest.s -o sdtest.o
ld65 -C sdtest.cfg sdtest.o -o sdtest.rom
od -An -v -tx1 sdtest.rom | tr -s ' ' '\n' | grep -v '^$' > sdtest.hex
ca65 --cpu 65C02 sdwedge.s -o sdwedge.o
ld65 -C sdtest.cfg sdwedge.o -o sdwedge.rom
od -An -v -tx1 sdwedge.rom | tr -s ' ' '\n' | grep -v '^$' > sdwedge.hex

echo "=== compile r65c02 VHDL core ==="
vlib work    >/dev/null 2>&1 || true
vlib r65c02_tc >/dev/null 2>&1 || true
vmap r65c02_tc r65c02_tc >/dev/null 2>&1
# leaf entities first; SKIP r65c02_tc_config.vhd (direct entity binding -> vcom-1184)
vcom -quiet -work r65c02_tc \
     "$RTL/regbank_axy.vhd" "$RTL/reg_pc.vhd" "$RTL/reg_sp.vhd" \
     "$RTL/fsm_intnmi.vhd" "$RTL/fsm_execution_unit.vhd" "$RTL/core.vhd" "$RTL/r65c02_tc.vhd"
vcom -quiet -work work "$RTL/r65c02_wrap.vhd"

# MJoergen 65c02 (rtl/cpu/mj65c02): the OSD-selectable 65C02.  Ported to
# STRICT VHDL-93 (std_logic_unsigned, no out-port reads, no port-map
# expressions) because Quartus Standard's 2008 support is partial; vcom -93
# here PROVES 93-cleanliness before Quartus sees it.
# Order: leaves -> datapath -> microcode -> control ->
# core -> wrapper (fmt/debug: excised from the vendored core).  The core
# goes into its OWN library (entity names alu/pc/... clash with the
# P65C816's in work -- VHDL is case-insensitive); only the wrapper is in
# work and references mj65c02.cpu_65c02.
compile_mj() {
    MJ="$(cd "$(dirname "$0")/../rtl/cpu/mj65c02" && pwd)"
    vlib mj65c02 >/dev/null 2>&1 || true
    vmap mj65c02 mj65c02 >/dev/null 2>&1
    vcom -quiet -93 -work mj65c02 \
         "$MJ/datapath/alu.vhd" "$MJ/datapath/ar.vhd" "$MJ/datapath/hi.vhd" \
         "$MJ/datapath/lo.vhd" "$MJ/datapath/mr.vhd" "$MJ/datapath/pc.vhd" \
         "$MJ/datapath/sp.vhd" "$MJ/datapath/sr.vhd" "$MJ/datapath/xr.vhd" \
         "$MJ/datapath/yr.vhd" "$MJ/datapath/zp.vhd" "$MJ/datapath/datapath.vhd" \
         "$MJ/control/microcode_6502.vhd" "$MJ/control/microcode_65c02.vhd" \
         "$MJ/control/control.vhd" "$MJ/cpu_65c02.vhd"
    vcom -quiet -93 -work work "$MJ/mj65c02_wrap.vhd"
}

echo "=== compile testbenches ==="
vlog -quiet -sv ../rtl/spi_sd_master.sv sd_card_sim.sv tb_cpu.v tb_cpu_hps.v tb_wedge.v tb_write.v

case "${1:-both}" in
  mock) TBS="tb_cpu" ;;
  hps)  TBS="tb_cpu_hps" ;;
  both) TBS="tb_cpu tb_cpu_hps" ;;
  write)
    # BUG3: SAVE/write path. sdwrite.s = faithful R49 CMD24 flow (2nd write =
    # the FAST no-poll loop).  Passes with the edge-triggered write-FIFO master
    # (rtl/spi_sd_master.sv); the old inline level-strobe master fails it.
    ca65 --cpu 65C02 sdwrite.s -o sdwrite.o
    ld65 -C sdtest.cfg sdwrite.o -o sdwrite.rom
    od -An -v -tx1 sdwrite.rom | tr -s ' ' '\n' | grep -v '^$' > sdwrite.hex
    ca65 --cpu 65C02 sdwrite2.s -o sdwrite2.o
    ld65 -C sdtest.cfg sdwrite2.o -o sdwrite2.rom
    od -An -v -tx1 sdwrite2.rom | tr -s ' ' '\n' | grep -v '^$' > sdwrite2.hex
    vlog -quiet -sv ../rtl/spi_sd_master.sv ../rtl/ext_ram_bram.sv sd_card_sim.sv tb_write.v tb_write2.v
    vsim -c -do "run -all; quit -f" tb_write 2>&1 | grep -E "CFG|TBC|MISMATCH"
    vsim -c -gW_LEAD=500000 -do "run -all; quit -f" tb_write 2>&1 | grep -E "ALL OK|FAILED|TIMEOUT"
    # v2: data sourced from the REAL ext_ram_bram (HiRAM sector_buffer @ $B9C6)
    vsim -c -do "run -all; quit -f" tb_write2 2>&1 | grep -E "TBC|MISMATCH"
    exit 0 ;;
  bank816)
    # Same bank/cart test, but with the P65C816 CPU (65C816 branch): proves
    # the SDRAM handshake (write FIFO, consume-clear, `| we` ready term)
    # against a CPU that HONORS rdy on writes.  banktest.s is 65C02 code =
    # emulation mode.
    ca65 --cpu 65C02 banktest.s -o banktest.o
    ld65 -C banktest.cfg banktest.o -o banktest.rom
    od -An -v -tx1 banktest.rom | tr -s ' ' '\n' | grep -v '^$' > banktest.hex
    C816="$(cd "$(dirname "$0")/../rtl/cpu/65C816_x16" && pwd)"
    vcom -quiet -2008 -work work \
         "$C816/P65816_pkg.vhd" "$C816/BCDAdder.vhd" "$C816/AddSubBCD.vhd" \
         "$C816/ALU.vhd" "$C816/AddrGen.vhd" "$C816/MCode.vhd" \
         "$C816/P65C816.vhd" "$C816/p65c816_wrap.vhd"
    vlog -quiet -sv ../rtl/ext_ram_sdram.sv ../rtl/ext_ram_bram.sv \
         ../rtl/lowram_bram.sv sdram_sim.v tb_bank.v
    vsim -c -gCPU816=1 -do "run -all; quit -f" tb_bank 2>&1 | grep -E "TBC|MEM|LD|Error|Fatal"
    exit 0 ;;
  bankmj)
    # Same bank/cart test on the mj65c02 (OSD-selectable 65C02): SDRAM
    # handshake with a CPU whose ce_i freezes writes too ('816 semantics).
    ca65 --cpu 65C02 banktest.s -o banktest.o
    ld65 -C banktest.cfg banktest.o -o banktest.rom
    od -An -v -tx1 banktest.rom | tr -s ' ' '\n' | grep -v '^$' > banktest.hex
    compile_mj
    vlog -quiet -sv ../rtl/ext_ram_sdram.sv ../rtl/ext_ram_bram.sv \
         ../rtl/lowram_bram.sv sdram_sim.v tb_bank.v
    vsim -c -gCPUMJ=1 -do "run -all; quit -f" tb_bank 2>&1 | grep -E "TBC|MEM|LD|Error|Fatal"
    exit 0 ;;
  ym)
    # YM2151 audio fix smoke test: real IKAOPM at 25MHz//7 phiM with the
    # cpu->pix write handshake + status-read idle (x16.sv glue verbatim).
    vlog -quiet -sv "../IKAOPM-main/src/IKAOPM.v" ../IKAOPM-main/src/IKAOPM_modules/*.v tb_ym.v
    vsim -c -do "run -all; quit -f" tb_ym 2>&1 | grep -E "YM|Error|Fatal"
    exit 0 ;;
  wfifo)
    # ext_ram_sdram write-FIFO unit test: '816-held writes (repeat-push hunt)
    # + r65c02 blow-through writes (fix safety regression).
    vlog -quiet -sv ../rtl/ext_ram_sdram.sv sdram_sim.v tb_wfifo.v
    vsim -c -do "run -all; quit -f" tb_wfifo 2>&1 | grep -E "WF|Error|Fatal"
    exit 0 ;;
  fb)
    # Framebuffer 16-bit-tap read/stream path (SDRAM-backed bitmap layer):
    # sdram.v dout16 + planar layout + ext_ram_sdram fb stream client.  Checks
    # single word, full 640px line + cadence, CPU interleave, planar write
    # round-trip.  sdram_sim.v is the two-plane 16-bit model.
    vlog -quiet -sv ../rtl/ext_ram_sdram.sv sdram_sim.v tb_fb.v
    vsim -c -do "run -all; quit -f" tb_fb 2>&1 | grep -E "FB  |Error|Fatal"
    exit 0 ;;
  bitmap)
    # SDRAM-backed bitmap layer.  bitmap_regs: $9F60-$6F decode / planar addr /
    # palette / pointer.  bitmap_engine: scanout vs golden image (real engine <->
    # ext_ram_sdram <-> sdram_sim, VERA-like raster in, every scanned pixel
    # checked against palette[golden(x,y)]).  TMODE=1 (8bpp), TMODE=2 (4bpp).
    vlog -quiet -sv ../rtl/bitmap_regs.sv tb_bmpregs.v
    vsim -c -do "run -all; quit -f" tb_bmpregs 2>&1 | grep -E "BR  |Error|Fatal"
    vlog -quiet -sv ../rtl/bitmap_regs.sv ../rtl/ext_ram_sdram.sv sdram_sim.v tb_bmpio.v
    vsim -c -do "run -all; quit -f" tb_bmpio 2>&1 | grep -E "BIO |Error|Fatal"
    vlog -quiet -sv ../rtl/bitmap_regs.sv ../rtl/ext_ram_sdram.sv sdram_sim.v tb_blit.v
    vsim -c -do "run -all; quit -f" tb_blit 2>&1 | grep -E "BLIT|Error|Fatal"
    vlog -quiet -sv ../rtl/bitmap_engine.sv ../rtl/ext_ram_sdram.sv sdram_sim.v tb_bitmap.v
    for M in 1 2; do
      echo "----- TMODE=$M -----"
      vsim -c -gTMODE=$M -do "run -all; quit -f" tb_bitmap 2>&1 | grep -E "BMP |Error|Fatal"
    done
    exit 0 ;;
  boot816)
    # FULL R49 ROM BOOT on the P65C816 (65C816 branch): the ROM detects the
    # '816 and boots through the NATIVE-mode paths -- the exact route the HW
    # splash-freeze takes.  Same TB as `boot`, -gCPU816=1.
    mkdir -p rom && cp -f ../rom/rom.hex rom/rom.hex
    C816="$(cd "$(dirname "$0")/../rtl/cpu/65C816_x16" && pwd)"
    vcom -quiet -2008 -work work \
         "$C816/P65816_pkg.vhd" "$C816/BCDAdder.vhd" "$C816/AddSubBCD.vhd" \
         "$C816/ALU.vhd" "$C816/AddrGen.vhd" "$C816/MCode.vhd" \
         "$C816/P65C816.vhd" "$C816/p65c816_wrap.vhd"
    vlog -quiet -sv +define+BAKE_ROM ../rtl/rom_banks.sv ../rtl/lowram_bram.sv \
         ../rtl/ext_ram_sdram.sv ../rtl/ext_ram_bram.sv sdram_sim.v \
         ../rtl/via65c22.sv ../rtl/smc_x16.sv ../rtl/rtc_x16.sv \
         vera_stub.sv tb_fullboot.v
    vsim -c -gCPU816=1 -do "run -all; quit -f" tb_fullboot 2>&1 | grep -E "HB|BRK|MON|KEY|TBC|Error|Fatal"
    exit 0 ;;
  bootmj)
    # FULL R49 ROM BOOT on the mj65c02: the ROM detects a real 65C02 and
    # boots the 6502-mode paths (like `boot`).  PASSED 2026-07-11: BRKs=0,
    # splash drawn, DATE$ path survived.  Same TB, -gCPUMJ=1.
    mkdir -p rom && cp -f ../rom/rom.hex rom/rom.hex
    compile_mj
    vlog -quiet -sv +define+BAKE_ROM ../rtl/rom_banks.sv ../rtl/lowram_bram.sv \
         ../rtl/ext_ram_sdram.sv ../rtl/ext_ram_bram.sv sdram_sim.v \
         ../rtl/via65c22.sv ../rtl/smc_x16.sv ../rtl/rtc_x16.sv \
         vera_stub.sv tb_fullboot.v
    vsim -c -gCPUMJ=1 -do "run -all; quit -f" tb_fullboot 2>&1 | grep -E "HB|BRK|MON|KEY|TBC|Error|Fatal"
    exit 0 ;;
  boot)
    # FULL R49 ROM BOOT on the real RTL (r65c02 + rom_banks + hiram + via +
    # smc + rtc + vera_stub), then types ?DA$<CR> via the SMC kbd path.
    # BRK trap dumps the wild-jump trail.  Slow: tens of minutes.
    mkdir -p rom && cp -f ../rom/rom.hex rom/rom.hex
    vlog -quiet -sv +define+BAKE_ROM ../rtl/rom_banks.sv ../rtl/lowram_bram.sv \
         ../rtl/ext_ram_sdram.sv ../rtl/ext_ram_bram.sv sdram_sim.v \
         ../rtl/via65c22.sv ../rtl/smc_x16.sv ../rtl/rtc_x16.sv \
         vera_stub.sv tb_fullboot.v
    vsim -c -do "run -all; quit -f" tb_fullboot 2>&1 | grep -E "HB|BRK|MON|KEY|TBC|Error|Fatal"
    exit 0 ;;
  date)
    # PRINT DATE$ crash hunt: real r65c02 + via65c22 + smc_x16 + rtc_x16,
    # running faithful i2c.s/rtc.s transcriptions with IRQ-driven SMC reads.
    ca65 --cpu 65C02 datetest.s -o datetest.o
    ld65 -C banktest.cfg datetest.o -o datetest.rom
    od -An -v -tx1 datetest.rom | tr -s ' ' '\n' | grep -v '^$' > datetest.hex
    vlog -quiet -sv ../rtl/via65c22.sv ../rtl/lowram_bram.sv ../rtl/smc_x16.sv \
         ../rtl/rtc_x16.sv tb_datetest.v
    vsim -c -do "run -all; quit -f" tb_datetest 2>&1 | grep -E "DATE|Error|Fatal"
    exit 0 ;;
  bank)
    # 256 ROM banks / cart feature: real ext_ram_sdram (+behavioral sdram) +
    # lowram_bram + ext_ram_bram + r65c02, running banktest.s.  Verifies HiRAM
    # SDRAM banks (regression), cart banks 32-255 r/w + isolation + EXECUTION
    # from cart, banks 16-31 = $FF, bank-0 ROM intact.
    ca65 --cpu 65C02 banktest.s -o banktest.o
    ld65 -C banktest.cfg banktest.o -o banktest.rom
    od -An -v -tx1 banktest.rom | tr -s ' ' '\n' | grep -v '^$' > banktest.hex
    vlog -quiet -sv ../rtl/ext_ram_sdram.sv ../rtl/ext_ram_bram.sv \
         ../rtl/lowram_bram.sv sdram_sim.v tb_bank.v
    vsim -c -do "run -all; quit -f" tb_bank 2>&1 | grep -E "TBC|MEM|LD|Error|Fatal"
    exit 0 ;;
  i2c)
    # HW splash-freeze hunt: both I2C slaves (smc_x16 + rtc_x16) on one bus
    # driven with the KERNAL i2c.s bit-bang timing and transaction shapes.
    vlog -quiet -sv ../rtl/smc_x16.sv ../rtl/rtc_x16.sv ../rtl/ps2_to_smc_bridge.sv tb_i2cboot.v
    vsim -c -do "run -all; quit -f" tb_i2cboot 2>&1 | grep -E "I2CB|Error|Fatal"
    exit 0 ;;
  periph)
    # 2026-07-05 peripheral batch: i2s_rx vs vera's real dacif, snes_pad vs
    # the KERNAL r49 joystick_scan protocol, ps2_to_smc_bridge key+mouse.
    vlog -quiet -sv ../rtl/x16_periph.sv ../rtl/ps2_to_smc_bridge.sv \
         ../rtl/rtc_x16.sv ../vera/fpga/source/audio/dacif.v tb_periph.v
    vsim -c -do "run -all; quit -f" tb_periph 2>&1 | grep -E "I2S|PAD|MOUS|TPM|RTC|TBC|Error|Fatal"
    exit 0 ;;
  cart)
    # Cart RAM save-back (2026-07-07): SC2 image -> restore via loader port
    # (16 KB multi-block), dirty-sector-only save via the bk_* read port.
    vlog -quiet -sv ../rtl/ext_ram_sdram.sv ../rtl/cart_backer.sv          sdram_sim.v tb_cart.v
    vsim -c -do "run -all; quit -f" tb_cart 2>&1 | grep -E "CART|Error|Fatal"
    exit 0 ;;
  spi100)
    # 100 MHz-domain SD master (SD saga act 3): 12.5 MHz sck, single-clock
    # SPI vs sd_card, toggle-handshake register CDC.  Incl. pre-mount boot
    # probe (the sequence that killed the 4 MHz build on HW).
    vlog -quiet -sv ../rtl/spi_sd_master100.sv sd_card_sim.sv tb_spi100.v
    vsim -c -do "run -all; quit -f" tb_spi100 2>&1 | grep -E "S100|Error|Fatal"
    exit 0 ;;
  spi4)
    # 4 MHz guest-SD SPI (2026-07-07 "PCM video stutters" fix): master at
    # cpu_clk/2 + late-MISO sampling + 1-FF capture vs sd_card @100 MHz --
    # the exact x16.sv topology.  CMD17 auto-tx burst + CMD24 fast write.
    vlog -quiet -sv ../rtl/spi_sd_master.sv sd_card_sim.sv tb_spi4.v
    vsim -c -gMH=3 -do "run -all; quit -f" tb_spi4 2>&1 | grep -E "SPI4|Error|Fatal"
    vsim -c -gMH=1 -do "run -all; quit -f" tb_spi4 2>&1 | grep -E "SPI4|Error|Fatal"
    vsim -c -gMH=0 -do "run -all; quit -f" tb_spi4 2>&1 | grep -E "SPI4|Error|Fatal"
    exit 0 ;;
  nvram)
    # RTC NVRAM persistence (2026-07-07): restore-on-mount + debounced
    # autosave through the S1 virtual-disk slot (rtl/nvram_backer.sv).
    vlog -quiet -sv ../rtl/rtc_x16.sv ../rtl/nvram_backer.sv tb_nvram.v
    vsim -c -do "run -all; quit -f" tb_nvram 2>&1 | grep -E "NVR|Error|Fatal"
    exit 0 ;;
  combo)
    # SMC keyboard combos (2026-07-07): Ctrl+Alt+Del -> reset,
    # Ctrl+Alt+PrtScr ($84 SysRq or E0 7C) -> NMI, via bridge + smc_x16.
    vlog -quiet -sv ../rtl/smc_x16.sv ../rtl/ps2_to_smc_bridge.sv tb_smccombo.v
    vsim -c -do "run -all; quit -f" tb_smccombo 2>&1 | grep -E "CMB|Error|Fatal"
    exit 0 ;;
  wai)
    # WAI/STP (2026-07-07): P65C816 native + r65c02 wai_shim control build.
    # (bare r65c02 without the shim documented-fails: $CB/$DB = 1-cycle NOPs)
    C816="$(cd "$(dirname "$0")/../rtl/cpu/65C816_x16" && pwd)"
    vcom -quiet -2008 -work work          "$C816/P65816_pkg.vhd" "$C816/BCDAdder.vhd" "$C816/AddSubBCD.vhd"          "$C816/ALU.vhd" "$C816/AddrGen.vhd" "$C816/MCode.vhd"          "$C816/P65C816.vhd" "$C816/p65c816_wrap.vhd"
    vlog -quiet -sv ../rtl/wai_shim.sv tb_wai.v
    vsim -c -gCPU816=1 -do "run -all; quit -f" tb_wai 2>&1 | grep -E "WAI|Error|Fatal"
    vsim -c -gCPU816=0 -gSHIM=1 -do "run -all; quit -f" tb_wai 2>&1 | grep -E "WAI|Error|Fatal"
    # mj65c02 + wai_shim (sync active-HIGH -> no inversion in the TB/x16.sv)
    compile_mj
    vsim -c -gCPUMJ=1 -do "run -all; quit -f" tb_wai 2>&1 | grep -E "WAI|Error|Fatal"
    exit 0 ;;
  verafx)
    # VERA FX (2026-07-07): cache fill/write/masks, transparency, 4-bit mode,
    # multiplier+accumulator, line draw / polygon filler / affine helpers,
    # 16-bit hop, version regs -- against the real VERA top, CPU-shaped bus.
    vlog -quiet ../vera/fpga/source/top.v ../vera/fpga/source/addr_data.v \
         ../vera/fpga/source/mult_accum.v ../vera/fpga/source/main_ram.v \
         ../vera/fpga/source/dpram.v ../vera/fpga/source/reset_sync.v \
         ../vera/fpga/source/vram_if.v ../vera/fpga/source/sim/palette_ram.v \
         ../vera/fpga/source/sim/sprite_ram.v \
         ../vera/fpga/source/graphics/composer.v \
         ../vera/fpga/source/graphics/layer_line_buffer.v \
         ../vera/fpga/source/graphics/layer_renderer.v \
         ../vera/fpga/source/graphics/sprite_line_buffer.v \
         ../vera/fpga/source/graphics/sprite_renderer.v \
         ../vera/fpga/source/video/video_vga.v \
         ../vera/fpga/source/video/video_composite.v \
         ../vera/fpga/source/video/video_modulator.v \
         ../vera/fpga/source/video/video_modulator_coslut.v \
         ../vera/fpga/source/video/video_modulator_sinlut.v \
         ../vera/fpga/source/audio/audio.v ../vera/fpga/source/audio/audio_fifo.v \
         ../vera/fpga/source/audio/dacif.v ../vera/fpga/source/audio/pcm.v \
         ../vera/fpga/source/audio/psg.v ../vera/fpga/source/spi/spictrl.v
    vlog -quiet tb_verafx.v
    vsim -c -do "run -all; quit -f" tb_verafx 2>&1 | grep -E "FX|Error|Fatal"
    exit 0 ;;
  pcm)
    # VERA PCM audio END-TO-END: real VERA top (extbus $9F3B-3D decode ->
    # pcm.v -> audio_fifo -> dacif) into the shipping i2s_rx, CPU-shaped bus
    # cycles.  16-bit stereo + 8-bit mono playback, flags, ISR fifo-low, vol 0.
    vlog -quiet ../vera/fpga/source/top.v ../vera/fpga/source/addr_data.v \
         ../vera/fpga/source/mult_accum.v ../vera/fpga/source/main_ram.v \
         ../vera/fpga/source/dpram.v ../vera/fpga/source/reset_sync.v \
         ../vera/fpga/source/vram_if.v ../vera/fpga/source/sim/palette_ram.v \
         ../vera/fpga/source/sim/sprite_ram.v \
         ../vera/fpga/source/graphics/composer.v \
         ../vera/fpga/source/graphics/layer_line_buffer.v \
         ../vera/fpga/source/graphics/layer_renderer.v \
         ../vera/fpga/source/graphics/sprite_line_buffer.v \
         ../vera/fpga/source/graphics/sprite_renderer.v \
         ../vera/fpga/source/video/video_vga.v \
         ../vera/fpga/source/video/video_composite.v \
         ../vera/fpga/source/video/video_modulator.v \
         ../vera/fpga/source/video/video_modulator_coslut.v \
         ../vera/fpga/source/video/video_modulator_sinlut.v \
         ../vera/fpga/source/audio/audio.v ../vera/fpga/source/audio/audio_fifo.v \
         ../vera/fpga/source/audio/dacif.v ../vera/fpga/source/audio/pcm.v \
         ../vera/fpga/source/audio/psg.v ../vera/fpga/source/spi/spictrl.v
    vlog -quiet -sv ../rtl/x16_periph.sv tb_pcm.v
    vsim -c -do "run -all; quit -f" tb_pcm 2>&1 | grep -E "PCM|Error|Fatal"
    exit 0 ;;
  wedge)
    # BUG1 reproduction: pre-mount SD probe wedges sd_card; ss-gating fixes it.
    # Expect: GATE_SS=0 -> READ FAILED (lba0 corrupt);  GATE_SS=1 -> READ OK.
    for G in 0 1; do
      echo "----- GATE_SS=$G -----"
      vsim -c -gGATE_SS=$G -do "run -all; quit -f" tb_wedge 2>&1 | grep -E "CFG|R1=|450/510/511|READ OK|READ FAIL|TIMEOUT"
    done
    exit 0 ;;
  relaunch)
    # RELAUNCH SD wedge (2026-07-15 tester bug): SC0 auto-remount pre-sets
    # vsd_sel=1 BEFORE the ROM boot probe -> the probe's abandoned CMD17 hits a
    # selected sd_card -> read_state wedges (WAIT_IO) -> later commands are
    # REJECTED -> locked boot / garbage first read.  FIX=4 = the shipped fix
    # (x16.sv vsd_hold): card DESELECTED for a boot-hold after every reset, so
    # the probe sees "no card" -- the HW-proven first-launch path.
    #   FIX=1 documents the rejected naive fix (vsd_sel low only during the
    #   reset pulse -- probe runs after release, still wedges).
    # Expect: FIX=0 -> READ FAILED (wedge or garbage lba0);  FIX=4 -> READ OK.
    vlog -quiet -sv ../rtl/spi_sd_master.sv sd_card_sim.sv tb_relaunch.v
    for F in 0 4; do
      echo "----- FIX=$F -----"
      vsim -c -gFIX=$F -gMHALF=0 -gHPS_READY_LAG=250000 -gBOOT_HOLD=5000000 -gWATCHDOG=120000000 \
           -do "run -all; quit -f" tb_relaunch 2>&1 | grep -E "CFG|res_n released|boot probe done|F7 typed|READ OK|READ FAIL|TIMEOUT|WEDGE"
    done
    exit 0 ;;
  sweep)
    for AD in 0 1 2 4 8; do
      echo "----- ACK_DROP=$AD -----"
      vsim -c -gACK_DROP=$AD -do "run -all; quit -f" tb_cpu_hps 2>&1 | grep -E "CFG|READ OK|READ FAIL|TIMEOUT"
    done
    for SP in 2 3 4 8 16 32; do
      echo "----- BYTE_SPACING=$SP -----"
      vsim -c -gBYTE_SPACING=$SP -do "run -all; quit -f" tb_cpu_hps 2>&1 | grep -E "CFG|READ OK|READ FAIL|TIMEOUT"
    done
    exit 0 ;;
esac

for TB in $TBS; do
  echo "===== run $TB ====="
  vsim -c -do "run -all; quit -f" $TB 2>&1 | grep -E "CFG|byte450|tail RAM|READ OK|READ FAIL|TIMEOUT"
done
