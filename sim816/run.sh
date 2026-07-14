#!/usr/bin/env bash
# P65C816 stall-IRQ-storm testbench runner (x16_mister copy of the core).
#
#   ./run.sh           # emu-mode stall pair + BOTH native tests (hooked + default)
#   ./run.sh stall     # emu-mode, stalled run only (tb_stall)
#   ./run.sh nostall   # emu-mode, clean run only (tb_stall)
#   ./run.sh trace N   # emu-mode stalled run with a bus trace from IRQ entry N
#   ./run.sh nat       # native HOOKED-cinv trampoline (irqnat.s), stall on/off
#   ./run.sh def       # native DEFAULT path (irqdef.s = irq_emulated_impl,
#                      #   the path normal boot/BASIC takes), stall on/off
set -e
MS=/c/intelFPGA_lite/17.0/modelsim_ase/win32aloem
CC65=/c/Emulator/cc65/bin
CORE="$(cd "$(dirname "$0")/../rtl/cpu/65C816_x16" && pwd)"
export PATH="$MS:$CC65:$PATH"
cd "$(dirname "$0")"

echo "=== assemble irqstall.s (ca65, 65C02 = emulation mode) ==="
ca65 --cpu 65C02 irqstall.s -o irqstall.o
ld65 -C irqstall.cfg irqstall.o -o irqstall.rom
od -An -v -tx1 irqstall.rom | tr -s ' ' '\n' | grep -v '^$' > irqstall.hex

echo "=== assemble irqnat.s + irqdef.s (ca65, 65816 native mode) ==="
ca65 --cpu 65816 irqnat.s -o irqnat.o
ld65 -C irqnat.cfg irqnat.o -o irqnat.rom
od -An -v -tx1 irqnat.rom | tr -s ' ' '\n' | grep -v '^$' > irqnat.hex
ca65 --cpu 65816 irqdef.s -o irqdef.o
ld65 -C irqnat.cfg irqdef.o -o irqdef.rom
od -An -v -tx1 irqdef.rom | tr -s ' ' '\n' | grep -v '^$' > irqdef.hex

echo "=== compile P65C816 core (VHDL-2008; MCode.vhd is 256KB, slow) ==="
if [ ! -d work ]; then vlib work >/dev/null 2>&1; fi
vcom -quiet -2008 -work work \
     "$CORE/P65816_pkg.vhd" "$CORE/BCDAdder.vhd" "$CORE/AddSubBCD.vhd" "$CORE/ALU.vhd" \
     "$CORE/AddrGen.vhd" "$CORE/MCode.vhd" "$CORE/P65C816.vhd" "$CORE/p65c816_wrap.vhd"

vlog -quiet -sv tb_stall.v tb_native.v

run_tb () {
  vsim -c -gSTALL_ON=$1 -gTRACE_FROM=${2:-0} -do "run -all; quit -f" tb_stall 2>&1 \
    | grep -E "TB\]|IRQ [0-9]|STORM|TR\]"
}

run_native () {  # $1 = hexfile, $2 = STALL_ON
  vsim -c -gHEXFILE="$1" -gSTALL_ON=$2 -do "run -all; quit -f" tb_native 2>&1 \
    | grep -E "TB\]|NIRQ|EVEC|STORM"
}

case "${1:-all}" in
  stall)   run_tb 1 ;;
  nostall) run_tb 0 ;;
  trace)   run_tb 1 "${2:-2}" ;;
  nat)     echo "----- irqnat (hooked cinv), STALL_ON=0 -----"; run_native irqnat.hex 0
           echo "----- irqnat (hooked cinv), STALL_ON=1 -----"; run_native irqnat.hex 1 ;;
  def)     echo "----- irqdef (DEFAULT path), STALL_ON=0 -----"; run_native irqdef.hex 0
           echo "----- irqdef (DEFAULT path), STALL_ON=1 -----"; run_native irqdef.hex 1 ;;
  all|both)
           echo "----- emu-mode STALL_ON=0 (expect clean) -----"; run_tb 0
           echo "----- emu-mode STALL_ON=1 -----"; run_tb 1
           echo "----- irqnat (hooked cinv), STALL_ON=1 -----"; run_native irqnat.hex 1
           echo "----- irqdef (DEFAULT path), STALL_ON=0 -----"; run_native irqdef.hex 0
           echo "----- irqdef (DEFAULT path), STALL_ON=1 -----"; run_native irqdef.hex 1 ;;
esac
