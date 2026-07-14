#!/usr/bin/env python3
"""Regenerate rom/rom.hex from an X16 ROM binary.

The core bakes a default ROM into the FPGA via
    rtl/rom_banks.sv:  initial $readmemh("rom/rom.hex", mem);
That rom.hex is NOT distributed with this repo -- the Commander X16 system ROM
is not ours to ship.  Bring your own 256 KiB ROM image (the concatenation of the
16 X16 ROM banks -- the same bytes you would load at runtime as boot1.rom) and
this rewrites rom/rom.hex in the exact format $readmemh expects: 262144 lines,
one byte per line, lowercase hex.

Where to get the ROM:
  * build the official ROM from source -- github.com/X16Community/x16-rom
    (its build/x16 output concatenated over all 16 banks), or
  * use the rom.bin / *.BIN that ships with your Commander X16 / emulator release.

Usage:
    python scripts/rom_bin2hex.py <rom.bin> [rom/rom.hex]

With no output path it writes rom/rom.hex at the project root.  After running it,
compile the core in Quartus to bake that ROM in as the default.  (Even without
it you can still load a ROM at runtime via boot1.rom / the OSD "Load ROM".)
"""
import os
import sys

EXPECTED = 262144   # 16 banks * 16 KiB = 256 KiB


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    src = argv[1]
    if len(argv) > 2:
        dst = argv[2]
    else:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        dst = os.path.join(root, 'rom', 'rom.hex')

    with open(src, 'rb') as f:
        data = f.read()

    if len(data) != EXPECTED:
        sys.stderr.write(
            f'warning: {src} is {len(data)} bytes, expected {EXPECTED} '
            f'(256 KiB = 16 X16 banks). Writing anyway.\n')

    # one byte per line, lowercase hex, LF endings -- exactly what $readmemh
    # reads and what the sim's `od -An -v -tx1 | tr` pipeline produced.
    with open(dst, 'w', newline='\n') as f:
        f.write(''.join('%02x\n' % b for b in data))

    print(f'wrote {dst}: {len(data)} bytes -> {len(data)} lines')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
