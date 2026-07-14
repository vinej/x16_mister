#!/usr/bin/env python3
"""
fix_fsinfo.py - recompute a FAT32 image's FSInfo free-cluster count.

WHY: tools like WinImage write the spec's "unknown" sentinel (0xFFFFFFFF)
into FSInfo after modifying an image.  The X16 ROM's DOS trusts FSInfo
blindly, so `@$` then shows nonsense like "4095 GB FREE".  This scans the
FAT, computes the true free count, and patches FSInfo in place.

Usage:  python fix_fsinfo.py <image.img>

Assumes an MBR image whose first partition is FAT32 (the layout mkfs/the
X16 tooling produces).  Harmless to run on an already-correct image.
"""

import struct
import sys


def main(path):
    f = open(path, 'r+b')

    # ---- find the first partition from the MBR ----
    mbr = f.read(512)
    if mbr[510:512] != b'\x55\xaa':
        sys.exit("not an MBR image (no 55AA signature)")
    part_lba = struct.unpack('<I', mbr[446 + 8:446 + 12])[0]
    base = part_lba * 512

    # ---- FAT32 BPB ----
    f.seek(base)
    vbr = f.read(512)
    if vbr[510:512] != b'\x55\xaa':
        sys.exit("partition has no valid boot sector")
    bps    = struct.unpack('<H', vbr[11:13])[0]
    spc    = vbr[13]
    rsvd   = struct.unpack('<H', vbr[14:16])[0]
    nfats  = vbr[16]
    totsec = struct.unpack('<I', vbr[32:36])[0]
    fatsz  = struct.unpack('<I', vbr[36:40])[0]
    fsinfo_sec = struct.unpack('<H', vbr[48:50])[0]
    nclust = (totsec - rsvd - nfats * fatsz) // spc
    print(f"partition @ LBA {part_lba}: {bps}B sectors, {spc} sec/cluster, "
          f"{nclust} clusters, FAT {fatsz} sectors x{nfats}")

    # ---- count free clusters in FAT #1 ----
    f.seek(base + rsvd * bps)
    fat = f.read(fatsz * bps)
    free = sum(1 for c in range(2, nclust + 2)
               if struct.unpack('<I', fat[c * 4:c * 4 + 4])[0] & 0x0FFFFFFF == 0)
    mb = free * spc * bps / 1048576
    print(f"true free clusters = {free}  ({mb:.1f} MB)")

    # ---- patch FSInfo ----
    off = base + fsinfo_sec * bps
    f.seek(off)
    fsi = f.read(512)
    if fsi[0:4] != b'RRaA' or fsi[484:488] != b'rrAa':
        sys.exit("FSInfo signatures not found -- refusing to patch")
    old = struct.unpack('<I', fsi[488:492])[0]
    f.seek(off + 488)
    f.write(struct.pack('<I', free))
    f.close()
    print(f"FSInfo free count: {old:#010x} -> {free:#010x}  PATCHED")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
