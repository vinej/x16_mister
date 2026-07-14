#!/usr/bin/env python3
"""Build a MiSTer-mountable X16 SD image from a folder OR a .zip, no admin rights.

Produces the same layout as the proven rom/boot0.img: MBR with a single
FAT32-LBA (type 0x0C) partition at LBA 2048, FAT32 volume with LFN support.

Usage:
    python make_sdimage.py <source_folder | source.zip> <output.img> [size_mb]

A .zip source is read straight from the archive (no temp unzip), so a lot of
X16-site releases -- which ship as .zip files holding files laid out like a
disk directory -- become a one-command mount:

    python make_sdimage.py newgame.zip newgame.img

The zip's internal directory structure is preserved as-is.  Then either mount
the .img from the OSD (Mount SD lists *.img) or copy it to
/media/fat/games/X16/boot0.vhd for auto-mount at core start.

Requires: pip install pyfatfs "setuptools<81"
"""
import os
import struct
import sys
import zipfile


class FolderSource:
    """A directory tree on disk."""

    def __init__(self, root):
        self.root = root

    def scan(self):
        """Return (dirs, files): dirs is a list of '/'-rooted FAT subdir paths
        (excluding '/'); files is a list of (fat_dir, name, key) where key is
        whatever read() needs to fetch the bytes."""
        dirs, files = [], []
        for r, _ds, fs in os.walk(self.root):
            rel = os.path.relpath(r, self.root).replace('\\', '/')
            fat_dir = '/' if rel == '.' else '/' + rel
            if fat_dir != '/':
                dirs.append(fat_dir)
            for name in fs:
                files.append((fat_dir, name, os.path.join(r, name)))
        return dirs, files

    def read(self, key):
        with open(key, 'rb') as fh:
            return fh.read()

    def close(self):
        pass


class ZipSource:
    """A .zip archive, streamed entry by entry (no extraction to disk)."""

    def __init__(self, path):
        self.zf = zipfile.ZipFile(path)

    def scan(self):
        dirs = set()
        files = []
        for info in self.zf.infolist():
            arc = info.filename
            parts = [p for p in arc.split('/') if p]
            if info.is_dir():
                # register the explicit directory and its ancestors
                acc = ''
                for p in parts:
                    acc += '/' + p
                    dirs.add(acc)
                continue
            if not parts:
                continue
            name = parts[-1]
            # register every ancestor directory of this file
            acc = ''
            for p in parts[:-1]:
                acc += '/' + p
                dirs.add(acc)
            fat_dir = '/' + '/'.join(parts[:-1]) if len(parts) > 1 else '/'
            files.append((fat_dir, name, arc))
        # shallow-first so parents are created before children
        return sorted(dirs, key=lambda d: d.count('/')), files

    def read(self, key):
        return self.zf.read(key)

    def close(self):
        self.zf.close()


def open_source(src):
    if os.path.isdir(src):
        print(f'source: folder {src}')
        return FolderSource(src)
    if os.path.isfile(src) and zipfile.is_zipfile(src):
        print(f'source: zip archive {src}')
        return ZipSource(src)
    sys.exit(f'error: {src} is not a folder or a .zip archive')


def build(src: str, out: str, size_mb: int) -> None:
    from pyfatfs.PyFat import PyFat
    from pyfatfs.PyFatFS import PyFatFS
    from pyfatfs.EightDotThree import EightDotThree

    # pyfatfs 1.1.0 bug: names with SPACES pass is_8dot3_conform, but the
    # 8.3 generator strips the space, so create() asks for an LFN entry and
    # make_lfn_entry refuses ("already 8.3 conform").  Real FAT semantics
    # (and Windows) treat space-names as LFN + tilde-SFN; classifying them
    # as non-conform restores exactly that.
    _orig_conform = EightDotThree.is_8dot3_conform

    def _conform_no_space(entry_name, encoding='ibm437'):
        if ' ' in entry_name:
            return False
        return _orig_conform(entry_name, encoding)

    EightDotThree.is_8dot3_conform = staticmethod(_conform_no_space)

    source = open_source(src)
    dirs, files = source.scan()

    part_lba = 2048                       # 1 MiB alignment, like the reference
    total_sectors = size_mb * 2048
    part_sectors = total_sectors - part_lba
    vol_tmp = out + '.vol'

    # 1. FAT32 volume (standalone, then spliced in at 1 MiB)
    print(f'[1/4] mkfs FAT32: {part_sectors} sectors '
          f'({part_sectors * 512 // (1024 * 1024)} MiB)')
    with open(vol_tmp, 'wb') as f:        # mkfs opens rb+, needs the file
        f.seek(part_sectors * 512 - 1)
        f.write(b'\x00')
    pf = PyFat()
    pf.mkfs(vol_tmp, fat_type=PyFat.FAT_TYPE_FAT32,
            size=part_sectors * 512, label='X16')
    pf.close()

    # 2. assemble MBR + volume
    print('[2/4] assembling MBR image')
    mbr = bytearray(512)
    # CHS fields are dummies (LBA-only, matches the reference image)
    entry = struct.pack('<B3sB3sII', 0x00, b'\xff\xff\xff', 0x0C,
                        b'\xff\xff\xff', part_lba, part_sectors)
    mbr[446:462] = entry
    mbr[510:512] = b'\x55\xaa'
    with open(out, 'wb') as f:
        f.write(mbr)
        f.write(b'\x00' * (part_lba * 512 - 512))
        with open(vol_tmp, 'rb') as v:
            while True:
                chunk = v.read(1 << 20)
                if not chunk:
                    break
                f.write(chunk)
    os.remove(vol_tmp)

    # patch BPB "hidden sectors" = partition offset (what a real format does)
    with open(out, 'r+b') as f:
        f.seek(part_lba * 512 + 0x1C)
        f.write(struct.pack('<I', part_lba))

    # 3. copy the tree
    print('[3/4] copying files')
    # preserve_case=True: lowercase 8.3-length names ("00.zsm") become
    # LFN + uppercase SFN, exactly like Windows.  (preserve_case=False
    # broke pyfatfs' own read-back lookups of such names.)  The space-name
    # crash is handled by the is_8dot3_conform patch above.
    fatfs = PyFatFS(out, offset=part_lba * 512, preserve_case=True,
                    lazy_load=False)
    for fat_dir in dirs:
        fatfs.makedirs(fat_dir, recreate=True)
    sizes = {}
    nfiles = 0
    nbytes = 0
    for fat_dir, name, key in files:
        data = source.read(key)
        fatfs.writebytes(fat_dir.rstrip('/') + '/' + name, data)
        sizes[(fat_dir, name)] = len(data)
        nfiles += 1
        nbytes += len(data)
        if nfiles % 100 == 0:
            print(f'      {nfiles} files, {nbytes // (1024 * 1024)} MiB')
    fatfs.close()
    print(f'      total {nfiles} files, {nbytes // (1024 * 1024)} MiB')

    # 4. verify: reopen read-only and spot-check sizes
    print('[4/4] verifying')
    fatfs = PyFatFS(out, offset=part_lba * 512, read_only=True)
    bad = 0
    checked = 0
    for fat_dir, name, _key in files:
        want = sizes[(fat_dir, name)]
        path = fat_dir.rstrip('/') + '/' + name
        try:
            got = fatfs.getsize(path)
        except Exception as e:
            print(f'      MISSING {path}: {e}')
            bad += 1
            continue
        if got != want:
            print(f'      SIZE MISMATCH {path}: {got} != {want}')
            bad += 1
        checked += 1
    fatfs.close()
    source.close()
    print(f'      {checked} files checked, {bad} problems')
    if bad:
        sys.exit(1)
    print(f'OK: {out} ({size_mb} MB)')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    src = sys.argv[1]
    out = sys.argv[2]
    size_mb = int(sys.argv[3]) if len(sys.argv) > 3 else 512
    build(src, out, size_mb)
