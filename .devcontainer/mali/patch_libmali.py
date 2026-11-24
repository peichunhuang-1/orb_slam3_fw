#!/usr/bin/env python3
"""Make the host's NXP libmali.so loadable in the Ubuntu 22.04 container (run by gpu_access.sh).

libmali comes from the i.MX BSP and is linked against a newer glibc than the container's 2.35, so the
dynamic linker refuses it ("version `GLIBC_2.38' not found"). Only a few symbols need those versions:
  - symbols that also exist in older glibc (sqrtf@GLIBC_2.43): their version requirement is renamed to
    the aarch64 base version GLIBC_2.17, so they bind to the container's glibc;
  - symbols that do not exist in glibc 2.35 (__isoc23_strtol, ...): same rename, and libmali-compat.so
    (compat.c), added as a DT_NEEDED by patchelf, provides them.
Only the copy in the container is changed; the host's file is mounted read-only.

usage: patch_libmali.py SRC_LIBMALI DST_LIBMALI COMPAT_SO
"""
import os
import re
import shutil
import struct
import subprocess
import sys

SHT_GNU_VERDEF = 0x6FFFFFFD
SHT_GNU_VERNEED = 0x6FFFFFFE
BASE_VERSION = "GLIBC_2.17"  # the first glibc version on aarch64, defined by every aarch64 glibc
LIBDIRS = ["/lib/aarch64-linux-gnu", "/usr/lib/aarch64-linux-gnu", "/lib", "/usr/lib"]


def elf_hash(name: bytes) -> int:
    h = 0
    for c in name:
        h = ((h << 4) + c) & 0xFFFFFFFF
        g = h & 0xF0000000
        if g:
            h ^= g >> 24
        h &= ~g & 0xFFFFFFFF
    return h


class Elf:
    def __init__(self, data):
        self.d = data
        if data[:4] != b"\x7fELF" or data[4] != 2 or data[5] != 1:
            raise SystemExit("not a little-endian ELF64 file")
        shoff, = struct.unpack_from("<Q", data, 0x28)
        shentsize, shnum, shstrndx = struct.unpack_from("<HHH", data, 0x3A)
        self.sections = []
        for i in range(shnum):
            name, typ, _flags, _addr, off, size, link, info, _align, _entsize = struct.unpack_from(
                "<IIQQQQIIQQ", data, shoff + i * shentsize)
            self.sections.append(dict(type=typ, off=off, size=size, link=link, info=info))

    def section(self, typ):
        return next((s for s in self.sections if s["type"] == typ), None)

    def cstr(self, off):
        return bytes(self.d[off:self.d.index(b"\0", off)])

    def defined_versions(self):
        vd = self.section(SHT_GNU_VERDEF)
        if vd is None:
            return set()
        strtab = self.sections[vd["link"]]["off"]
        out, off = set(), vd["off"]
        for _ in range(vd["info"]):
            _ver, _flags, _ndx, cnt, _hash, aux, nxt = struct.unpack_from("<HHHHIII", self.d, off)
            if cnt:
                name, = struct.unpack_from("<I", self.d, off + aux)
                out.add(self.cstr(strtab + name).decode())
            off += nxt
            if not nxt:
                break
        return out

    def needed_versions(self):
        """Yields (file, version, offset of the Vernaux entry, offset of .dynstr)."""
        vn = self.section(SHT_GNU_VERNEED)
        if vn is None:
            return
        strtab = self.sections[vn["link"]]["off"]
        off = vn["off"]
        for _ in range(vn["info"]):
            _ver, cnt, file_, aux, nxt = struct.unpack_from("<HHIII", self.d, off)
            fname = self.cstr(strtab + file_).decode()
            a = off + aux
            for _ in range(cnt):
                _h, _fl, _other, name, anext = struct.unpack_from("<IHHII", self.d, a)
                yield fname, self.cstr(strtab + name).decode(), a, strtab
                a += anext
                if not anext:
                    break
            off += nxt
            if not nxt:
                break


def container_versions(soname):
    for d in LIBDIRS:
        p = os.path.join(d, soname)
        if os.path.exists(p):
            with open(os.path.realpath(p), "rb") as f:
                return Elf(bytearray(f.read())).defined_versions()
    return None


def main():
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    src, dst, compat = sys.argv[1:]
    tmp = dst + ".tmp"
    shutil.copyfile(src, tmp)
    subprocess.check_call(["patchelf", "--add-needed", compat, tmp])

    with open(tmp, "rb") as f:
        data = bytearray(f.read())
    elf = Elf(data)
    provided = {}
    renamed, missing = [], []
    for fname, ver, aux_off, strtab in list(elf.needed_versions()):
        if fname not in provided:
            provided[fname] = container_versions(fname)
        have = provided[fname]
        if have is None:
            missing.append(f"{fname} (not installed in the container)")
            continue
        if ver in have:
            continue
        if not ver.startswith("GLIBC_2."):
            missing.append(f"{fname} {ver}")
            continue
        m = re.search(rb"\0" + BASE_VERSION.encode() + rb"\0", data[strtab:])
        if m is None:
            raise SystemExit(f"{BASE_VERSION} is not in libmali's .dynstr")
        name_off = m.start() + 1  # offset relative to .dynstr
        struct.pack_into("<IHHII", data, aux_off,
                         elf_hash(BASE_VERSION.encode()),
                         0,  # vna_flags
                         struct.unpack_from("<H", data, aux_off + 6)[0],  # keep vna_other (the versym index)
                         name_off,
                         struct.unpack_from("<I", data, aux_off + 12)[0])
        renamed.append(f"{fname} {ver}")
    if missing:
        os.unlink(tmp)
        raise SystemExit("patch_libmali: the container lacks " + ", ".join(sorted(set(missing))))
    with open(tmp, "wb") as f:
        f.write(data)
    os.chmod(tmp, 0o755)
    os.replace(tmp, dst)
    print("patch_libmali: %s -> %s (renamed to %s: %s)" % (src, dst, BASE_VERSION, ", ".join(renamed) or "none"))


if __name__ == "__main__":
    main()
