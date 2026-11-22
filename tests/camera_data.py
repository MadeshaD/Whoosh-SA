"""Verify named camera collections in retail or repacked MW and Carbon VPAKs."""
import math
from pathlib import Path
import struct
import sys


def check(data, carbon):
    assert data[:4] == b"VPAK"
    vlt = struct.unpack_from("<I", data, 32)[0]
    ptrs = data.index(b"NrtP", vlt)
    size = struct.unpack_from("<I", data, ptrs + 4)[0]
    bases = (vlt, 128)
    refs = {}
    for offset in range(ptrs + 8, ptrs + size - 11, 12):
        source, kind, index, target = struct.unpack_from("<IHHI", data, offset)
        if kind == 0:
            break
        if kind == 2:
            base = bases[index]
        if kind == 3:
            refs[base + source] = bases[index] + target

    name, lag, fov, height, angle = (0, 0x30, 0x4c, 0x5c, 0x6c) if carbon else (0x60, 0x20, 0x30, 0x40, 0x50)
    presets = {
        b"bumper": (.5, 83, .746, 1.5),
        b"close": (-4.7, 71 if carbon else 78, 1.62, 0),
        b"far": (-5.9 if carbon else -5.436, 71 if carbon else 90, 1.7, 0),
    }
    seen = set()
    header = struct.pack("<4H", 2, 2, 4, 0)
    for source, target in refs.items():
        key = data[target:target+8].split(b"\0")[0]
        layout = source - name
        if key not in presets or data[layout+fov:layout+fov+8] != header:
            continue
        for offset, value in zip((lag, fov, height, angle), presets[key]):
            assert data[layout+offset:layout+offset+8] == header
            actual = struct.unpack_from("<f", data, layout+offset+8)[0]
            assert math.isclose(actual, value, abs_tol=1e-5), (key, offset, actual, value)
        seen.add(key)
    assert seen == presets.keys()
    print(("Carbon" if carbon else "Most Wanted") + ": bumper, close and far camera collections verified.")


if __name__ == "__main__":
    check(Path(sys.argv[1]).read_bytes(), False)
    check(Path(sys.argv[2]).read_bytes(), True)
