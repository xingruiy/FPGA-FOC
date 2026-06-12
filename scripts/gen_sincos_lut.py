#!/usr/bin/env python3
"""Generate rtl/math/sincos_lut.mem - quarter-wave sine table with slopes.

Layout: 1024 words x 36 bit, one RAMB36 (1024x36 true dual port).
  word[35:18] = sin value at quarter-code 16*i, Q1.17 unsigned (clamped
                to 0x1FFFF; the exact-1.0 endpoint is a bypass in RTL)
  word[17:0]  = slope to the next entry (v[i+1] - stored v[i]), so that
                linear interpolation with the 4 LSB fraction reconstructs
                sin() to well under 1 LSB of Q1.15.

Angle convention: full electrical circle = 2^16 codes; the table covers
the first quadrant, codes 0..16383, sampled every 16 codes.
"""
import math
from pathlib import Path

N = 1024          # entries (segments) per quarter wave
SEG = 16          # angle codes per segment
SCALE = 1 << 17   # Q1.17

# ideal endpoint values for i = 0..N (v[N] = sin(pi/2) = 131072, unclamped)
vals = [round(math.sin(2 * math.pi * (i * SEG) / 65536) * SCALE)
        for i in range(N + 1)]

out = Path(__file__).resolve().parent.parent / "rtl" / "math" / "sincos_lut.mem"
out.parent.mkdir(parents=True, exist_ok=True)

lines = []
for i in range(N):
    store = min(vals[i], SCALE - 1)
    slope = vals[i + 1] - store
    assert 0 <= slope < (1 << 18), (i, slope)
    lines.append(f"{(store << 18) | slope:09X}")

out.write_text("\n".join(lines) + "\n")
print(f"wrote {out} ({len(lines)} words)")
