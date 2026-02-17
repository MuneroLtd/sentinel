#!/usr/bin/env python3
"""Patch the LPC4320 boot ROM checksum into a firmware binary.

The LPC43xx boot ROM validates the vector table before jumping to user code.
The sum of the first 8 words (offsets 0x00–0x1C) must equal zero.
Word[7] at offset 0x1C is reserved for the checksum: the 2's complement of
the sum of words[0..6].

Usage:
    python3 lpc_checksum.py <firmware.bin>

The file is modified in-place.

Reference: UM10503 §6.4 (Boot ROM), §6.4.4.1 (Criterion for Valid User Code)
"""

import struct
import sys


def patch_checksum(path: str) -> None:
    with open(path, "r+b") as f:
        header = f.read(32)
        if len(header) < 32:
            print(f"ERROR: {path} is too small ({len(header)} bytes)", file=sys.stderr)
            sys.exit(1)

        words = list(struct.unpack("<8I", header))

        # Compute checksum: 2's complement of sum of words[0..6]
        checksum = (0 - sum(words[:7])) & 0xFFFFFFFF

        if words[7] == checksum:
            print(f"Checksum already valid: 0x{checksum:08X}")
            return

        old = words[7]
        words[7] = checksum

        # Verify: sum of all 8 words must be 0
        assert sum(words) & 0xFFFFFFFF == 0, "Checksum arithmetic error"

        # Patch word[7] in the file
        f.seek(0x1C)
        f.write(struct.pack("<I", checksum))

        print(f"Patched checksum at 0x1C: 0x{old:08X} -> 0x{checksum:08X}")
        print(f"  Word[0] SP:    0x{words[0]:08X}")
        print(f"  Word[1] Reset: 0x{words[1]:08X}")
        print(f"  Word[7] Csum:  0x{words[7]:08X}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <firmware.bin>", file=sys.stderr)
        sys.exit(1)
    patch_checksum(sys.argv[1])
