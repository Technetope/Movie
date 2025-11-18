#!/usr/bin/env python3
"""
Convert one or more WAV files to C headers with PROGMEM arrays.

Usage:
  python3 wav_to_header.py src/wav/aurora.wav src/wav/birds.wav

Outputs:
  For each input `foo.wav`, writes `foo_wav.h` in the same directory.
"""

import argparse
import pathlib


def wav_to_header(path: pathlib.Path) -> pathlib.Path:
    data = path.read_bytes()
    base = path.stem.replace("-", "_")
    array_name = f"{base}_wav"
    lines = [
        "#pragma once",
        "#include <cstdint>",
        "#include <pgmspace.h>",
        "",
        f"static const uint8_t {array_name}[] PROGMEM = {{",
    ]
    for i in range(0, len(data), 12):
        chunk = data[i : i + 12]
        hexes = ", ".join(f"0x{b:02x}" for b in chunk)
        lines.append(f"  {hexes},")
    lines.append("};")
    lines.append(f"static const size_t {array_name}_len = {len(data)};")
    lines.append("")

    out_path = path.with_name(f"{base}_wav.h")
    out_path.write_text("\n".join(lines))
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert WAV files to C headers with PROGMEM arrays."
    )
    parser.add_argument(
        "wav",
        nargs="+",
        type=pathlib.Path,
        help="Input WAV files",
    )
    args = parser.parse_args()

    for wav_file in args.wav:
        out = wav_to_header(wav_file)
        print(f"Wrote {out} ({wav_file.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
