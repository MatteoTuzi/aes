#!/usr/bin/env python3
"""
extract_q17_to_bin.py

Estrae i valori Q1.7 da weights_group0.h e li salva in file .bin (signed char, 1 byte per valore).

Input:  assignment_5/aes_mnist_assigment_groups/group_0/weights_group0.h
Output: assignment_5/aes_mnist_assigment_groups/group_0/weights_q17/
        - bias0_q17.bin   (64 bytes)
        - weights0_q17.bin (64*784 = 50176 bytes)
        - bias1_q17.bin   (10 bytes)
        - weights1_q17.bin (10*64 = 640 bytes)

Uso:
  python extract_q17_to_bin.py [--header PATH] [--output-dir PATH]
"""

import argparse
import re
import struct
from pathlib import Path


def parse_q17_macro(header_path: Path, macro_name: str) -> list[int]:
    """Estrae i valori numerici da un #define MACRO val1,val2,..."""
    with open(header_path, "r", encoding="utf-8") as f:
        content = f.read()

    pattern = rf"#define\s+{re.escape(macro_name)}\s+([^\n]+)"
    match = re.search(pattern, content)
    if not match:
        raise ValueError(f"Macro {macro_name} non trovata in {header_path}")

    values_str = match.group(1).strip()
    values = [int(x.strip()) for x in values_str.split(",")]
    return values


def main():
    parser = argparse.ArgumentParser(
        description="Estrae Q1.7 da weights_group0.h in file .bin"
    )
    parser.add_argument(
        "--header",
        type=Path,
        default=Path(__file__).parent.parent / "aes_mnist_assigment_groups" / "group_0" / "weights_group0.h",
        help="Percorso al file weights_group0.h",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).parent.parent / "aes_mnist_assigment_groups" / "group_0" / "weights_q17",
        help="Directory di output per i file .bin",
    )
    args = parser.parse_args()

    if not args.header.exists():
        print(f"Errore: {args.header} non trovato.")
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    macros = [
        ("bias0_q17", "bias0_q17.bin", 64),
        ("weights0_q17", "weights0_q17.bin", 64 * 784),
        ("bias1_q17", "bias1_q17.bin", 10),
        ("weights1_q17", "weights1_q17.bin", 10 * 64),
    ]

    for macro_name, out_name, expected_len in macros:
        values = parse_q17_macro(args.header, macro_name)
        if len(values) != expected_len:
            print(f"Warning: {macro_name} ha {len(values)} valori (attesi {expected_len})")

        # Converti in signed char (1 byte) e scrivi
        data = struct.pack("<" + "b" * len(values), *[max(-128, min(127, v)) for v in values])
        out_path = args.output_dir / out_name
        with open(out_path, "wb") as f:
            f.write(data)
        print(f"Scritto: {out_path} ({len(data)} bytes)")

    print(f"\nFile Q1.7 salvati in: {args.output_dir}")
    return 0


if __name__ == "__main__":
    exit(main())
