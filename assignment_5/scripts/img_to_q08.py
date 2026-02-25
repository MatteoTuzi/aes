#!/usr/bin/env python3
"""
Convert MNIST 28x28 images to Q0.8 binary format (TIP1).
Output: 784 bytes per image (1 byte per pixel, 0-255).
Use with: python img_to_q08.py image.png -o out.bin
Then send: python send_file_bin.py out.bin
"""

import argparse
from PIL import Image


def img_to_q08(img_path: str, out_path: str) -> None:
    """Load 28x28 grayscale image, convert to Q0.8 (0-255), save as raw bytes."""
    img = Image.open(img_path).convert("L")
    if img.size != (28, 28):
        img = img.resize((28, 28), Image.BILINEAR)
    data = img.tobytes()
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"Wrote {len(data)} bytes to {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Convert MNIST image to Q0.8 binary (784 bytes)")
    parser.add_argument("image", help="Input image (28x28 or will be resized)")
    parser.add_argument("-o", "--output", default="image_q08.bin", help="Output binary file")
    args = parser.parse_args()
    img_to_q08(args.image, args.output)


if __name__ == "__main__":
    main()
