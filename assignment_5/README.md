# Assignment 5 – DNN on MNIST (Group 0)

Lab on **Advanced Embedded Systems (AES)**: fully-connected neural network for MNIST digit classification on Zynq, with fixed-point optimizations (Q8.8, Q0.8, Q1.7) and UART image input. The goal is to reduce memory and UART transmission time while keeping accuracy comparable to the baseline.

---

## Overview

- **Board:** Zybo Z7 (Zynq-7000, ARM Cortex-A9)
- **Network:** 784 → 64 → 10 (FC0 → ReLU → FC1), Group 0 topology
- **Input:** 28×28 grayscale MNIST image (784 pixels)
- **UART:** 115200 8N1; image size depends on MODE (784 or 1568 bytes per frame)

---

## Network architecture

| Stage   | Description                          |
|--------|--------------------------------------|
| Input  | 28×28 grayscale (784 pixels)        |
| FC0    | Fully-connected 784 → 64             |
| ReLU   | Nonlinear activation                 |
| FC1    | Fully-connected 64 → 10 (logits)     |
| Output | Argmax over 10 classes (digit 0–9)   |

**Data flow:**  
UART → `image[]` → FC0 → ReLU → FC1 → argmax → predicted class (0–9).

---

## Fixed-point formats (Q)

Base format for weights and activations is **Q8.8** (see course document “Tips and tricks for optimization”.)

**Qm.n:** *m* bits integer (including sign), *n* bits fractional; real value = stored_value / 2^n.

| Format | Bits | C type           | Use                    | Range (approx.) |
|--------|------|------------------|------------------------|------------------|
| Q8.8   | 16   | `short`          | Images, weights (baseline) | −128 … +128  |
| Q0.8   | 8    | `unsigned char`  | Images via UART (TIP1)     | 0 … 1         |
| Q1.7   | 8    | `signed char`    | Weights (TIP2)             | −1 … +1       |

**Conversions:**

- Q8.8 → Q0.8: `q08 = q88 >> 8` (UART reduction)
- Q8.8 → Q1.7: `q17 = clamp(q88 >> 1, -128, 127)` (weight memory reduction)
- Q1.7 → Q8.8: `q88 = (short)q17 << 1`

---

## Modes and optimizations

### MODE 0 – Baseline

- **Images:** 2 bytes per pixel (Q8.8) ⇒ **1568 bytes** per image via UART
- **Weights:** 2 bytes per parameter ⇒ **~100 KB** total (50176 + 640 + 128 + 20)
- **Precision:** Maximum; reference for comparing MODE 1/2

### MODE 1 – TIP1 (image format)

- **Idea:** In Q8.8, the MSB of each pixel is always 0x00 (MNIST in [0,1]). Send only one byte per pixel (Q0.8).
- **UART:** **784 bytes** per image (50% less). On the board, `readfromUART()` does `(short)px << 8` to get Q8.8; rest of the pipeline unchanged. Weights stay Q8.8.

### MODE 2 – TIP1 + TIP2 (image + weights)

- **Idea:** Weights/bias in hex have MSB 0x00 or 0xFF (integer part = sign only). Store them in Q1.7 (1 byte each).
- **Memory:** **~50 KB** for weights (50% less). Conversion in `FC_forward_q17()`: `(short)weights[i] << 1`, `(short)bias[h] << 1` to align to Q8.8 for the MAC. Images still 784 bytes (TIP1).

### Comparison

| Parameter        | MODE 0   | MODE 1 (TIP1) | MODE 2 (TIP1+TIP2) |
|------------------|----------|----------------|---------------------|
| UART bytes/image | 1568     | 784            | 784                 |
| Weight memory    | ~100 KB  | ~100 KB       | ~50 KB              |
| Image format     | Q8.8     | Q0.8→Q8.8     | Q0.8→Q8.8           |
| Weight format    | Q8.8     | Q8.8          | Q1.7→Q8.8           |
| vs baseline      | —        | 50% UART      | 50% UART + 50% weights |

Set `MODE` in `mnist_group/src/mnist_group0.c` (e.g. `#define MODE 0`) and rebuild.

---

## Software structure (mnist_group0.c)

- **Configuration:** `MODE`, `IMG_SIZE`=784, `N_BIAS0/1`, `N_WEIGHTS0/1`, `DATA` = `short`, `FIXED2FLOAT` macro.
- **Globals:** Weights/bias (Q8.8 or Q1.7 from `weights_group0.h`), `immagine[10][IMG_SIZE]`, UART instance.
- **Main:** Init platform and UART, print banner, run offline test, then UART loop.
- **UART:** `uart_inbyte()` (blocking), `readfromUART()` (2 bytes LSB-first in MODE 0, 1 byte Q0.8 in MODE 1/2), `uart_init()` (115200), `uart_receive_image()`.
- **Layers:** `FC_forward()` (Q8.8 MAC, 64-bit accumulator, `>> qf`); `FC_forward_q17()` (MODE 2, Q1.7 weights/bias scaled with `<< 1`).
- **ReLU / output:** `relu_forward()`, `resultsProcessing()` (fixed→float, argmax → class 0–9).
- **Inference:** `dnn_inference(image)` → FC0 → ReLU → FC1 → `resultsProcessing` → predicted class.
- **Offline test:** Load 10 images from `test_images.h` into `immagine[][]`, run inference, print expected vs predicted.
- **UART loop:** Print expected format; for each iteration: t0 → receive image (t1) → inference (t2) → print prediction + sleep (t3); print timing in µs: **RX**, **DNN**, **PRINT**, **total**.

---

## Project layout

| Path | Description |
|------|-------------|
| `mnist_group/` | DNN application |
| `mnist_group/src/mnist_group0.c` | Main: UART, FC, ReLU, argmax, offline test, UART loop |
| `mnist_group/src/weights_group0.h` | Weights and biases (Q8.8 and/or Q1.7) |
| `mnist_group/src/test_images.h` | 10 embedded test images (digits 0–9) |
| `mnist_group_bsp/` | BSP for the application |
| `report/` | LaTeX report; see `report/README.md` for build |
| `scripts/` | e.g. `bin2headers_group0.py`, `img_to_q08.py`, `extract_q17_to_bin.py` |
| `Tips and tricks for optimization.pdf` | Course notes (TIP1/TIP2) |

---

## Build and run

1. Open in **Xilinx SDK** or **Vitis**; build the `mnist_group` application with the correct BSP.
2. Program the board and run. The program:
   - Runs an **offline test** on the 10 embedded images (expected vs predicted).
   - Enters the **UART loop**: waits for one image, runs inference, prints predicted digit and timing (RX, DNN, PRINT, total in µs).
3. **UART input** @ 115200:
   - **MODE 0:** 1568 bytes (Q8.8, LSB-first, 2 bytes per pixel).
   - **MODE 1/2:** 784 bytes (Q0.8, 1 byte per pixel); board converts to Q8.8 with `(short)px << 8`.

---

## Conclusions (from report)

- TIP1: **50% less UART traffic** (784 vs 1568 bytes per image).
- TIP2: **50% less weight memory** (Q1.7 vs Q8.8).
- Accuracy remains comparable to the baseline; fixing overflow in image initialization was necessary for correct predictions in all modes.

---

## References

- Tips and tricks for optimization (PDF), course material — Q8.8/Q0.8/Q1.7, Tip 1 (image Q0.8 over UART), Tip 2 (weights/bias Q1.7, memory and UART).
