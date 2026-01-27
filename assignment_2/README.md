# Assignment 2 – UART Image Processing Microservers

**University of Cagliari – Advanced Embedded Systems (AES)**

This assignment implements three **UART microservers** on a Zybo Z7 (Zynq-7000). Each server receives a P6 PPM image over UART, applies a different image-processing algorithm, and sends the result back. The applications are **standalone** and **separately compilable**: each has its own `main()` and no shared headers.

---

## Overview

| Application   | Source file     | Processing            | Image size            |
|---------------|-----------------|-----------------------|------------------------|
| **Negative**  | `negative.c`    | Pixel inversion       | Fixed **128×128**      |
| **Histogram** | `histogram.c`   | Linear stretching     | **Arbitrary** (≤512×512) |
| **Equalized** | `equalized.c`   | Histogram equalization| **Arbitrary** (≤512×512) |

- **Hardware:** Zybo Z7 (ARM Cortex-A9 PS). **UART:** PS UART1 @ **115200 baud**, 8N1.
- **Protocol:** Client sends PPM (header + raw RGB) over serial; server replies with header + processed pixels.
- **Tools:** Xilinx SDK / Vitis and RealTerm for binary send/receive.

---

## PPM Format (P6)

- **Magic:** `P6`
- **Dimensions:** width and height (ASCII, e.g. `128 128` or `64 64`)
- **Max value:** `255`
- **Pixel data:** Raw binary RGB, one byte per component, row-major. Total bytes = `width × height × 3`.

**Header layout (3 lines):**
```
P6
<width> <height>
255
```
No comments; newline after `255`. Binary pixels follow immediately.

---

## Applications

### 1. `negative.c` – Negative (fixed 128×128)

- **Input:** Fixed header `P6\n128 128\n255\n` (15 bytes) + `128×128×3` RGB bytes.
- **Processing:** `p_out = 255 - p_in` for each R, G, B byte.
- **Output:** Same header + inverted pixels.
- **Buffers:** Static `header[15]` and `image[128*128*3]`; no `malloc`.

### 2. `histogram.c` – Histogram linear stretching

- **Input:** Arbitrary P6 (up to 512×512). Header is parsed line-by-line (`read_line`, `parse_width_height`, `to_int`).
- **Processing:**  
  - Find min and max intensity over all bytes: `I_min`, `I_max`.  
  - `scale = 255 / (I_max - I_min)`  
  - `p_out = (p_in - I_min) * scale`, clamped to [0, 255].  
  - If `I_max == I_min`, image is unchanged.
- **Output:** Same header + stretched pixels.
- **Buffers:** Static `image_buf[512*512*3]`; images larger than 512×512 are rejected.

### 3. `equalized.c` – Histogram equalization

- **Input:** Same as histogram (arbitrary P6, ≤512×512).
- **Processing:**  
  1. Build histogram `hist[0..255]` over all bytes.  
  2. Build CDF: `cdf[i] = sum(hist[0..i])`.  
  3. Find first non-zero CDF value `n_min`.  
  4. LUT: `map[i] = round((cdf[i] - n_min) / (n - n_min) * 255)`, clamped.  
  5. Replace each byte: `img[i] = map[img[i]]`.  
  If all pixels equal, no change.
- **Output:** Same header + equalized pixels.
- **Buffers:** Static `image_buf[512*512*3]`; same size limit as histogram.

---

## Project structure

```
assignment_2/
├── README.md           # This file
├── negative.c          # Negative microserver (128×128)
├── histogram.c         # Histogram stretching (arbitrary ≤512×512)
├── equalized.c         # Histogram equalization (arbitrary ≤512×512)
├── image_uart_Z7.sdk/  # SDK workspace (e.g. microserver app + BSP)
├── negative/           # SDK project for negative (if used)
├── histogram/          # SDK project for histogram (if used)
├── hist_equalized/     # SDK project for equalized (if used)
└── …                   # BSP, platform, etc.
```

Each of `negative.c`, `histogram.c`, and `equalized.c` is **self-contained**: no `.h` dependencies except Xilinx BSP (`platform.h`, `xil_printf.h`, `xuartps.h`, `xparameters.h`).

---

## Build and run

1. **Create (or use) an SDK/Vitis application** for the Zybo Z7 design (ARM Cortex-A9, same hardware as `image_uart_Z7.sdk`).
2. **Add the desired source** to the project:
   - For **negative:** add `negative.c` (and remove or exclude other `main` sources, e.g. `main.c`).
   - For **histogram:** add `histogram.c` (and exclude other `main` sources).
   - For **equalized:** add `equalized.c` (and exclude other `main` sources).
3. **Include paths:** Ensure the BSP include path is set (e.g. `.../ps7_cortexa9_0/include`) so `xparameters.h`, `xuartps.h`, etc. resolve.
4. **Build** the application, **program** the board (bitstream + ELF), then **run**.
5. **Connect** a serial terminal to PS UART1 (115200 8N1) **before** starting the program. The server waits for the full PPM stream (blocking `XUartPs_RecvByte`).

---

## Testing

### Send and receive via UART

1. Open RealTerm; select the Zybo UART COM port, **115200 8N1**.
2. **Send file:** Send `original_file.ppm` (for negative) or your PPM (for histogram/equalized) as **binary**.
3. **Receive:** Capture the bytes the board sends back into a file (e.g. `output.ppm`).
