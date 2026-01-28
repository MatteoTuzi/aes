# Assignment 3 – DSP Audio Processing on Zybo Z7

**University of Cagliari – Advanced Embedded Systems (AES)**

This lab implements a **DSP application** that processes **audio signals** on the **Zybo Z7** board. The system acquires audio from the **LINE IN** jack, applies **Finite Impulse Response (FIR)** filters on the ARM-based Processing System (PS), and plays the result through the **HPH OUT** jack. Audio is handled by the **Analog Devices SSM2603** codec (ADC/DAC), configured via **I²C** and streamed over **I²S** through AXI-I2S and AXI FIFO IP.

---

## Overview

| Application | Source | Step | Description |
|-------------|--------|------|-------------|
| **Loopback** | `loopback/` | 1 | IN→OUT passthrough, Fs measurement via Global Timer |
| **Audio filtering** | `audio_filtering/` | 2 | FIR filtering, test-vector selftest, switch-selectable filters |
| **FIR execution time** | `audio_filtering_execution_time/` | 3 | FIR cycle timing, RT budget, cache on/off comparison |

- **Board:** Zybo Z7 (Zynq-7000, ARM Cortex-A9 @ 667 MHz).
- **Codec:** [SSM2603](https://www.analog.com/media/en/technical-documentation/data-sheets/SSM2603.pdf) — I²C control, I²S data.
- **Interfaces:** AXI-I2S ADI, AXI FIFO (MM2S/S2MM), PS I²C, ARM Global Timer.
- **Tools:** Xilinx SDK / Vitis, UART @ **115200 8N1** for debug output.

---

## Hardware & Data Path

- **LINE IN** → SSM2603 ADC → I²S → AXI-I2S RX → AXI FIFO RX → **ARM PS** (optional FIR) → AXI FIFO TX → AXI-I2S TX → I²S → SSM2603 DAC → **HPH OUT**.

Codec registers are written over **I²C** (100 kHz). Stereo samples are read/written via **blocking** I²S FIFO access: `FifoRxRead` / `FifoTxWrite` on the AXI FIFO mapped to the I²S controller.

---

## Step 1: Loopback & Fs Measurement

**Goal:** Check codec init, I²S acquire/playback, and **measure the actual sampling frequency** using the Zynq **Global Timer**.

### Implementation (`loopback/`)

- **`CodecInit`:** SCU timer, I²C driver init, SSM2603 register setup (`CodecRegWrite`), I²S clock divider, FIFO enable.
- **`FifoInit`:** AXI FIFO reset and interrupt/threshold setup for I²S streaming.
- **Main loop:** Read L/R from `FifoRxRead` → optionally measure Fs → write L/R to `FifoTxWrite` (passthrough).

### Fs Measurement

- **Global Timer:** 64‑bit counter @ **CPU_freq/2 ≈ 333 MHz**. Only the **LSB 32 bits** are used.
- **Registers:** `GTIMER_BASE + GTIMER_CTRL` (enable/auto-increment), `GTIMER_BASE + GTIMER_CNT_LO` (counter LSB).
- **Procedure:**
  1. Enable timer (`Xil_Out32(GTIMER_BASE + GTIMER_CTRL, 0x03)`).
  2. After `FS_WARMUP` loop iterations, for `N_SAMPLES_FS` stereo frames: read `t0 = Xil_In32(GTIMER_BASE + GTIMER_CNT_LO)`, read L/R, read `t1`, accumulate `Δt = t1 − t0`.
  3. `avg_Δt = sum(Δt) / N_SAMPLES_FS`, then **Fs = GLOBAL_TMR_FREQ / avg_Δt**.
- **Printing:** UART output is done **only after** the measurement so it does not affect timing.
- **Parameters:** `N_SAMPLES_FS` (e.g. 100), `FS_WARMUP` (e.g. 300). `GLOBAL_TMR_FREQ = CPU_FREQ_HZ / 2`.

---

## Step 2: FIR Audio Filtering

**Goal:** Implement a **parameterised FIR** over a **sliding window** of input samples, optionally verify with **test vectors**, then run **real-time** filtering with **switch-selectable** filters.

### FIR Model

- **Convolution:**  
  **y[n] = h₀ x[n] + h₁ x[n−1] + … + hₙ x[n−N]**

- Sliding window: `buffer[0]` = newest sample `x[n]`, `buffer[k]` = `x[n−k]`. Each new sample shifts the window and overwrites `buffer[0]`.

- **`FirApply(buffer, coeff, taps)`:**  
  Single-channel FIR; `coeff` and `taps` are configurable. Result truncated to `int` for I²S.

### Filters

- **Base LP/HP** (e.g. `coeffLP` / `coeffHP`, `N_LP` / `N_HP`).
- **Aggressive LP/HP** (`coeffLP_AGG` / `coeffHP_AGG`, `N_LP_AGG` / `N_HP_AGG`).  
  Coefficients are in the project headers; you can add more (e.g. from [t-filter](http://t-filter.engineerjs.com/)).

### Implementation (`audio_filtering/`)

1. **Selftest (test vectors):**  
   Run `FirSelftestLP` and `FirSelftestHP` on `TestVector.h` data (e.g. 250 Hz). Print `n`, `y`, `expected` per sample.
2. **Audio init:** `CodecInit`, `FifoInit`, zero L/R sliding buffers (`FIR_BUF_LEN` = 29).
3. **Real-time loop:**  
   Read L/R → update sliding windows → **switch** selects filter or passthrough → write L/R.

### Switch Mapping

| Switch | Effect |
|--------|--------|
| SW0 | Low‑pass (base) |
| SW1 | High‑pass (base) |
| SW2 | Low‑pass (aggressive) |
| SW3 | High‑pass (aggressive) |
| None | Passthrough |

Priority: SW0 → SW1 → SW2 → SW3. No switch ON ⇒ clean passthrough.

### Testing

- **Test vectors:** Use `TestVector.h` (e.g. 250 Hz) to check `FirApply` before live audio.
- **Live tones:** e.g. [Szynalski tone generator](https://www.szynalski.com/tone-generator/) on LINE IN; listen at HPH OUT and toggle switches to compare filtered vs clean.

---

## Step 3: FIR Execution-Time Measurement

**Goal:** Use the **Global Timer** to:

1. **Measure** FIR execution time (cycles per `FirApply` call).
2. **Compute** cycles available per sample at the **target sampling rate** (e.g. 48 kHz).
3. **Estimate** how many **filter taps** can run in **real time** without violating the sampling period.
4. **Repeat** with **caches disabled** (`Xil_ICacheDisable`, `Xil_DCacheDisable`) and compare.

### Implementation (`audio_filtering_execution_time/`)

- **`FirMeasureCycles(buf, coeff, taps, n_iter)`:**  
  Runs `FirApply` `n_iter` times, reads Global Timer LSB before/after each run, returns **average cycles** per call.
- **Real-time budget:**  
  `budget_cy = GTIMER_FREQ_HZ / FS_AUDIO_HZ` (e.g. 333e6 / 48000 ≈ **6937** cycles per sample @ 48 kHz).
- **Per-tap cost:**  
  `cpt = avg_cy / N_LP` (or similar). **Max RT-safe taps** ≈ `budget_cy / cpt`.
- **Cache OFF:**  
  Call `Xil_ICacheDisable()` and `Xil_DCacheDisable()`, then repeat the measurement; print cycles, per-tap cost, and max taps as with cache ON.

After timing, the app performs **audio init** and the same **real-time FIR loop** as in Step 2 (switch-based filters, passthrough).

---

## Project Structure

```
assignment_3/
├── README.md                          # This file
├── loopback/
│   └── src/
│       ├── loopback.c                 # Step 1: loopback + Fs
│       ├── loopback.h                 # Defines, declarations
│       ├── platform.c / platform.h
│       ├── timer_ps.c / timer_ps.h
│       └── lscript.ld
├── audio_filtering/
│   └── src/
│       ├── audio_filtering.c          # Step 2: FIR + selftest + live
│       ├── audio_filtering.h
│       ├── TestVector.h               # 250 Hz (and other) test vectors
│       ├── platform.c / platform.h
│       ├── timer_ps.c / timer_ps.h
│       └── lscript.ld
├── audio_filtering_execution_time/
│   └── src/
│       ├── audio_filtering_execution_time.c   # Step 3: timing + FIR
│       ├── audio_filtering_execution_time.h
│       ├── platform.c / platform.h
│       ├── timer_ps.c / timer_ps.h
│       └── lscript.ld
├── loopback_bsp/                      # BSP for loopback
├── audio_filtering_bsp/               # BSP for audio_filtering
└── audio_filtering_execution_time_bsp/ # BSP for Step 3
```

Each application has its own **`main()`** and is built as a **separate** SDK/Vitis project, typically with a shared hardware platform (e.g. design with AXI-I2S, AXI FIFO, GPIO for switches).

---

## Shared Functions

Used across the three projects (names and roles):

| Function | Role |
|----------|------|
| `CodecRegWrite` | Write one 9‑bit codec register over I²C |
| `CodecInit` | Timer, I²C, codec, I²S clock/FIFO init |
| `FifoRxRead` | Blocking read of one 32‑bit sample from I²S RX FIFO |
| `FifoTxWrite` | Blocking write of one 32‑bit sample to I²S TX FIFO |
| `FifoInit` | AXI FIFO reset and config for I²S |
| `FirApply` | Single-channel FIR convolution (Step 2 & 3) |

---

## Build & Run

1. **Hardware:** Export from Vivado (Zynq, AXI-I2S, AXI FIFO, GPIO, etc.) → create SDK/Vitis platform.
2. **BSP:** Create a BSP per app (or share one) with `xiicps`, `xil_io`, etc.
3. **Projects:** Create application projects for `loopback`, `audio_filtering`, `audio_filtering_execution_time`; add the corresponding `src` and `*.h`, and point to the platform.
4. **Run:** Program the Zybo, connect **LINE IN** / **HPH OUT**, open serial terminal @ 115200 8N1. Run the desired app from SDK/Vitis.

---

## UART Output (Summary)

- **Step 1:** `"Step 1 — Fs measurement + loopback"`, then `"Fs (avg over N samples) ~= X.XX Hz"`.
- **Step 2:** `"Step 2 — FIR Audio Filtering"`, selftest logs (`n | y | exp`), `"FIR self-test done."`, switch legend, then continuous filtering.
- **Step 3:** `"Step 3 — FIR timing (Global Timer)"`, then `[CACHE ON]` / `[CACHE OFF]` lines (cycles, per-tap cost, RT budget, max taps), then the same real-time FIR loop as Step 2.

---

## References

- [SSM2603 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/SSM2603.pdf)
- [Tone generator](https://www.szynalski.com/tone-generator/) – test tones
- [T-filter](http://t-filter.engineerjs.com/) – FIR coefficient design
