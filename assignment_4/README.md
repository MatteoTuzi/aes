# Assignment 4 — Dual-core vector addition and timing

**UniCa · Advanced Embedded Systems (AES)**

---

## Purpose

This assignment uses the two ARM Cortex-A9 cores on the Zynq in a bare-metal application: the sum of two vectors (C = A + B) is computed first sequentially on a single core, then by splitting the work between Core 0 and Core 1. The goal is to compare execution times and derive a speedup, using the ARM Global Timer as the cycle reference.

---

## Platform and tools

- **Board:** Zybo Z7 (Zynq-7000, dual Cortex-A9).
- **Environment:** Xilinx SDK or Vitis; serial terminal 115200 8N1.
- **Peripherals used:** Shared DDR (for A, B, C and flags), Global Timer (cycle count), AXI GPIO (start button), UART (log from master).

Both cores share a fixed-address DDR region; each core has its own linker script that places code and data in dedicated segments (DDR Core0 / DDR Core1) without overlapping the shared region.

---

## Software architecture

**Master (Core 0)** — After waiting for the button, it initializes vectors A and B in shared memory and the synchronization flags. It runs the serial vector add and records its cycle count (baseline). Then it clears C, waits for the worker to signal readiness (`ready1`), starts the parallel phase by setting `start`, computes the first half of C, waits for the worker to finish (`done1`), reads the timer again and computes the parallel-phase cycles. Finally it computes the speedup and checks that vector C matches the serial result.

**Worker (Core 1)** — It initializes its flags (`ready1`, `done1`), immediately signals that it is ready (`ready1 = 1`), then waits for `start`. When the master sets `start`, it computes the second half of C (indices from ARRAY_SIZE/2 to ARRAY_SIZE-1), sets `done1` and enters an idle loop.

Coordination is done only via flags in shared memory (busy-wait); no OS or threading libraries are used.

---

## Shared memory

The shared region is mapped at a fixed address (e.g. `0x2F100000`), defined in `shared.h` as `SHARED_BASE`. The structure contains:

- Vectors **A**, **B**, **C** (length `ARRAY_SIZE`).
- Volatile flags **ready1**, **start**, **done0**, **done1**.

This address must be excluded from both applications’ linker scripts (no .text, .data, .bss, heap or stack section may use it), so that neither core overwrites it.

---

## Timing measurement

The Cortex-A9 **Global Timer** is used (32-bit register, typically read via `XPAR_GLOBAL_TMR_BASEADDR + 0x00`). For each phase (serial and parallel) the timer is read immediately before and after the code under measurement; the difference gives the cycle count. The timer frequency is tied to the system clock (e.g. about 333 MHz); results are reported in cycles, with no conversion to seconds. Caches are disabled for deterministic timing and immediate visibility of shared-memory updates.

---

## Project configuration

- **Build:** Two separate applications, `master` and `worker`, each with its own BSP and the same `shared.h` (same `ARRAY_SIZE` and struct layout). Use **optimisation -O3** for both (Project → Properties → C/C++ Build → Settings → Optimization) so that the serial loop is vectorised and timing is comparable to the report.
- **Linker:** Link scripts must place Core 0 in one DDR region (e.g. ddrCore0) and Core 1 in another (e.g. ddrCore1), leaving the shared-memory address range (e.g. around `0x2F100000`) unused by any section.
- **Run:** Use a **System Debugger** configuration that loads `master.elf` on Core 0 (ps7_cortexa9_0) and `worker.elf` on Core 1 (ps7_cortexa9_1), with FPGA programming and system reset enabled, so both cores start correctly.

---

## Execution and testing

1. Build master and worker, then start a session with System Debugger.
2. Open the serial terminal (115200 8N1); output comes from the master.
3. Press the button connected to the GPIO used in the code; the master leaves the wait, runs the serial phase first, then the parallel one.
4. Output shows: array size, serial run cycles, parallel run cycles, speedup (e.g. x.xx) and the number of validation errors (0 if the parallel result matches the serial one).

To try different sizes, change `ARRAY_SIZE` in both `shared.h` files, rebuild and run again.

---

## Interpreting the results

On systems with shared DDR and no private caches enabled, a kernel like vector add is typically **memory-bound**: execution time is dominated by DDR access. Splitting the work across two cores that share the same memory may not yield a speedup close to 2x, because memory bandwidth is shared. With very small arrays, synchronization overhead can be comparable to compute time. Record cycles (serial, parallel) and speedup for several `ARRAY_SIZE` values (e.g. 10, 100, 1000, 5000) and discuss them in the report.

---

## Directory structure

```
assignment_4/
├── README.md
├── report/
│   ├── assignment4_report.tex
│   ├── assignment4_report.pdf
│   └── figures/
├── master/
│   └── src/
│       ├── master.c
│       └── shared.h
├── worker/
│   └── src/
│       ├── worker.c
│       └── shared.h
├── master_bsp/
└── worker_bsp/
```

---

## References

- Zynq-7000 Technical Reference Manual (Global Timer, memory map)
- Xilinx SDK / Vitis: System Debugger and dual-core run configuration
