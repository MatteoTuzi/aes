/***************************************************************
 *  Dual-core vector add (master / Core 0)
 *  ------------------------------------------------------------
 *  UniCa - Advanced Embedded Systems (AES)
 *
 *  Summary:
 *  ------------------------------------------------------------
 *  Measures the gain from data-parallel execution on a
 *  dual Cortex-A9 (Zynq) by comparing two runs of C = A + B:
 *
 *    1) Sequential run:
 *       - Single core (Core 0) computes the full vector
 *       - Serves as reference for timing
 *
 *    2) Concurrent run:
 *       - Core 0 handles indices [0, N/2)
 *       - Core 1 (worker) handles [N/2, N)
 *       - Coordination through flags in shared DDR
 *
 *  Cycle count is taken from the ARM Global Timer (lower 32 bits
 *  only); speedup is derived from serial vs parallel cycles.
 *
 ***************************************************************/

#include "xparameters.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "shared.h"
#include "xil_io.h"
#include "xtime_l.h"



/* Returns the lower 32 bits of the ARM Global Timer (cycle counter). */
static inline u32 read_global_timer(void)
{
    return Xil_In32(XPAR_GLOBAL_TMR_BASEADDR + 0x00U);
}

/* Vector add: c[i] = a[i] + b[i] for i in [0, n). Uses restrict for optimization. */
void vec_add_serial(u32 * restrict c, const u32 * restrict a, const u32 * restrict b, int n) {
    for (int i = 0; i < n; i++) {
        c[i] = a[i] + b[i];
    }
}

/* Busy-wait until the GPIO (button) reads non-zero. */
static void wait_gpio(void)
{
    while (Xil_In32(XPAR_AXI_GPIO_2_BASEADDR) == 0)
        ;
}

/* Reset sync flags and fill A, B in shared memory; clear C. */
static void init_shared_and_vectors(void)
{
    int i;
    SHARED->start  = FLAG_RESET;
    SHARED->done0  = FLAG_RESET;
    SHARED->done1  = FLAG_RESET;
    SHARED->ready1 = FLAG_RESET;
    for (i = 0; i < ARRAY_SIZE; ++i) {
        SHARED->A[i] = (u32)i;
        SHARED->B[i] = (u32)(2 * i);
        SHARED->C[i] = 0;
    }
}

/* Wait for worker ready, run Core 0 half, wait for worker done. Returns parallel cycle count. */
static u32 run_parallel_section(void)
{
    u32 t0, t1;
    int i;
    while (SHARED->ready1 != FLAG_SET)
        ;
    t0 = read_global_timer();
    SHARED->start = FLAG_SET;
    for (i = 0; i < ARRAY_SIZE / 2; ++i)
        SHARED->C[i] = SHARED->A[i] + SHARED->B[i];
    SHARED->done0 = FLAG_SET;
    while (SHARED->done1 != FLAG_SET)
        ;
    t1 = read_global_timer();
    return (t1 - t0) ? (t1 - t0) : 1;
}

/* Print serial/parallel cycles and speedup (2 decimal places) to UART. */
static void print_timing(u32 cy_serial, u32 cy_parallel)
{
    u32 sp = (cy_serial * 100) / cy_parallel;
    xil_printf("\r\n[Timing] serial=%lu parallel=%lu speedup=%lu.%02lux\r\n",
               (unsigned long)cy_serial, (unsigned long)cy_parallel,
               sp / 100, sp % 100);
}

/* Compare SHARED->C with ref; return number of mismatches. */
static int validate_against_ref(const u32 *ref)
{
    int i, err = 0;
    for (i = 0; i < ARRAY_SIZE; ++i)
        if (SHARED->C[i] != ref[i])
            err++;
    return err;
}

/* ============================================================
 * MAIN – MASTER CORE (Core 0)
 * ============================================================ */
int main(void)
{
    static u32 ref_C[ARRAY_SIZE];  /* reference result from serial run */
    u32 cy_serial, cy_parallel;
    u32 t0, t1;
    int i, errs;

    /* Disable caches for deterministic timing and shared-memory visibility */
    Xil_ICacheDisable();
    Xil_DCacheDisable();

    xil_printf("[master] boot, N=%d\r\n", ARRAY_SIZE);

    /* Wait for user to press button before starting the experiment */
    wait_gpio();
    init_shared_and_vectors();

    /* Run serial baseline and save result for later validation */
    xil_printf("[master] serial run...\r\n");
    t0 = read_global_timer();
    vec_add_serial(ref_C, (const u32 *)SHARED->A, (const u32 *)SHARED->B, ARRAY_SIZE);
    t1 = read_global_timer();
    for (i = 0; i < ARRAY_SIZE; ++i)
        SHARED->C[i] = 0;
    cy_serial = t1 - t0;
    xil_printf("[master] serial done, cy=%lu\r\n", (unsigned long)cy_serial);

    /* Run parallel section (Core 0 + Core 1) and get its cycle count */
    xil_printf("[master] parallel run...\r\n");
    cy_parallel = run_parallel_section();
    xil_printf("[master] parallel done.\r\n");

    /* Print timing summary and check parallel result against serial reference */
    print_timing(cy_serial, cy_parallel);
    errs = validate_against_ref(ref_C);
    xil_printf("[master] mismatches=%d\r\n", errs);

    /* Idle loop; no further work */
    for (;;)
        ;
    return 0;
}
