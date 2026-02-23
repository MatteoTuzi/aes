/*
 * Dual-core vector add (worker / Core 1)
 * ------------------------------------------------------------
 * UniCa - Advanced Embedded Systems (AES)
 *
 * Core 1 computes C[i] = A[i] + B[i] for i in [ARRAY_SIZE/2, ARRAY_SIZE).
 * Signals ready1, then waits for start from Core 0; on completion sets done1.
 */

#include "xparameters.h"
#include "xil_cache.h"
#include "shared.h"
#include "xil_io.h"

/* Reset flags owned by Core 1 (ready1, done1). Do not touch start (master-owned). */
static void init_worker_flags(void)
{
    SHARED->ready1 = FLAG_RESET;
    SHARED->done1  = FLAG_RESET;
}

/* Busy-wait until master sets start, then compute second half of C and set done1. */
static void run_second_half(void)
{
    int i;
    while (SHARED->start != FLAG_SET)
        ;
    for (i = ARRAY_SIZE / 2; i < ARRAY_SIZE; ++i)
        SHARED->C[i] = SHARED->A[i] + SHARED->B[i];
    SHARED->done1 = FLAG_SET;
}

/* ============================================================
 * MAIN ? WORKER CORE (Core 1)
 * ============================================================ */
int main(void)
{
    /* Disable caches for deterministic timing and shared-memory visibility */
    Xil_ICacheDisable();
    Xil_DCacheDisable();

    init_worker_flags();
    /* Signal ready so master can proceed; no GPIO wait here */
    SHARED->ready1 = FLAG_SET;

    run_second_half();

    /* Idle loop; no further work */
    for (;;)
        ;
    return 0;
}
