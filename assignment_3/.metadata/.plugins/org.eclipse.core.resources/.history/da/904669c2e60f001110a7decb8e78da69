// shared.h
#ifndef SHARED_H
#define SHARED_H

#include "xil_types.h"

#define ARRAY_SIZE 10

/*
 * Choose a DDR address that is:
 * - Inside your DDR memory range
 * - Not used by your linker script / BSS / heap / stack
 *
 * 0x00100000 is a common safe choice on many Zynq setups,
 * but double-check your memory map and adjust if needed.
 */
#define SHARED_BASE 0x2f100000U

typedef struct {
    u32 A[ARRAY_SIZE];
    u32 B[ARRAY_SIZE];
    u32 C[ARRAY_SIZE];

    volatile u32 start;  // 0 = wait, 1 = go
    volatile u32 done0;  // set by core 0
    volatile u32 done1;  // set by core 1
} shared_mem_t;

#define SHARED ((volatile shared_mem_t *)SHARED_BASE)

#endif // SHARED_H
