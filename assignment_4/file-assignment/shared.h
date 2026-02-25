/*
 * shared.h - Shared DDR layout for dual-core vector add (master + worker)
 * UniCa - Advanced Embedded Systems (AES)
 *
 * Single region: vectors A, B, C and sync flags. Base address must be
 * in the shared DDR range and equal on both cores.
 */

#ifndef SHARED_H
#define SHARED_H

#include "xil_types.h"

/* Vector length; change and rebuild both projects to test scaling */
#define ARRAY_SIZE  5000

/* Physical base of shared region (must match linker / memory map) */
#define SHARED_BASE  0x2F100000U

#define FLAG_RESET  0U
#define FLAG_SET    1U

typedef struct __attribute__((aligned(4))) {
    u32 A[ARRAY_SIZE];
    u32 B[ARRAY_SIZE];
    u32 C[ARRAY_SIZE];
    volatile u32 ready1;   /* Core 1 ready */
    volatile u32 start;    /* 0 = wait, 1 = go */
    volatile u32 done0;     /* Core 0 finished its half */
    volatile u32 done1;     /* Core 1 finished its half */
} shared_mem_t;

#define SHARED  ((volatile shared_mem_t *) SHARED_BASE)

#endif /* SHARED_H */
