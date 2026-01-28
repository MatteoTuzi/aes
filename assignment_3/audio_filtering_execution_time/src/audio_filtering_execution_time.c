/******************************************************************************
 * Lab 3 – Step 3: FIR Execution-Time Analysis
 * University of Cagliari – Advanced Embedded Systems (AES)
 ******************************************************************************
 *
 * OVERVIEW
 * --------
 * Cycle-level timing of the software FIR filter on the ARM Cortex-A9 (Zybo Z7).
 * Extends Step 2 by measuring FirApply() execution time and assessing
 * real-time feasibility under I²S audio streaming.
 *
 * STEP 3 OBJECTIVES
 * -----------------
 *   • Measure execution time of FirApply()
 *   • Compute cycles available per sample (Fs ≈ 48 kHz)
 *   • Estimate max FIR taps executable in real time (no sampling-period violation)
 *   • Compare performance with caches enabled vs disabled
 *   • Ensure filtering keeps up with I²S data rate
 *
 * TIMING (ARM Global Timer)
 * ------------------------
 *   • 64-bit counter @ CPU_Freq/2 ≈ 333 MHz; only LSB 32 bits are used.
 *   • Procedure:
 *       1. Read Global Timer LSB → t_start
 *       2. Call FirApply() once
 *       3. Read Global Timer LSB → t_end
 *       4. Δ = t_end − t_start (cycles)
 *   • Cache-disabled worst-case: Xil_ICacheDisable(); Xil_DCacheDisable();
 *     then repeat the measurement.
 *
 * REAL-TIME CONSTRAINT
 * --------------------
 *   cycles_per_sample = 333e6 / 48000 ≈ 6937 cycles
 *   This bounds the maximum FIR complexity sustainable at 48 kHz.
 *
 * PLATFORM
 * --------
 *   Board: Zybo Z7 (Zynq-7000)
 *   Codec: Analog Devices SSM2603 (I²C setup, I²S data)
 *   Interfaces: AXI-I2S, AXI FIFO, PS I²C
 *   Tools: Xilinx SDK/Vitis, UART @ 115200 8N1
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xiicps.h"
#include "timer_ps.h"
#include "audio_filtering_execution_time.h"

/*
 * LP[], HP[], LP_AGG[], HP_AGG[] — FIR coefficient arrays.
 * Definitions (storage allocated here). Extern declarations are in the .h.
 * Used by FirApply() and by the switch-based filter selection in main().
 */
float LP[] = { coeffLP };
float HP[] = { coeffHP };
float LP_AGG[] = { coeffLP_AGG };
float HP_AGG[] = { coeffHP_AGG };

/* Iic — global I²C controller instance. Used by CodecRegWrite and CodecInit. */
XIicPs Iic;

/*******************************************************************************
 * CodecRegWrite
 * Writes a single 9-bit register value to the SSM2603 codec over I²C.
 * Encodes regAddr and regData into a 2-byte frame and sends it via
 * XIicPs_MasterSendPolled. Blocks until the bus is idle before returning.
 *
 * IIcPtr — I²C driver instance (typically &Iic).
 * regAddr — codec register index (0..15).
 * regData — 9-bit value to write (lower 9 bits used).
 * Returns XST_SUCCESS on success, XST_FAILURE on I²C error.
 ******************************************************************************/
int CodecRegWrite(XIicPs *IIcPtr, u8 regAddr, u16 regData)
{
    int Status;       /* XST_SUCCESS / XST_FAILURE from I²C APIs */
    u8 SendBuffer[2]; /* I²C payload: [addr|msb of data], [lsb of data] */

    /* Upper byte: 7-bit reg addr (left-shifted) + MSB of regData */
    SendBuffer[0] = regAddr << 1;
    SendBuffer[0] = SendBuffer[0] | ((regData >> 8) & 0b1);
    SendBuffer[1] = regData & 0xFF;

    Status = XIicPs_MasterSendPolled(IIcPtr, SendBuffer, 2, IIC_SLAVE_ADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC send failed\n\r");
        return XST_FAILURE;
    }
    /* Block until I²C bus is free before starting another transaction */
    while (XIicPs_BusIsBusy(IIcPtr)) {
        /* NOP */
    }
    return XST_SUCCESS;
}

/*******************************************************************************
 * CodecInit
 * One-time setup of the audio path: SCU timer, I²C driver, SSM2603 codec
 * registers, and I²S clock/FIFO control. Must be called before any I²S
 * read/write. Uses CodecRegWrite() to configure the codec; then programs
 * I²S clock divider and enables RX/TX FIFOs.
 *
 * timerID — SCU timer device ID (for TimerDelay).
 * iicID   — PS I²C device ID connected to the codec.
 * i2sAddr — base address of the I²S controller (clock config).
 * Returns XST_SUCCESS on success, XST_FAILURE on any init error.
 ******************************************************************************/
int CodecInit(u16 timerID, u16 iicID, u32 i2sAddr)
{
    int Status;           /* accumulates init errors */
    XIicPs_Config *Config;/* I²C config from lookup table */
    u32 i2sClkDiv;       /* BCLK/LRCLK divider for I²S (MCLK/4, LRCLK = BCLK/64) */

    TimerInitialize(timerID);

    Config = XIicPs_LookupConfig(iicID);
    if (NULL == Config)
        return XST_FAILURE;

    Status = XIicPs_CfgInitialize(&Iic, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS)
        return XST_FAILURE;

    Status = XIicPs_SelfTest(&Iic);
    if (Status != XST_SUCCESS)
        return XST_FAILURE;

    Status = XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);
    if (Status != XST_SUCCESS)
        return XST_FAILURE;

    Status = CodecRegWrite(&Iic, 15, 0b000000000);
    TimerDelay(75000);
    Status |= CodecRegWrite(&Iic, 6, 0b000110000);
    Status |= CodecRegWrite(&Iic, 0, 0b000010111);
    Status |= CodecRegWrite(&Iic, 1, 0b000010111);
    Status |= CodecRegWrite(&Iic, 2, 0b101111001);
    Status |= CodecRegWrite(&Iic, 4, 0b000010000);
    Status |= CodecRegWrite(&Iic, 5, 0b000000000);
    Status |= CodecRegWrite(&Iic, 7, 0b000001010);
    Status |= CodecRegWrite(&Iic, 8, 0b000000000);
    TimerDelay(75000);
    Status |= CodecRegWrite(&Iic, 9, 0b000000001);
    Status |= CodecRegWrite(&Iic, 6, 0b000100000);
    Status = CodecRegWrite(&Iic, 4, 0b000010000);

    if (Status != XST_SUCCESS)
        return XST_FAILURE;

    i2sClkDiv = 1 | (31 << 16);
    Xil_Out32(i2sAddr + I2S_CLK_CTRL_REG, i2sClkDiv);
    Xil_Out32(AUDIO_CTRL_BASEADDR + I2S_RESET_REG, 0b110);
    Xil_Out32(AUDIO_CTRL_BASEADDR + I2S_CTRL_REG, 0b011);
    return XST_SUCCESS;
}

/*******************************************************************************
 * FifoTxWrite
 * Pushes one 32-bit audio sample into the AXI FIFO TX path. Writes the
 * sample to the FIFO data register, then the length (4 bytes). Blocks until
 * the "transmit complete" flag is set, then clears it (toggle-on-write ack).
 *
 * i2sBaseAddr — base address of the AXI FIFO used for I²S TX.
 * audioData   — 32-bit sample to send (e.g. left or right channel).
 ******************************************************************************/
void FifoTxWrite(u32 i2sBaseAddr, u32 audioData)
{
    Xil_Out32(i2sBaseAddr + 0x10, audioData);
    Xil_Out32(i2sBaseAddr + 0x14, 4);
    /* Wait until TX completion flag is set */
    while ((Xil_In32(i2sBaseAddr + 0x00) & 0x08000000) != 0x08000000)
        ;
    /* Clear the flag (ack) */
    Xil_Out32(i2sBaseAddr + 0x00, 0x08000000);
}

/*******************************************************************************
 * FifoRxRead
 * Blocks until at least one 32-bit sample is available in the AXI FIFO RX
 * path, then reads and returns it. Used to consume incoming I²S samples
 * (typically left, then right per stereo pair).
 *
 * i2sBaseAddr — base address of the AXI FIFO used for I²S RX.
 * Returns the next 32-bit sample from the FIFO.
 ******************************************************************************/
u32 FifoRxRead(u32 i2sBaseAddr)
{
    /* Wait until RX FIFO has data (count > 0) */
    while (Xil_In32(i2sBaseAddr + 0x1C) == 0)
        ;
    return (u32)Xil_In32(i2sBaseAddr + 0x20);
}

/*******************************************************************************
 * FifoInit
 * Resets and configures the AXI FIFO: clear ISR/IER, set up interrupt
 * enable and threshold-related control. Prepares the FIFO for I²S
 * stream tx/rx. fifoAddr is the FIFO base; AUDIO_FIFO is also used for
 * the 0x2c write (MM2S reset or similar).
 *
 * fifoAddr — base address of the AXI FIFO to initialize.
 ******************************************************************************/
void FifoInit(u32 fifoAddr)
{
    Xil_Out32(AUDIO_FIFO + 0x2c, 0);
    Xil_Out32(fifoAddr + FIFO_ISR, 0xFFFFFFFF);
    Xil_Out32(fifoAddr + FIFO_IER, 0x0C000000);
    Xil_Out32(fifoAddr + FIFO_TDR, 0x00000000);
    Xil_Out32(fifoAddr + FIFO_ISR, 0xFFFFFFFF);
    Xil_Out32(fifoAddr + FIFO_IER, 0x04100000);
    Xil_Out32(fifoAddr + FIFO_ISR, 0x00100000);
}

/*******************************************************************************
 * FirApply
 * Single-channel FIR convolution: y[n] = sum_{k=0}^{taps-1} h[k] * x[n-k].
 * buffer[0] is the newest sample x[n], buffer[k] is x[n-k]. Coefficients
 * are in coeff[0..taps-1]. Result is accumulated in float and truncated
 * to int for the I²S integer path.
 *
 * buffer — sliding-window of delayed samples (newest at index 0).
 * coeff  — FIR coefficients h[k].
 * taps   — filter length (number of coefficients).
 * Returns the filtered sample as an integer.
 ******************************************************************************/
int FirApply(int *buffer, float *coeff, int taps)
{
    float acc = 0.0f; /* accumulator for sum h[k]*x[n-k] */
    int k;

    /* Convolution loop: for each tap k, add coeff[k] * buffer[k] (x[n-k]) */
    for (k = 0; k < taps; k++)
        acc += coeff[k] * (float)buffer[k];
    return (int)acc;
}

/*******************************************************************************
 * FirMeasureCycles
 * Runs FirApply() n_iter times on the given buffer/coeff/taps and
 * measures execution time using the Global Timer LSB. Returns the average
 * cycle count per FIR call. Used for Step 3 timing (cache on/off).
 *
 * buf    — dummy input buffer (values not used; only timing matters).
 * coeff  — coefficient array (e.g. LP).
 * taps   — filter length (e.g. N_LP).
 * n_iter — number of iterations to average.
 * Returns average cycles per FirApply() call.
 ******************************************************************************/
static u32 FirMeasureCycles(int *buf, float *coeff, int taps, int n_iter)
{
    u64 sum = 0;  /* sum of (t1 - t0) over all iterations */
    u32 t0, t1;   /* Global Timer LSB before and after FirApply() */
    int i;

    /* Loop: read timer, run FIR, read timer again, accumulate cycle delta */
    for (i = 0; i < n_iter; i++) {
        t0 = Xil_In32(GTIMER_BASE + GTIMER_CNT_LO);
        (void)FirApply(buf, coeff, taps);
        t1 = Xil_In32(GTIMER_BASE + GTIMER_CNT_LO);
        sum += (u64)(t1 - t0);
    }
    return (u32)(sum / (u64)n_iter);
}

/*******************************************************************************
 * main
 * 1) Platform init, then Step 3 timing: enable Global Timer, measure FIR
 *    with caches on (print cycles, per-tap cost, RT budget, max RT-safe taps).
 * 2) Disable I/D caches, repeat measurement, print same metrics (cache off).
 * 3) Audio init (codec + FIFO), zero the L/R sliding-window buffers.
 * 4) Infinite loop: read L/R samples from I²S RX FIFO, update sliding
 *    windows, select filter from switches (SW0–SW3 → LP/HP/LP_AGG/HP_AGG
 *    or passthrough), write filtered/passthrough L/R to I²S TX FIFO.
 ******************************************************************************/
int main(void)
{
    int timing_buf[FIR_BUF_LEN];              /* dummy buffer for FIR timing */
    int bufferL[FIR_BUF_LEN], bufferR[FIR_BUF_LEN]; /* sliding windows L/R */
    u32 budget_cy, avg_cy, cpt, max_taps;     /* timing: RT budget, avg cy, per-tap, max taps */
    u32 sw;                                   /* GPIO switch bits (SW0–SW3) */
    int SampleL, SampleR, outL, outR;         /* current samples in, filtered out */
    int i;

    init_platform();

    xil_printf("Step 3 — FIR timing (Global Timer)\n\r");
    sleep(1);

    /* Enable Global Timer (bit0 enable, bit1 auto-increment) */
    Xil_Out32(GTIMER_BASE + GTIMER_CTRL, 0x03);
    for (i = 0; i < FIR_BUF_LEN; i++)
        timing_buf[i] = 0;  /* zero dummy buffer used only for timing */

    budget_cy = GTIMER_FREQ_HZ / FS_AUDIO_HZ; /* cycles per sample @ 48 kHz */

    avg_cy = FirMeasureCycles(timing_buf, LP, N_LP, FIR_MEAS_ITER);
    cpt = (avg_cy / N_LP) + 1;
    max_taps = budget_cy / cpt;
    xil_printf("[CACHE ON] FIR exec (avg %d runs): %u cy\n\r", FIR_MEAS_ITER, avg_cy);
    xil_printf("[CACHE ON] Per-tap cost: %u cy  |  RT budget @48kHz: %u cy/sample\n\r", cpt, budget_cy);
    xil_printf("[CACHE ON] Max taps (RT-safe): ~%u\n\r", max_taps);
    sleep(1);

    Xil_ICacheDisable();
    Xil_DCacheDisable();
    avg_cy = FirMeasureCycles(timing_buf, LP, N_LP, FIR_MEAS_ITER);
    cpt = avg_cy / N_LP;
    max_taps = budget_cy / cpt;
    xil_printf("\n[CACHE OFF] FIR exec (avg %d runs): %u cy\n\r", FIR_MEAS_ITER, avg_cy);
    xil_printf("[CACHE OFF] Per-tap cost: %u cy  |  RT budget @48kHz: %u cy/sample\n\r", cpt, budget_cy);
    xil_printf("[CACHE OFF] Max taps (RT-safe): ~%u\n\r", max_taps);
    sleep(1);

    CodecInit(SCU_TIMER_ID, AUDIO_IIC_ID, AUDIO_CTRL_BASEADDR);
    FifoInit(AUDIO_FIFO);

    /* Cold start: zero both sliding-window buffers before first sample */
    for (i = 0; i < FIR_BUF_LEN; i++) {
        bufferL[i] = 0;
        bufferR[i] = 0;
    }

    while (1) {
        SampleL = (int)FifoRxRead(AUDIO_FIFO);
        SampleR = (int)FifoRxRead(AUDIO_FIFO);

        /* Sliding window: shift old samples right; insert newest at [0] */
        for (i = FIR_BUF_LEN - 1; i > 0; i--) {
            bufferL[i] = bufferL[i - 1];
            bufferR[i] = bufferR[i - 1];
        }
        bufferL[0] = SampleL;
        bufferR[0] = SampleR;

        sw = Xil_In32(SWI_BASE_ADDR);
        /* SW0→LP, SW1→HP, SW2→LP_AGG, SW3→HP_AGG; else passthrough */
        if (sw & 0x1) {
            outL = FirApply(bufferL, LP, N_LP);
            outR = FirApply(bufferR, LP, N_LP);
        } else if (sw & 0x2) {
            outL = FirApply(bufferL, HP, N_HP);
            outR = FirApply(bufferR, HP, N_HP);
        } else if (sw & 0x4) {
            outL = FirApply(bufferL, LP_AGG, N_LP_AGG);
            outR = FirApply(bufferR, LP_AGG, N_LP_AGG);
        } else if (sw & 0x8) {
            outL = FirApply(bufferL, HP_AGG, N_HP_AGG);
            outR = FirApply(bufferR, HP_AGG, N_HP_AGG);
        } else {
            outL = SampleL;
            outR = SampleR;
        }

        FifoTxWrite(AUDIO_FIFO, outL);
        FifoTxWrite(AUDIO_FIFO, outR);
    }

    cleanup_platform();
    return 0;
}
