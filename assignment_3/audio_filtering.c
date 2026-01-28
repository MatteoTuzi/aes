/******************************************************************************
 * Lab 3 – Step 2: FIR Audio Filtering
 * University of Cagliari – Advanced Embedded Systems (AES)
 ******************************************************************************
 *
 * OVERVIEW
 * --------
 * Real-time FIR audio filtering on the Zybo Z7. Acquires stereo samples
 * (L/R) from the SSM2603 codec via AXI-I2S, stores them in sliding
 * windows, applies a switch-selectable FIR, and sends filtered (or
 * passthrough) samples to the I²S TX FIFO.
 *
 * STEP 2 OBJECTIVES
 * -----------------
 *   • Implement a generic FIR filter (y[n] = sum h[k]*x[n-k])
 *   • Use sliding-window buffers for past samples
 *   • Select filter via board switches (SW0–SW3)
 *   • Support LP/HP base and aggressive kernels; passthrough when none selected
 *
 * FILTER SELECTION
 * ----------------
 *   SW0 = 1 → Low-Pass (base)     SW1 = 1 → High-Pass (base)
 *   SW2 = 1 → Low-Pass (aggressive)  SW3 = 1 → High-Pass (aggressive)
 *   No switch ON → raw passthrough. Priority: SW0 → SW1 → SW2 → SW3.
 *
 * PLATFORM
 * --------
 *   Board: Zybo Z7 (Zynq-7000). Codec: SSM2603 (I²C, I²S).
 *   AXI-I2S, AXI FIFO, PS I²C. UART @ 115200 8N1.
 *
 * FLOW
 * ----
 *   1. Run FIR selftest (TestVector.h, 250 Hz LP/HP): print expected vs actual.
 *   2. Initialize audio (codec, FIFO), then real-time loop (switch-based
 *      filters or passthrough). Both phases always executed.
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
#include "audio_filtering.h"
#include "TestVector.h"

#define TEST_LEN_250          (sizeof(test_input) / sizeof(test_input[0]))

static int test_input[] = { inputTest_250 };
static int test_output_LP[] = { outputTest_F_250_LP };
static int test_output_HP[] = { outputTest_F_250_HP };

/* FIR coefficient arrays (definitions; extern in .h). Used by FirApply and switch logic. */
float LP[] = { coeffLP };
float HP[] = { coeffHP };
float LP_AGG[] = { coeffLP_AGG };
float HP_AGG[] = { coeffHP_AGG };

XIicPs Iic;   /* I²C instance for SSM2603 */

/*******************************************************************************
 * CodecRegWrite
 * Writes a single 9-bit register value to the SSM2603 codec over I²C.
 * Encodes regAddr and regData into a 2-byte frame, sends via
 * XIicPs_MasterSendPolled, blocks until bus idle.
 ******************************************************************************/
int CodecRegWrite(XIicPs *IIcPtr, u8 regAddr, u16 regData)
{
    int Status;
    u8 SendBuffer[2];   /* I²C payload: [addr|msb], [lsb] */

    SendBuffer[0] = regAddr << 1;
    SendBuffer[0] = SendBuffer[0] | ((regData >> 8) & 0b1);
    SendBuffer[1] = regData & 0xFF;

    Status = XIicPs_MasterSendPolled(IIcPtr, SendBuffer, 2, IIC_SLAVE_ADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC send failed\n\r");
        return XST_FAILURE;
    }
    while (XIicPs_BusIsBusy(IIcPtr)) {
        /* wait until bus idle */
    }
    return XST_SUCCESS;
}

/*******************************************************************************
 * CodecInit
 * One-time setup: SCU timer, I²C driver, SSM2603 registers, I²S clock/FIFO.
 * Must be called before any I²S read/write.
 ******************************************************************************/
int CodecInit(u16 timerID, u16 iicID, u32 i2sAddr)
{
    int Status;
    XIicPs_Config *Config;
    u32 i2sClkDiv;

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
 * Pushes one 32-bit sample into the AXI FIFO TX path. Blocks until
 * transmit-complete flag, then clears it.
 ******************************************************************************/
void FifoTxWrite(u32 i2sBaseAddr, u32 audioData)
{
    Xil_Out32(i2sBaseAddr + 0x10, audioData);
    Xil_Out32(i2sBaseAddr + 0x14, 4);
    while ((Xil_In32(i2sBaseAddr + 0x00) & 0x08000000) != 0x08000000)
        ;
    Xil_Out32(i2sBaseAddr + 0x00, 0x08000000);
}

/*******************************************************************************
 * FifoRxRead
 * Blocks until a 32-bit sample is available in the AXI FIFO RX path,
 * then reads and returns it.
 ******************************************************************************/
u32 FifoRxRead(u32 i2sBaseAddr)
{
    while (Xil_In32(i2sBaseAddr + 0x1C) == 0)
        ;
    return (u32)Xil_In32(i2sBaseAddr + 0x20);
}

/*******************************************************************************
 * FifoInit
 * Resets and configures the AXI FIFO (ISR/IER, etc.) for I²S streaming.
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
 * Single-channel FIR: y[n] = sum_{k=0}^{taps-1} h[k]*x[n-k]. buffer[0] = x[n],
 * buffer[k] = x[n-k]. Result truncated to int for I²S.
 ******************************************************************************/
int FirApply(int *buffer, float *coeff, int taps)
{
    float acc = 0.0f;
    int k;

    /* Convolution: for each tap k, add coeff[k] * buffer[k] (x[n-k]) */
    for (k = 0; k < taps; k++)
        acc += coeff[k] * (float)buffer[k];
    return (int)acc;
}

/*******************************************************************************
 * FirSelftestLP
 * Feeds test_input[] through FirApply with LP kernel. Prints n, y, expected
 * per sample. Run before real-time loop.
 ******************************************************************************/
static void FirSelftestLP(void)
{
    int buffer[N_LP];
    int n, y, i;

    xil_printf("\n=== FIR SELF-TEST LP (250 Hz) ===\n\r");
    for (i = 0; i < N_LP; i++)
        buffer[i] = 0;

    for (n = 0; n < (int)TEST_LEN_250; n++) {
        for (i = N_LP - 1; i > 0; i--)
            buffer[i] = buffer[i - 1];
        buffer[0] = test_input[n];
        y = FirApply(buffer, LP, N_LP);
        xil_printf("n=%3d | y=%6d | exp=%6d\n\r", n, y, test_output_LP[n]);
    }
}

/*******************************************************************************
 * FirSelftestHP
 * Same as FirSelftestLP but uses HP kernel and test_output_HP[].
 ******************************************************************************/
static void FirSelftestHP(void)
{
    int buffer[N_HP];
    int n, y, i;

    xil_printf("\n=== FIR SELF-TEST HP (250 Hz) ===\n\r");
    for (i = 0; i < N_HP; i++)
        buffer[i] = 0;

    for (n = 0; n < (int)TEST_LEN_250; n++) {
        for (i = N_HP - 1; i > 0; i--)
            buffer[i] = buffer[i - 1];
        buffer[0] = test_input[n];
        y = FirApply(buffer, HP, N_HP);
        xil_printf("n=%3d | y=%6d | exp=%6d\n\r", n, y, test_output_HP[n]);
    }
}

/*******************************************************************************
 * main
 * Platform init, Step 2 banner. Run FIR selftest (LP then HP), then
 * audio init, zero L/R buffers, real-time loop (read L/R, sliding window,
 * SW0–SW3 filter or passthrough, write L/R).
 ******************************************************************************/
int main(void)
{
    int bufferL[FIR_BUF_LEN], bufferR[FIR_BUF_LEN];
    u32 sw;
    int SampleL, SampleR, outL, outR;
    int i;

    init_platform();

    xil_printf("Step 2 — FIR Audio Filtering\n\r");
    xil_printf("FIR selftest (250 Hz LP/HP)...\n\r");
    FirSelftestLP();
    FirSelftestHP();
    xil_printf("FIR self-test done.\n\r");

    xil_printf("SW0=LP base | SW1=HP base | SW2=LP agg | SW3=HP agg | none=passthrough\n\r");

    CodecInit(SCU_TIMER_ID, AUDIO_IIC_ID, AUDIO_CTRL_BASEADDR);
    FifoInit(AUDIO_FIFO);

    /* Cold start: zero both sliding-window buffers */
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
