/******************************************************************************
 * Lab 3 – Step 1: Audio Passthrough & Fs Measurement
 * University of Cagliari – Advanced Embedded Systems (AES)
 ******************************************************************************
 *
 * OVERVIEW
 * --------
 * Real-time audio loopback on the Zybo Z7. Acquires stereo samples (L/R)
 * from the SSM2603 via AXI-I2S and forwards them to the I²S TX FIFO
 * (passthrough). Measures the actual sampling frequency Fs using the
 * ARM Global Timer.
 *
 * STEP 1 OBJECTIVES
 * -----------------
 *   • Verify SSM2603 init via I²C and blocking I²S acquisition
 *   • Test IN → OUT loopback (ADC → I²S RX → CPU → I²S TX → DAC)
 *   • Measure Fs: timer ticks between consecutive stereo frames, averaged
 *     over N_SAMPLES_FS periods; Fs = GLOBAL_TMR_FREQ / avg_delta. UART
 *     output only after measurement to avoid perturbing timing.
 *
 * PLATFORM
 * --------
 *   Board: Zybo Z7 (Zynq-7000). Codec: SSM2603 (I²C, I²S).
 *   AXI-I2S, AXI FIFO, PS I²C, PS Global Timer. UART @ 115200 8N1.
 *
 ******************************************************************************/

#include <stdio.h>
#include <math.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xiicps.h"
#include "timer_ps.h"
#include "loopback.h"

XIicPs Iic;   /* I²C instance for SSM2603 */

/*******************************************************************************
 * CodecRegWrite
 * Writes a single 9-bit register value to the SSM2603 codec over I²C.
 * Blocks until bus idle after transfer.
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
 * transmit-complete, then clears the flag.
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
 * then reads and returns it. Used for synchronous sample-by-sample acquisition.
 ******************************************************************************/
u32 FifoRxRead(u32 i2sBaseAddr)
{
    while (Xil_In32(i2sBaseAddr + 0x1C) == 0)
        ;
    return (u32)Xil_In32(i2sBaseAddr + 0x20);
}

/*******************************************************************************
 * FifoInit
 * Resets and configures the AXI FIFO for I²S streaming.
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
 * main
 * Platform init, audio init (codec + FIFO), enable Global Timer. Main loop:
 * read L/R from I²S RX, run Fs measurement (after FS_WARMUP iters, over
 * N_SAMPLES_FS periods: t0 → read L/R → t1, accumulate Δ; then print Fs
 * once), forward L/R to I²S TX (loopback). UART only after timing.
 ******************************************************************************/
int main(void)
{
    u32 L, R;
    u32 L_tmp, R_tmp;
    u32 t0, t1;
    u64 accum_ticks;
    int sample_cnt, j;
    float avg_delta, Fs;
    int Fs100;

    init_platform();

    xil_printf("Step 1 — Fs measurement + loopback\n\r");

    CodecInit(SCU_TIMER_ID, AUDIO_IIC_ID, AUDIO_CTRL_BASEADDR);
    FifoInit(AUDIO_FIFO);

    Xil_Out32(GTIMER_BASE + GTIMER_CTRL, 0x03);

    accum_ticks = 0;
    sample_cnt = 0;
    j = 0;

    while (1) {
        L = FifoRxRead(AUDIO_FIFO);
        R = FifoRxRead(AUDIO_FIFO);

        /* Fs measurement: after FS_WARMUP iters, average over N_SAMPLES_FS periods */
        if (j >= FS_WARMUP && sample_cnt < N_SAMPLES_FS) {
            t0 = Xil_In32(GTIMER_BASE + GTIMER_CNT_LO);
            L_tmp = FifoRxRead(AUDIO_FIFO);
            R_tmp = FifoRxRead(AUDIO_FIFO);
            t1 = Xil_In32(GTIMER_BASE + GTIMER_CNT_LO);
            accum_ticks += (u64)(t1 - t0);
            sample_cnt++;
            FifoTxWrite(AUDIO_FIFO, L_tmp);
            FifoTxWrite(AUDIO_FIFO, R_tmp);
        }

        if (sample_cnt == N_SAMPLES_FS) {
            avg_delta = (float)accum_ticks / (float)N_SAMPLES_FS;
            Fs = (float)GLOBAL_TMR_FREQ / avg_delta;
            Fs100 = (int)(Fs * 100);
            xil_printf("Fs (avg over %d samples) ~= %d.%02d Hz\n\r",
                       N_SAMPLES_FS, Fs100 / 100, Fs100 % 100);
            sample_cnt++;
        }

        /* Loopback: forward L/R to I²S TX */
        FifoTxWrite(AUDIO_FIFO, L);
        FifoTxWrite(AUDIO_FIFO, R);
        j++;
    }

    cleanup_platform();
    return 0;
}
