/******************************************************************************
 * loopback.h
 * Lab 3 – Step 1: Audio Passthrough & Fs Measurement
 * University of Cagliari – Advanced Embedded Systems (AES)
 ******************************************************************************
 * Defines, globals, and function declarations for real-time audio loopback
 * and sampling-frequency measurement on Zybo Z7 (SSM2603, AXI-I2S, AXI FIFO,
 * ARM Global Timer).
 ******************************************************************************/

#ifndef LOOPBACK_H
#define LOOPBACK_H

#include "xparameters.h"
#include "xiicps.h"

/* ---------------------------------------------------------------------------
 * I2S register offsets
 * --------------------------------------------------------------------------- */
#define I2S_RESET_REG         0x00
#define I2S_CTRL_REG          0x04
#define I2S_CLK_CTRL_REG      0x08
#define I2S_FIFO_STS_REG      0x20
#define I2S_RX_FIFO_REG       0x28
#define I2S_TX_FIFO_REG       0x2C

/* ---------------------------------------------------------------------------
 * AXI FIFO register offsets
 * --------------------------------------------------------------------------- */
#define FIFO_ISR              0x00
#define FIFO_IER              0x04
#define FIFO_TDFV             0x0C
#define FIFO_TDFD             0x10
#define FIFO_TLR              0x14
#define FIFO_RDFO             0x1C
#define FIFO_RDFD             0x20
#define FIFO_RLR              0x24
#define FIFO_TDR              0x2C
#define FIFO_RDR              0x30

/* ---------------------------------------------------------------------------
 * I²C / SSM2603
 * --------------------------------------------------------------------------- */
#define IIC_SLAVE_ADDR        0b0011010
#define IIC_SCLK_RATE         100000

/* ---------------------------------------------------------------------------
 * Hardware base addresses (from xparameters)
 * --------------------------------------------------------------------------- */
#define AUDIO_IIC_ID          XPAR_XIICPS_0_DEVICE_ID
#define AUDIO_CTRL_BASEADDR   XPAR_AXI_I2S_ADI_0_S00_AXI_BASEADDR
#define SCU_TIMER_ID          XPAR_SCUTIMER_DEVICE_ID
#define SWI_BASE_ADDR         XPAR_AXI_GPIO_2_BASEADDR
#define LED_BASE_ADDR         XPAR_AXI_GPIO_1_BASEADDR
#define BUT_BASE_ADDR         XPAR_AXI_GPIO_0_BASEADDR
#define AUDIO_FIFO            XPAR_AXI_FIFO_MM_S_0_BASEADDR
#define FIR_FIFO              XPAR_AXI_FIFO_MM_S_1_BASEADDR

/* ---------------------------------------------------------------------------
 * Global Timer (Fs measurement). 64-bit counter @ CPU/2 ≈ 333 MHz; LSB only.
 * --------------------------------------------------------------------------- */
#define GTIMER_BASE           XPAR_PS7_GLOBALTIMER_0_S_AXI_BASEADDR
#define GTIMER_CTRL           0x08
#define GTIMER_CNT_LO         0x00
#define CPU_FREQ_HZ           667000000
#define GLOBAL_TMR_FREQ       (CPU_FREQ_HZ / 2)
#define N_SAMPLES_FS          100   /* number of periods to average for Fs */
#define FS_WARMUP             300   /* loop iterations before starting Fs measurement */

/* ---------------------------------------------------------------------------
 * Global variables
 * --------------------------------------------------------------------------- */
extern XIicPs Iic;

/* ---------------------------------------------------------------------------
 * Function declarations
 * --------------------------------------------------------------------------- */
int CodecRegWrite(XIicPs *IIcPtr, u8 regAddr, u16 regData);
int CodecInit(u16 timerID, u16 iicID, u32 i2sAddr);
void FifoTxWrite(u32 i2sBaseAddr, u32 audioData);
u32 FifoRxRead(u32 i2sBaseAddr);
void FifoInit(u32 fifoAddr);

#endif /* LOOPBACK_H */
