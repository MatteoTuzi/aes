/***************************************************************
 *  Lab 2 – UART Microserver (Negative Version)
 *  Author: Lello Molinario
 *  University of Cagliari – Advanced Embedded Systems (AES)
 *  Student ID: 70/90/000369
 *  Date: November 2025
 *
 *  Description
 *  ------------------------------------------------------------
 *  UART-based microserver running on the Zybo Z7.
 *  This version expects a *fixed-size* PPM P6 image of
 *  resolution 128×128 pixels. The server receives the image
 *  over UART, computes the negative of each RGB pixel, and
 *  returns the processed PPM to the client.
 *
 *  Supported PPM format:
 *   • P6 (binary)
 *   • Fixed size: 128 × 128
 *   • 1-byte RGB channels (maxval = 255)
 *
 *  Communication protocol:
 *   1. Client sends 3 header lines:
 *        - "P6"
 *        - "128 128"
 *        - "255"
 *   2. Client sends raw RGB data (width × height × 3 bytes)
 *   3. Microserver applies negative transformation
 *   4. Microserver responds with:
 *        - Same header (3 lines)
 *        - Processed RGB data
 *
 *  Platform
 *  ------------------------------------------------------------
 *   • Hardware: Zybo Z7 board (ARM Cortex-A9 – PS7)
 *   • UART: PS UART1 @ 115200 baud, 8N1
 *   • Tools: Xilinx SDK / Vitis + RealTerm (Windows) or gtkterm (Linux)
 *
 ***************************************************************/

// Standard C and Xilinx libraries
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "stdlib.h"

// Base address of PS UART1 used for all RX/TX operations
#define UART_BASE XPAR_PS7_UART_1_BASEADDR

// Fixed-size PPM header length for the 128×128 P6 image:
// Expected format: "P6\n128 128\n255\n" → 15 bytes
#define HEADER_SIZE 15 // Example: "P6\n128 128\n255\n"

// Fixed image resolution (128×128 RGB)
#define IMG_W 128
#define IMG_H 128

// Total number of bytes in the RGB image buffer:
//   128 × 128 pixels × 3 (R,G,B) = 49 152 bytes
#define PIXELS (IMG_W * IMG_H * 3)

// Static buffers allocated in .bss:
// • header[]  – stores the received PPM header (15 bytes)
// • image[]   – stores the raw RGB data (49 152 bytes)
u8 header[HEADER_SIZE];
u8 image[PIXELS];

/***************************************************************
 *  apply_negative()
 *  ------------------------------------------------------------
 *  Applies the negative transform to each pixel:
 *      pixel_out = 255 - pixel_in
 *
 *  PARAMETERS:
 *    - img : pointer to pixel buffer
 *    - n   : number of bytes (width * height * 3)
 ***************************************************************/
void apply_negative(u8 *img, int n) {
    for (int i = 0; i < n; i++)
        img[i] = 255 - img[i];
}

/***************************************************************
 *  main()
 *  ------------------------------------------------------------
 *  Implements the UART PPM microserver:
 *  - init UART
 *  - read header
 *  - read pixel data
 *  - apply negative transform
 *  - send header back
 *  - send processed pixel buffer
 ***************************************************************/

int main()
{
    init_platform();
     /***********************************************************
     *  UART initialization (PS UART1 @ 115200 baud)
     **********************************************************/
    XUartPs Uart_1_PS;
    u16 DeviceId_1= XPAR_PS7_UART_1_DEVICE_ID;
    int Status_1;
    XUartPs_Config *Config_1;
    Config_1 = XUartPs_LookupConfig(DeviceId_1);
    if (NULL == Config_1) {
        return XST_FAILURE;
    }
    Status_1 = XUartPs_CfgInitialize(&Uart_1_PS, Config_1, Config_1->BaseAddress); //init  UART
    if (Status_1 != XST_SUCCESS) {
        return XST_FAILURE;
    }
    u32 BaudRate = (u32)115200;
    Status_1 = XUartPs_SetBaudRate(&Uart_1_PS, BaudRate); //set the BaudRate = 115200
    if (Status_1 != (s32)XST_SUCCESS) {
        return XST_FAILURE;
    }

    /***********************************************************
     *  Receive PPM header (fixed 15 bytes)
     **********************************************************/

    for (int i = 0; i < HEADER_SIZE; i++)
        header[i] = XUartPs_RecvByte(UART_BASE);

    /***********************************************************
     * Receive raw RGB pixel data (128×128×3 bytes)
     **********************************************************/
    for (int i = 0; i < PIXELS; i++)
        image[i] = XUartPs_RecvByte(UART_BASE);

    /***********************************************************
     * Apply negative transformation
     **********************************************************/
    apply_negative(image, PIXELS);

    /***********************************************************
     * Send back header
     **********************************************************/
    for (int i = 0; i < HEADER_SIZE; i++)
        XUartPs_SendByte(UART_BASE, header[i]);

    /***********************************************************
     * Send back processed pixel buffer
     **********************************************************/
    for (int i = 0; i < PIXELS; i++)
        XUartPs_SendByte(UART_BASE, image[i]);

    cleanup_platform();    // Shut down Zybo platform and de-initialize drivers

    return 0;
}
