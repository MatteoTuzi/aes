/***************************************************************
 *  Lab 2 – UART Microserver (Histogram Stretching)
 *  Author: Lello Molinario
 *  University of Cagliari – Advanced Embedded Systems (AES)
 *  Student ID: 70/90/000369
 *  Date: November 2025
 *
 *  Description
 *  ------------------------------------------------------------
 *  UART-based microserver running on the Zybo Z7.
 *  The server receives a PPM (P6) image of arbitrary size over
 *  UART, performs linear histogram stretching on all RGB bytes,
 *  and sends the processed PPM image back to the client.
 *
 *  Histogram Stretching:
 *      I_min = minimum intensity in the image
 *      I_max = maximum intensity in the image
 *
 *      scale  = 255.0 / (I_max - I_min)
 *      p_out = (p_in - I_min) * scale
 *
 *  Supported PPM format:
 *   • P6 (binary)
 *   • Arbitrary image size (width × height)
 *   • 1-byte RGB channels (maxval = 255)
 *
 *  Communication protocol:
 *   1. Client sends 3 header lines:
 *        - "P6"
 *        - "<width> <height>"
 *        - "255"
 *   2. Client sends raw RGB data (width × height × 3 bytes)
 *   3. Microserver applies Histogram Stretching
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

/***************************************************************
 *  read_line()
 *  ------------------------------------------------------------
 *  Reads characters from UART until '\n' is encountered,
 *  or until maxlen - 1 characters have been stored.
 *
 *  The resulting string is NULL-terminated.
 ***************************************************************/
int read_line(char *buf, int maxlen) {
    int i = 0;
    char c;
    while (i < maxlen-1) {
        c = XUartPs_RecvByte(UART_BASE); // blocking receive
        buf[i++] = c;
        if (c == '\n') // end of textual line
        break;
    }
    buf[i] = 0;// end string
    return i;
}

/***************************************************************
 *  to_int()
 *  ------------------------------------------------------------
 *  Converts an ASCII numeric string to integer.
 *  Stops parsing at the first non-digit character.
 ***************************************************************/
int to_int(char *s) {
    int n = 0;
    int i = 0;
    while (s[i] >= '0' && s[i] <= '9') {
        n = n*10 + (s[i] - '0');
        i++;
    }
    return n;
}

/***************************************************************
 *  apply_histogram_stretching()
 *  ------------------------------------------------------------
 *  Finds I_min and I_max across the entire image buffer and
 *  applies linear stretching:
 *
 *      scale = 255.0 / (I_max - I_min)
 *      p_new = (p - I_min) * scale
 *
 ***************************************************************/
void apply_histogram_stretching(u8 *img, int n) {

    // 1) Find minimum and maximum pixel intensity
    u8 I_min = 255;
    u8 I_max = 0;

    for (int i = 0; i < n; i++) {
        if (img[i] < I_min) I_min = img[i];
        if (img[i] > I_max) I_max = img[i];
    }

    // Avoid division by zero
    if (I_max == I_min)
        return;

    // 2) Compute scale factor
    float scale = 255.0f / (float)(I_max - I_min);

    // 3) Apply stretching
    for (int i = 0; i < n; i++) {
        float p = (img[i] - I_min) * scale;

        // Clamp result to [0,255]
        if (p < 0) p = 0;
        if (p > 255) p = 255;

        img[i] = (u8)p;
    }
}

/***************************************************************
 *  main()
 *  ------------------------------------------------------------
 *  Implements the UART PPM microserver:
 *  - init UART
 *  - read header
 *  - read pixel data
 *  - apply histogram stretching
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
    * Read PPM header (3 lines)
    **********************************************************/

    char line1[32], line2[32], line3[32];

    read_line(line1, 32);   // e.g., "P6\n"
    read_line(line2, 32);   // e.g., "128 128\n"
    read_line(line3, 32);   // e.g., "255\n"

   /***********************************************************
    * Parse width and height from ASCII line2
    **********************************************************/
    int width = 0, height = 0;
    int i = 0;

    // parse width
    while (line2[i] >= '0' && line2[i] <= '9') {
        width = width*10 + (line2[i] - '0');
        i++;
    }

    i++;  // skip the space ' '

    // parse height
    while (line2[i] >= '0' && line2[i] <= '9') {
        height = height*10 + (line2[i] - '0');
        i++;
    }

    int maxval = to_int(line3); // expected: 255

    int num_pixels = width * height * 3;

    /***********************************************************
    * Allocate buffer for RGB pixels
    **********************************************************/
    u8 *image = malloc(num_pixels);

    /***********************************************************
    *  Receive raw RGB bytes from UART
    **********************************************************/
    for (int i = 0; i < num_pixels; i++)
        image[i] = XUartPs_RecvByte(UART_BASE);

    /***********************************************************
    * Apply histogram stretching
    **********************************************************/
    apply_histogram_stretching(image, num_pixels);

   /***********************************************************
    * Send original PPM header back
    **********************************************************/
    for (int i = 0; line1[i] != 0; i++)
        XUartPs_SendByte(UART_BASE, line1[i]);

    for (int i = 0; line2[i] != 0; i++)
        XUartPs_SendByte(UART_BASE, line2[i]);

    for (int i = 0; line3[i] != 0; i++)
        XUartPs_SendByte(UART_BASE, line3[i]);

     /***********************************************************
     *  Send processed image bytes
     **********************************************************/
    for (int i = 0; i < num_pixels; i++)
        XUartPs_SendByte(UART_BASE, image[i]);

    free(image);          // Release dynamically allocated image buffer
    cleanup_platform();    // Shut down Zybo platform and de-initialize drivers

    return 0;
}
