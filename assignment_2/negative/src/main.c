/*******************************************************************************
 *  UART Microserver – Negative (fixed 128×128 PPM)
 *  University of Cagliari – Advanced Embedded Systems (AES)
 *
 *  UART microserver on Zybo Z7. Receives a fixed-size P6 PPM (128×128),
 *  applies pixel negative, sends the processed image back over UART.
 *
 *  PPM: P6, 128×128, maxval 255. Header "P6\n128 128\n255\n" (15 bytes).
 *  Protocol: client sends header + raw RGB; server echoes header + negative.
 *
 *  Hardware: Zybo Z7 (ARM Cortex-A9). UART: PS UART1 @ 115200, 8N1.
 *  Tools: Xilinx SDK / Vitis, RealTerm or gtkterm.
 ******************************************************************************/

#include "platform.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "xparameters.h"

/* UART base address (PS UART1) */
#define UART_BASE    XPAR_PS7_UART_1_BASEADDR
/* Header size: "P6\n128 128\n255\n" */
#define HEADER_SIZE  15
/* Fixed image dimensions (128×128) */
#define IMG_W        128
#define IMG_H        128
/* Total RGB bytes (width × height × 3) */
#define PIXELS       (IMG_W * IMG_H * 3)

/* Buffers for header and image; fixed size, no malloc */
static u8 header[HEADER_SIZE];
static u8 image[PIXELS];

/*******************************************************************************
 * apply_negative
 * Inverts each byte: p_out = 255 - p_in. Applied to all RGB components.
 ******************************************************************************/
static void apply_negative(u8 *img, int n)
{
	int i;
	for (i = 0; i < n; i++)
		img[i] = (u8)(255 - img[i]);
}

/*******************************************************************************
 * run_negative_server
 * Receives header (15 bytes) + raw RGB, applies negative in-place, then
 * sends header + processed pixels back over UART.
 ******************************************************************************/
static void run_negative_server(u32 base)
{
	int i;

	for (i = 0; i < HEADER_SIZE; i++)
		header[i] = XUartPs_RecvByte(base);
	for (i = 0; i < PIXELS; i++)
		image[i] = XUartPs_RecvByte(base);

	apply_negative(image, PIXELS);

	for (i = 0; i < HEADER_SIZE; i++)
		XUartPs_SendByte(base, header[i]);
	for (i = 0; i < PIXELS; i++)
		XUartPs_SendByte(base, image[i]);
}

/*******************************************************************************
 * setup_uart
 * Initialises PS UART1 @ 115200 8N1. Fills *base with UART base address.
 * Returns 0 on success, -1 on failure.
 ******************************************************************************/
static int setup_uart(XUartPs *uart, u32 *base)
{
	XUartPs_Config *cfg;

	cfg = XUartPs_LookupConfig(XPAR_PS7_UART_1_DEVICE_ID);
	if (!cfg)
		return -1;
	if (XUartPs_CfgInitialize(uart, cfg, cfg->BaseAddress) != XST_SUCCESS)
		return -1;
	if (XUartPs_SetBaudRate(uart, (u32)115200) != (s32)XST_SUCCESS)
		return -1;
	*base = cfg->BaseAddress;
	return 0;
}

/*******************************************************************************
 * main
 * Platform init, UART setup, run negative server once, then cleanup.
 ******************************************************************************/
int main(void)
{
	XUartPs uart;
	u32 uart_base;

	init_platform();

	if (setup_uart(&uart, &uart_base) != 0) {
		cleanup_platform();
		return -1;
	}

	run_negative_server(uart_base);

	cleanup_platform();
	return 0;
}
