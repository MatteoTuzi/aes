/*******************************************************************************
 *  UART Microserver – Histogram Equalization (arbitrary-size PPM)
 *  University of Cagliari – Advanced Embedded Systems (AES)
 *
 *  Receives a P6 PPM over UART, applies histogram equalization to all RGB
 *  bytes, sends the processed image back. Uses a LUT to map each intensity
 *  [0..255] to its equalized value (CDF-based).
 *
 *  PPM: 3-line header ("P6", "W H", "255"), then raw RGB. Arbitrary size.
 *  Hardware: Zybo Z7, PS UART1 @ 115200 8N1.
 ******************************************************************************/

#include "platform.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "xparameters.h"

/* UART base address (PS UART1) */
#define UART_BASE       XPAR_PS7_UART_1_BASEADDR
/* Max characters per header line (e.g. "128 128\n") */
#define LINE_MAX        32
/* Max image dimension; images larger than 512x512 are rejected */
#define IMG_MAX_DIM     512
/* Max number of RGB bytes (width * height * 3) */
#define IMG_MAX_PIXELS  (IMG_MAX_DIM * IMG_MAX_DIM * 3)
/* Number of intensity levels (0..255) */
#define LEVELS          256

/* Static buffer for received/processed image; avoids malloc on small heap */
static u8 image_buf[IMG_MAX_PIXELS];

/*******************************************************************************
 * read_line
 * Reads bytes from UART until '\n' or maxlen-1 chars. Result is
 * null-terminated. Blocking.
 ******************************************************************************/
static int read_line(u32 base, char *buf, int maxlen)
{
	int i = 0;
	char c;
	while (i < maxlen - 1) {
		c = (char)XUartPs_RecvByte(base);
		buf[i++] = c;
		if (c == '\n')
			break;
	}
	buf[i] = '\0';
	return i;
}

/*******************************************************************************
 * to_int
 * Converts ASCII decimal string to int. Stops at first non-digit.
 ******************************************************************************/
static int to_int(const char *s)
{
	int n = 0;
	int i = 0;
	while (s[i] >= '0' && s[i] <= '9') {
		n = n * 10 + (s[i] - '0');
		i++;
	}
	return n;
}

/*******************************************************************************
 * parse_width_height
 * Extracts width and height from line2 (e.g. "128 128\n"). Skips
 * any whitespace (space, tab, \r) between the two numbers.
 ******************************************************************************/
static void parse_width_height(const char *line2, int *width, int *height)
{
	int i = 0;
	*width = 0;
	*height = 0;
	while (line2[i] >= '0' && line2[i] <= '9')
		*width = *width * 10 + (line2[i++] - '0');
	while (line2[i] == ' ' || line2[i] == '\t' || line2[i] == '\r')
		i++;
	while (line2[i] >= '0' && line2[i] <= '9')
		*height = *height * 10 + (line2[i++] - '0');
}

/*******************************************************************************
 * apply_histogram_equalization
 * Equalizes the image using a CDF-based LUT:
 *   1. Build histogram hist[0..255] over all bytes.
 *   2. Build CDF: cdf[i] = sum of hist[0..i].
 *   3. Find first non-zero CDF value (n_min).
 *   4. LUT: map[i] = round( (cdf[i]-n_min) / (n-n_min) * 255 ), clamped.
 *   5. Replace each pixel: img[i] = map[img[i]].
 * If all pixels have the same value, no change (early return).
 ******************************************************************************/
static void apply_histogram_equalization(u8 *img, int n)
{
	int hist[LEVELS];
	int cdf[LEVELS];
	u8 map[LEVELS];
	int n_min;
	int i;

	/* Build histogram */
	for (i = 0; i < LEVELS; i++)
		hist[i] = 0;
	for (i = 0; i < n; i++)
		hist[img[i]]++;

	/* Cumulative distribution function */
	cdf[0] = hist[0];
	for (i = 1; i < LEVELS; i++)
		cdf[i] = cdf[i - 1] + hist[i];

	/* First non-zero CDF value (avoids division by zero in LUT) */
	n_min = 0;
	for (i = 0; i < LEVELS; i++) {
		if (cdf[i] != 0) {
			n_min = cdf[i];
			break;
		}
	}
	if (n_min >= n)
		return;

	/* Build LUT: equalized value for each intensity 0..255 */
	for (i = 0; i < LEVELS; i++) {
		float norm = (float)(cdf[i] - n_min) / (float)(n - n_min);
		int v = (int)(norm * 255.0f);
		if (v < 0) v = 0;
		if (v > 255) v = 255;
		map[i] = (u8)v;
	}

	/* Apply LUT to every byte */
	for (i = 0; i < n; i++)
		img[i] = map[img[i]];
}

/*******************************************************************************
 * run_equalized_server
 * Receives PPM (3-line header + raw RGB), equalizes, sends header + processed
 * pixels back. Returns 0 on success, -1 on invalid header or size.
 ******************************************************************************/
static int run_equalized_server(u32 base)
{
	char line1[LINE_MAX], line2[LINE_MAX], line3[LINE_MAX];
	int width, height, maxval, num_pixels;
	int i;

	read_line(base, line1, LINE_MAX);
	read_line(base, line2, LINE_MAX);
	read_line(base, line3, LINE_MAX);

	parse_width_height(line2, &width, &height);
	maxval = to_int(line3);
	if (maxval != 255 || width <= 0 || height <= 0 ||
	    width > IMG_MAX_DIM || height > IMG_MAX_DIM)
		return -1;

	num_pixels = width * height * 3;
	if (num_pixels > IMG_MAX_PIXELS)
		return -1;

	for (i = 0; i < num_pixels; i++)
		image_buf[i] = XUartPs_RecvByte(base);

	apply_histogram_equalization(image_buf, num_pixels);

	for (i = 0; line1[i] != '\0'; i++)
		XUartPs_SendByte(base, (u8)line1[i]);
	for (i = 0; line2[i] != '\0'; i++)
		XUartPs_SendByte(base, (u8)line2[i]);
	for (i = 0; line3[i] != '\0'; i++)
		XUartPs_SendByte(base, (u8)line3[i]);

	for (i = 0; i < num_pixels; i++)
		XUartPs_SendByte(base, image_buf[i]);

	return 0;
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
 * Platform init, UART setup, run equalization server once, then cleanup.
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

	if (run_equalized_server(uart_base) != 0) {
		cleanup_platform();
		return -1;
	}

	cleanup_platform();
	return 0;
}
