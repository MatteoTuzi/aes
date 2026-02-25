/***************************************************************
 *  Lab 5 – Group 0 DNN on MNIST (784→64→10)
 *  ------------------------------------------------------------
 *  Custom topology: FC0 → ReLU → FC1
 *  TIP1: Q0.8 images (784 bytes UART) → Q8.8
 *  TIP2: Q1.7 weights (optional, use weights0_q17/weights1_q17)
 *
 *  Set MODE: 0=baseline, 1=TIP1, 2=TIP1+TIP2
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "platform.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "weights_group0.h"
#include "test_images.h"
#include "sleep.h"
#include <xtime_l.h>
#include <time.h>

/* ---------------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------------- */
#define MODE 0
#define IMG_SIZE   (28 * 28)
#define QF         8
#define N_BIAS0    64
#define N_WEIGHTS0 (64 * 784)
#define N_BIAS1    10
#define N_WEIGHTS1 (10 * 64)
#define SIZEWA     10

typedef short int DATA;

#define FIXED2FLOAT(a, qf) (((float)(a)) / (1 << (qf)))

/* ---------------------------------------------------------------------------
 * Global / static variables
 * --------------------------------------------------------------------------- */
#if MODE == 2
static const signed char gemm0_bias_q17[N_BIAS0] = {bias0_q17};
static const signed char gemm0_weights_q17[N_WEIGHTS0] = {weights0_q17};
static const signed char gemm1_bias_q17[N_BIAS1] = {bias1_q17};
static const signed char gemm1_weights_q17[N_WEIGHTS1] = {weights1_q17};
#else
static DATA gemm0_bias[N_BIAS0] = {bias0};
static DATA gemm0_weights[N_WEIGHTS0] = {weights0};
static DATA gemm1_bias[N_BIAS1] = {bias1};
static DATA gemm1_weights[N_WEIGHTS1] = {weights1};
#endif

static DATA immagine[10][IMG_SIZE];
static XUartPs Uart_1_PS;

/* ---------------------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------------------- */
static u8 uart_inbyte(void);
static DATA readfromUART(void);
static int uart_init(void);
static void uart_receive_image(DATA *image);

void FC_forward(DATA *input, DATA *output, int in_s, int out_s,
                DATA *weights, DATA *bias, int qf);
#if MODE == 2
void FC_forward_q17(DATA *input, DATA *output, int in_s, int out_s,
                    const signed char *weights, const signed char *bias, int qf);
#endif
static inline void relu_forward(DATA *input, DATA *output, int size);
int resultsProcessing(DATA *results, int size);

static int dnn_inference(DATA *image);
static void run_offline_test(void);
static void run_uart_loop(void);

/* ---------------------------------------------------------------------------
 * Main
 * --------------------------------------------------------------------------- */
/* Init platform and UART, print banner, run offline test then UART inference loop. */
int main(void) {
    init_platform();

    if (uart_init() != XST_SUCCESS) {
        xil_printf("UART init failed\r\n");
        return XST_FAILURE;
    }

    xil_printf("\r\nMNIST Group 0 (784->64->10)");
#if MODE == 1
    xil_printf(" [TIP1]");
#elif MODE == 2
    xil_printf(" [TIP1+TIP2]");
#endif
    xil_printf("\r\n");

    run_offline_test();
    run_uart_loop();

    cleanup_platform();
    return 0;
}

/* ---------------------------------------------------------------------------
 * UART
 * --------------------------------------------------------------------------- */
/* Block until one byte is received. */
static inline u8 uart_inbyte(void) {
    u8 b;
    while (XUartPs_Recv(&Uart_1_PS, &b, 1) != 1);
    return b;
}

/* MODE 0: read 2 bytes (LSB-first) as Q8.8. MODE 1/2: read 1 byte (Q0.8), extend to Q8.8 by left shift. */
#if MODE == 0
static DATA readfromUART(void) {
    u8 lsb = uart_inbyte();
    u8 msb = uart_inbyte();
    return (DATA)((msb << 8) | lsb);
}
#else
static DATA readfromUART(void) {
    u8 px = uart_inbyte();
    return (DATA)((short)px << 8);
}
#endif

/* Lookup and init UART PS, set baud rate to 115200. */
static int uart_init(void) {
    XUartPs_Config *cfg = XUartPs_LookupConfig(XPAR_PS7_UART_1_DEVICE_ID);
    if (!cfg) return XST_FAILURE;
    if (XUartPs_CfgInitialize(&Uart_1_PS, cfg, cfg->BaseAddress) != XST_SUCCESS)
        return XST_FAILURE;
    if (XUartPs_SetBaudRate(&Uart_1_PS, 115200) != (s32)XST_SUCCESS)
        return XST_FAILURE;
    return XST_SUCCESS;
}

/* Read IMG_SIZE pixels from UART into image[] (format depends on MODE). */
static void uart_receive_image(DATA *image) {
    for (int i = 0; i < IMG_SIZE; i++)
        image[i] = readfromUART();
}

/* ---------------------------------------------------------------------------
 * Fully Connected layers
 * --------------------------------------------------------------------------- */
/* output = weights * input + bias in Q8.8; mac in 64-bit, then >> qf. */
void FC_forward(DATA *input, DATA *output, int in_s, int out_s,
                DATA *weights, DATA *bias, int qf) {
    for (int h = 0; h < out_s; h++) {
        long long mac = ((long long)bias[h]) << qf;
        for (int w = 0; w < in_s; w++)
            mac += (long long)input[w] * weights[h * in_s + w];
        output[h] = (DATA)(mac >> qf);
    }
}

#if MODE == 2
/* FC with Q1.7 weights/bias: scale to Q8.8 via <<1, then MAC in 64-bit; mac = multiply-accumulate
   accumulator (bias + sum of input*weight), kept shifted by qf to avoid overflow; result >> qf. */
void FC_forward_q17(DATA *input, DATA *output, int in_s, int out_s,
                    const signed char *weights, const signed char *bias, int qf) {
    for (int h = 0; h < out_s; h++) {
        long long mac = ((long long)((short)bias[h] << 1)) << qf;
        for (int w = 0; w < in_s; w++) {
            DATA w88 = (DATA)((short)weights[h * in_s + w] << 1);
            mac += (long long)input[w] * w88;
        }
        output[h] = (DATA)(mac >> qf);
    }
}
#endif

/* ReLU: output[i] = max(0, input[i]). */
static inline void relu_forward(DATA *input, DATA *output, int size) {
    for (int i = 0; i < size; i++)
        output[i] = input[i] > 0 ? input[i] : 0;
}

/* Convert logits from fixed-point to float, return index of maximum (predicted class 0..size_wa-1). */
int resultsProcessing(DATA *results, int size) {
    int size_wa = (size > SIZEWA) ? SIZEWA : size;
    float results_float[SIZEWA];

    for (int i = 0; i < size_wa; i++)
        results_float[i] = FIXED2FLOAT(results[i], 8);

    int top = 0;
    float topval = results_float[0];
    for (int i = 1; i < size_wa; i++) {
        if (results_float[i] > topval) {
            top = i;
            topval = results_float[i];
        }
    }
    return top;
}

/* ---------------------------------------------------------------------------
 * DNN inference
 * --------------------------------------------------------------------------- */
/* FC0 -> ReLU -> FC1 -> argmax; uses FC_forward or FC_forward_q17 depending on MODE. */
static int dnn_inference(DATA *image) {
    static DATA out0[64], in1[64], out1[10];

#if MODE == 2
    FC_forward_q17(image, out0, IMG_SIZE, 64,
                   gemm0_weights_q17, gemm0_bias_q17, QF);
#else
    FC_forward(image, out0, IMG_SIZE, 64, gemm0_weights, gemm0_bias, QF);
#endif
    relu_forward(out0, in1, 64);

#if MODE == 2
    FC_forward_q17(in1, out1, 64, 10,
                   gemm1_weights_q17, gemm1_bias_q17, QF);
#else
    FC_forward(in1, out1, 64, 10, gemm1_weights, gemm1_bias, QF);
#endif
    return resultsProcessing(out1, 10);
}

/* ---------------------------------------------------------------------------
 * Offline test
 * --------------------------------------------------------------------------- */
/* Load 10 embedded images, run inference for each, print expected vs predicted. */
static void run_offline_test(void) {
    /* test_images.h: values 0-256 (Q8.8). */
    { int t[] = {imm_test_0}; for (int i = 0; i < IMG_SIZE; i++) immagine[0][i] = (DATA)t[i]; }
    { int t[] = {imm_test_1}; for (int i = 0; i < IMG_SIZE; i++) immagine[1][i] = (DATA)t[i]; }
    { int t[] = {imm_test_2}; for (int i = 0; i < IMG_SIZE; i++) immagine[2][i] = (DATA)t[i]; }
    { int t[] = {imm_test_3}; for (int i = 0; i < IMG_SIZE; i++) immagine[3][i] = (DATA)t[i]; }
    { int t[] = {imm_test_4}; for (int i = 0; i < IMG_SIZE; i++) immagine[4][i] = (DATA)t[i]; }
    { int t[] = {imm_test_5}; for (int i = 0; i < IMG_SIZE; i++) immagine[5][i] = (DATA)t[i]; }
    { int t[] = {imm_test_6}; for (int i = 0; i < IMG_SIZE; i++) immagine[6][i] = (DATA)t[i]; }
    { int t[] = {imm_test_7}; for (int i = 0; i < IMG_SIZE; i++) immagine[7][i] = (DATA)t[i]; }
    { int t[] = {imm_test_8}; for (int i = 0; i < IMG_SIZE; i++) immagine[8][i] = (DATA)t[i]; }
    { int t[] = {imm_test_9}; for (int i = 0; i < IMG_SIZE; i++) immagine[9][i] = (DATA)t[i]; }

    xil_printf("\r\n--- Group 0 offline test ---\r\n");
    for (int i = 0; i < 10; i++) {
        int pred = dnn_inference(immagine[i]);
        xil_printf("  [%d] expected=%d predicted=%d\r\n", i, i, pred);
        usleep(1000000);
    }
    xil_printf("--- Done ---");
}

/* ---------------------------------------------------------------------------
 * UART loop
 * --------------------------------------------------------------------------- */
/* Wait for image over UART, run inference, print prediction and timing (RX, DNN, PRINT, total us). */
static void run_uart_loop(void) {
    static DATA image_uart[IMG_SIZE];
    XTime t0, t1, t2, t3;
    double us_per_tick = 1e6 / (double)COUNTS_PER_SECOND;

#if MODE == 0
    const char *msg = "Ready. Send image (1568 bytes, Q8.8 LSB-first)...\r\n";
#else
    const char *msg = "Ready. Send image (784 bytes, Q0.8 TIP1)...\r\n";
#endif

    while (1) {
        xil_printf("\r\n%s", msg);

        XTime_GetTime(&t0);
        uart_receive_image(image_uart);
        XTime_GetTime(&t1);

        int pred = dnn_inference(image_uart);
        XTime_GetTime(&t2);
        xil_printf("Predicted: %d\r\n", pred);
        usleep(1000000);
        XTime_GetTime(&t3);

        u32 rx_us   = (u32)((t1 - t0) * us_per_tick);
        u32 dnn_us  = (u32)((t2 - t1) * us_per_tick);
        u32 prn_us  = (u32)((t3 - t2) * us_per_tick);
        u32 iter_us = (u32)((t3 - t0) * us_per_tick);
        xil_printf("Timing [us]: RX=%lu DNN=%lu PRINT=%lu total=%lu\r\n\r\n",
                   rx_us, dnn_us, prn_us, iter_us);
    }
}
