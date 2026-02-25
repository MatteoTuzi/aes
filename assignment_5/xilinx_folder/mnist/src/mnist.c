/***************************************************************
 *  Lab 5 – DNN on MNIST (Fixed-Point Inference on ARM Cortex-A9)
 *  ------------------------------------------------------------
 *  File: mnist.c
 *
 *  Course: Advanced Embedded Systems (AES)
 *  University of Cagliari
 *
 *  DNN topology: FC0(784→64) → ReLU → FC1(64→32) → ReLU → FC2(32→10)
 *  Fixed-point Q8.8, UART input, Global Timer profiling.
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "platform.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "weights.h"
#include "test_images.h"
#include "sleep.h"
#include <xtime_l.h>
#include <time.h>

/* ---------------------------------------------------------------------------
 * Network parameters
 * --------------------------------------------------------------------------- */
#define N_BIAS0     64
#define N_WEIGHTS0  (64 * 784)
#define N_BIAS1     32
#define N_WEIGHTS1  (32 * 64)
#define N_BIAS2     10
#define N_WEIGHTS2  (10 * 32)
#define IMG_SIZE    (28 * 28)
#define QF          8

typedef short int DATA;

DATA gemm0_bias[N_BIAS0] = {bias0};
DATA gemm0_weights[N_WEIGHTS0] = {weights0};
DATA gemm1_bias[N_BIAS1] = {bias1};
DATA gemm1_weights[N_WEIGHTS1] = {weights1};
DATA gemm2_bias[N_BIAS2] = {bias2};
DATA gemm2_weights[N_WEIGHTS2] = {weights2};

#define FIXED2FLOAT(a, qf) (((float)(a)) / (1 << (qf)))
#define FLOAT2FIXED(a, qf) ((short int)round((a) * (1 << (qf))))
#define _MAX_ ((1 << (sizeof(DATA) * 8 - 1)) - 1)
#define _MIN_ (-(_MAX_ + 1))

/* ---------------------------------------------------------------------------
 * Function prototypes
 * --------------------------------------------------------------------------- */
void FC_forward(DATA *input, DATA *output, int in_s, int out_s,
                DATA *weights, DATA *bias, int qf);
static inline void relu_forward(DATA *input, DATA *output, int size);
int resultsProcessing(DATA *results, int size);

static int uart_init(void);
static void uart_receive_image(DATA *image);
static int dnn_inference(DATA *image);
static void run_offline_test(void);
static void run_uart_loop(void);

/* ---------------------------------------------------------------------------
 * Reference images (digits 0–9) for offline validation
 * --------------------------------------------------------------------------- */
DATA immagine[10][IMG_SIZE] = {
    {imm_test_0}, {imm_test_1}, {imm_test_2}, {imm_test_3}, {imm_test_4},
    {imm_test_5}, {imm_test_6}, {imm_test_7}, {imm_test_8}, {imm_test_9}
};

/* ---------------------------------------------------------------------------
 * UART
 * --------------------------------------------------------------------------- */
static XUartPs Uart_1_PS;

static inline u8 uart_inbyte(void) {
    u8 b;
    while (XUartPs_Recv(&Uart_1_PS, &b, 1) != 1);
    return b;
}

static DATA readfromUART(void) {
    u8 lsb = uart_inbyte();
    u8 msb = uart_inbyte();
    return (DATA)((msb << 8) | lsb);
}

static int uart_init(void) {
    XUartPs_Config *cfg = XUartPs_LookupConfig(XPAR_PS7_UART_1_DEVICE_ID);
    if (!cfg) return XST_FAILURE;
    if (XUartPs_CfgInitialize(&Uart_1_PS, cfg, cfg->BaseAddress) != XST_SUCCESS)
        return XST_FAILURE;
    if (XUartPs_SetBaudRate(&Uart_1_PS, 115200) != (s32)XST_SUCCESS)
        return XST_FAILURE;
    return XST_SUCCESS;
}

static void uart_receive_image(DATA *image) {
    for (int i = 0; i < IMG_SIZE; i++)
        image[i] = readfromUART();
}

/* ---------------------------------------------------------------------------
 * DNN inference: image → predicted digit (0–9)
 * --------------------------------------------------------------------------- */
static int dnn_inference(DATA *image) {
    static DATA out0[64], in1[64], out1[32], in2[32], out2[10];

    FC_forward(image, out0, IMG_SIZE, 64, gemm0_weights, gemm0_bias, QF);
    relu_forward(out0, in1, 64);

    FC_forward(in1, out1, 64, 32, gemm1_weights, gemm1_bias, QF);
    relu_forward(out1, in2, 32);

    FC_forward(in2, out2, 32, 10, gemm2_weights, gemm2_bias, QF);
    return resultsProcessing(out2, 10);
}

/* ---------------------------------------------------------------------------
 * Offline test: run DNN on embedded images, print expected vs predicted
 * --------------------------------------------------------------------------- */
static void run_offline_test(void) {
    xil_printf("\r\n--- Offline test (embedded images) ---\r\n");
    for (int i = 0; i < 10; i++) {
        int pred = dnn_inference(immagine[i]);
        xil_printf("  [%d] expected=%d predicted=%d\r\n", i, i, pred);
        usleep(1000000);
    }
    xil_printf("--- Offline test done ---\r\n\r\n");
}

/* ---------------------------------------------------------------------------
 * UART inference loop: wait for image, run DNN, print result and timing
 * --------------------------------------------------------------------------- */
static void run_uart_loop(void) {
    static DATA image_uart[IMG_SIZE];
    XTime t0, t1, t2, t3;
    double us_per_tick = 1e6 / (double)COUNTS_PER_SECOND;

    while (1) {
        xil_printf("\r\nReady. Send image (1568 bytes, Q8.8 LSB-first)...\r\n");

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

/* ---------------------------------------------------------------------------
 * Main
 * --------------------------------------------------------------------------- */
int main(void) {
    init_platform();

    if (uart_init() != XST_SUCCESS) {
        xil_printf("UART init failed\r\n");
        return XST_FAILURE;
    }

    xil_printf("\r\nMNIST DNN (Lab 5)\r\n");

    run_offline_test();
    run_uart_loop();

    cleanup_platform();
    return 0;
}

/* ---------------------------------------------------------------------------
 * Fully connected layer: output = W * input + bias (fixed-point Q8.8)
 * --------------------------------------------------------------------------- */
void FC_forward(DATA *input, DATA *output, int in_s, int out_s,
                DATA *weights, DATA *bias, int qf) {
    for (int h = 0; h < out_s; h++) {
        long long mac = ((long long)bias[h]) << qf;
        for (int w = 0; w < in_s; w++)
            mac += (long long)input[w] * weights[h * in_s + w];
        output[h] = (DATA)(mac >> qf);
    }
}

/* ---------------------------------------------------------------------------
 * ReLU activation
 * --------------------------------------------------------------------------- */
static inline void relu_forward(DATA *input, DATA *output, int size) {
    for (int i = 0; i < size; i++)
        output[i] = input[i] > 0 ? input[i] : 0;
}

/* ---------------------------------------------------------------------------
 * Classifier: argmax over logits, returns predicted digit (0–9)
 * --------------------------------------------------------------------------- */
#define SIZEWA 10
int resultsProcessing(DATA *results, int size) {
    int size_wa = (size > SIZEWA) ? SIZEWA : size;
    float results_float[SIZEWA];
    float sum = 0.0f;

    for (int i = 0; i < size_wa; i++)
        results_float[i] = FIXED2FLOAT(results[i], 8);

    for (int i = 0; i < size_wa; i++)
        sum += (float)exp(results_float[i]);

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
