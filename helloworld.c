/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdbool.h>
#include "xaxidma.h"
#include "platform.h"
#include "xparameters.h"
#include <stdlib.h>
#include <unistd.h>
#include <complex.h>
#include <time.h>
#include <xtime_l.h>
#include <math.h>

// Parameters for Data
#define ARR_LEN 8

// Parameters for Algorithm
#define N 8 // Must be power of 2
#define LEVEL 3 // Please set LEVEL to log2(N)
const int bitRevIndex[N] = {0, 4, 2, 6, 1, 5, 3, 7};
const float complex W[N/2] = {1-0*I, 0.7071067811865476-0.7071067811865475*I, 0.0-1*I, -0.7071067811865475-0.7071067811865476*I};

// Global Variables
XAxiDma AxiDma;

// Function headers
void initialize_customWave(float complex arr[ARR_LEN]);
void bitReverse(float complex dataIn[ARR_LEN], float complex dataOut[ARR_LEN]);
void FFT_PS(float complex Data_in[ARR_LEN], float complex Data_out[ARR_LEN]);
int init_DMA();
u32 checkIdle(u32 baseAddress, u32 offset); // This function is defined to check whether DMA is busy or not.
											// offset = 0x4  to check for DMA to FIFO transaction channel.
											// offset = 0x34 to check for FIFO to MDA transaction channel.

int main()
{
    init_platform();

    printf("---Start of Program---\n");

    if (ARR_LEN % N)
    {
    	printf("ARR_LEN must be a multiple of N!\n");
    	printf("Terminating program...\n");
    	return 0;
    }

    // Timing Variables
	XTime tProcessorStart, tProcessorEnd;
	XTime tFPGAstart, tFPGAend;

	// Array Variables
    float complex Data_in[ARR_LEN];
    float complex Data_rev[ARR_LEN];
    float complex Data_PSout[ARR_LEN];
    float complex Data_PLout[ARR_LEN];

    // Array Initialization
    printf("Start custom wave initialization.\n");
	initialize_customWave(Data_in);
	printf("Custom wave initialization done.\n");

	// Benchmarks
	printf("Benchmarking CPU (single thread)...\n");
	XTime_GetTime(&tProcessorStart);
	bitReverse(Data_in, Data_rev);
	FFT_PS(Data_rev, Data_PSout);
	XTime_GetTime(&tProcessorEnd);
	printf("CPU (single thread) benchmark finished.\n");

	int status_dma = init_DMA();
	if (status_dma != XST_SUCCESS)
	{
		printf("my_error: Can't initialize DMA\n");
		return XST_FAILURE;
	}

	// Flush Cache to Write Back the Data in DDR
	Xil_DCacheFlushRange((UINTPTR)Data_in, (sizeof(float complex)*ARR_LEN));
	Xil_DCacheFlushRange((UINTPTR)Data_PLout, (sizeof(float complex)*ARR_LEN));

	int status, status_transfer;
	printf("Benchmarking FPGA...\n");
	XTime_GetTime(&tFPGAstart);
	// status = 0 and status = 2 both indicate idle
//		printf("DMA status before transfer\n");
//		printf("DMA to Device: %d\n", checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x4));
//		printf("Device to DMA: %d\n", checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x34));
	status_transfer = XAxiDma_SimpleTransfer(
			&AxiDma,
			(UINTPTR)Data_PLout,
			sizeof(float complex)*ARR_LEN,
			XAXIDMA_DEVICE_TO_DMA
			);
	if (status_transfer != XST_SUCCESS)
	{
		printf("my_error: Write data to PL via DMA failed\n");
		return 0;
	}
	// status = 0 and status = 2 both indicate idle
//		printf("DMA status between transfer\n");
//		printf("DMA to Device: %d\n", checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x4));
//		printf("Device to DMA: %d\n", checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x34));

	status_transfer = XAxiDma_SimpleTransfer(
			&AxiDma,
			(UINTPTR)Data_in,
			sizeof(float complex)*ARR_LEN,
			XAXIDMA_DMA_TO_DEVICE
			);
	if (status_transfer != XST_SUCCESS)
	{
		printf("my_error: Read data from PL via DMA failed\n");
		return 0;
	}
	// status = 0 and status = 2 both indicate idle
//		printf("DMA status after transfer\n");
//		printf("DMA to Device: %d\n", checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x4));
//		printf("Device to DMA: %d\n", checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x34));

	do {
		status = checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x4); // DMA to device
	} while (status != 2);
	do {
		status = checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x34); // device to DMA
	} while (status != 2);
	XTime_GetTime(&tFPGAend);
	printf("FPGA benchmark finished.\n");

	printf("Comparing results from CPU and FPGA...\n");
	float diff_r, diff_i;
	bool err_flag = false;
	int j;
	for (j = 0; j < ARR_LEN; j++)
	{
//		printf("CPU: %f + %f i    FPGA: %f + %f i\n",
//				creal(Data_PSout[j]),
//				cimag(Data_PSout[j]),
//				creal(Data_PLout[j]),
//				cimag(Data_PLout[j])
//				);
		diff_i = abs(creal(Data_PSout[j]) - creal(Data_PLout[j]));
		diff_r = abs(cimag(Data_PSout[j]) - cimag(Data_PLout[j]));
		if (diff_r > 0.001 || diff_i > 0.001)
		{
			err_flag = true;
			break;
		}
	}

	if (err_flag)
	{
		printf("Data mismatch found at %d.\n", j);
		printf("Result from CPU : %f + %f i\n", creal(Data_PSout[j]), cimag(Data_PSout[j]));
		printf("Result from FPGA: %f + %f i\n", creal(Data_PLout[j]), cimag(Data_PLout[j]));
	}
	else
	{
		printf("Results from CPU and FPGA all match!\n");
	}

	printf("---Timing---\n");
	float time_diff;
	time_diff = (float)1.0 * (tProcessorEnd - tProcessorStart) / (COUNTS_PER_SECOND/1000000);
	printf("CPU: %f\n", time_diff);
	time_diff = (float)1.0 * (tFPGAend - tFPGAstart) / (COUNTS_PER_SECOND/1000000);
	printf("FPGA: %f\n", time_diff);

    printf("---End of Program---\n");
    cleanup_platform();
    return 0;
}

void initialize_customWave(float complex arr[ARR_LEN])
{
	for (int i = 0; i < ARR_LEN; i+=N)
	{
		arr[i  ] = 11+23*I;
		arr[i+1] = 32+10*I;
		arr[i+2] = 91+94*I;
		arr[i+3] = 15+69*I;
		arr[i+4] = 47+96*I;
		arr[i+5] = 44+12*I;
		arr[i+6] = 96+17*I;
		arr[i+7] = 49+58*I;
	}
}

void bitReverse(float complex dataIn[ARR_LEN], float complex dataOut[ARR_LEN])
{
	int j;
	for (int i = 0; i < ARR_LEN; i+=N)
	{
		for (j = 0; j < N; j++)
		{
			dataOut[i+j] = dataIn[i+bitRevIndex[j]];
		}
	}
}

void FFT_PS(float complex Data_in[ARR_LEN], float complex Data_out[ARR_LEN])
{
	int j, k;
	float complex temp1[N], temp2[N];
	float complex* Data_in_wOffset;
	float complex* Data_out_wOffset;
	float complex tempComplex;
	for (int i = 0; i < ARR_LEN; i+=N)
	{
		Data_in_wOffset = Data_in + i;
		Data_out_wOffset = Data_out + i;
		for (j = 0; j < N; j+=2)
		{
			temp1[j] = Data_in_wOffset[j] + Data_in_wOffset[j+1];
			temp1[j+1] = Data_in_wOffset[j] - Data_in_wOffset[j+1];
		}

		for (j = 0; j < N; j+=4)
		{
			for (k = 0; k < 2; ++k)
			{
				tempComplex = W[2*k]*temp1[j+k+2];
				temp2[j+k] = temp1[j+k] + tempComplex;
				temp2[j+k+2] = temp1[j+k] - tempComplex;
			}
		}

		for (j = 0; j < N/2; j++)
		{
			tempComplex = W[j]*temp2[j+4];
			Data_out_wOffset[j] = temp2[j] + tempComplex;
			Data_out_wOffset[j+4] = temp2[j] - tempComplex;
		}
	}
}

int init_DMA()
{
	XAxiDma_Config* CfgPtr;
	int status;

	CfgPtr = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	if (!CfgPtr)
	{
		printf("my_error: No config found for %d\n", XPAR_AXI_DMA_0_DEVICE_ID);
		return XST_FAILURE;
	}

	status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
	if (status != XST_SUCCESS)
	{
		printf("my_error: DMA Initialization Failed. Return Status: %d\n", status);
		return XST_FAILURE;
	}
	if (XAxiDma_HasSg(&AxiDma)) // Check that DMA isn't in Scatter Gather Mode.
	{
		printf("my_error: Device should not be in Scatter Gather Mode\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

u32 checkIdle(u32 baseAddress, u32 offset)
{
	u32 status;

	status = (XAxiDma_ReadReg(baseAddress, offset)) & XAXIDMA_IDLE_MASK;

	return status;
}
