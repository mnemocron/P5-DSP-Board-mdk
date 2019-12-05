/*******************************************************************************
 * @file        fir.c
 * @brief       C Library to process FIR Filters using CMSIS/DSP
 * @details     
 * @version     1.0
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.12.05
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
 * @see         https://arm-software.github.io/CMSIS_5/DSP/html/arm_fir_example_f32_8c-example.html#_a12
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <arm_math.h>
#include "fir.h"
#include "dsp_processing.h"

/* Private variables ---------------------------------------------------------*/

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
 /** @important each FIR Filter uses its own buffer */
static float32_t firStateF32_L[FIR_BLOCK_SIZE + FIR_NUM_TAPS_STEREO - 1];
static float32_t firStateF32_R[FIR_BLOCK_SIZE + FIR_NUM_TAPS_STEREO - 1];
static float32_t firStateF32_Mono[FIR_BLOCK_SIZE + FIR_NUM_TAPS_MONO - 1];
static float32_t firStateF32_Adaptive_L[FIR_BLOCK_SIZE + FIR_NUM_TAPS_ADAPTIVE -1];
static float32_t firStateF32_Adaptive_R[FIR_BLOCK_SIZE + FIR_NUM_TAPS_ADAPTIVE -1];

uint32_t blockSize = FIR_BLOCK_SIZE;
uint32_t numBlocks = DSP_BUFFERSIZE_HALF;

arm_fir_instance_f32 FIR_F32_Struct_L;
arm_fir_instance_f32 FIR_F32_Struct_R;
arm_fir_instance_f32 FIR_F32_Struct_Mono;
arm_fir_instance_f32 FIR_F32_Struct_Adapt_L;
arm_fir_instance_f32 FIR_F32_Struct_Adapt_R;

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 1000/48000)
** ------------------------------------------------------------------- */
float32_t firCoeffs32_adaptive[FIR_NUM_TAPS_ADAPTIVE] = {
	0.004662909902645, 0.005444886609392, 0.007609635193228, 0.011119448232003,
	0.015853681950732, 0.021613824789871, 0.028133632645981, 0.035093765687500,
	0.042140052015045, 0.048904249412152, 0.055025996207495, 0.060174549002798,
	0.064068905869162, 0.066495009187560, 0.067318906588870, 0.066495009187560,
	0.064068905869162, 0.060174549002798, 0.055025996207495, 0.048904249412152,
	0.042140052015045, 0.035093765687500, 0.028133632645981, 0.021613824789871,
	0.015853681950732, 0.011119448232003, 0.007609635193228, 0.005444886609392,
	0.004662909902645
};

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
const float32_t firCoeffs32_long[FIR_NUM_TAPS_MONO] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(10, 6/24)
** ------------------------------------------------------------------- */
const float32_t firCoeffs32[FIR_NUM_TAPS_STEREO] = {
	-0.003871323167475, 0.000000000000000, 0.032087799410030, 0.116708621643743,
   0.220701186106900, 0.268747432013603, 0.220701186106900, 0.116708621643743,
   0.032087799410030, 0.000000000000000, -0.003871323167475 
};

/* FIR Init ------------------------------------------------------------------*/
/**
  * @important must be called before the FIR Filter is called
  */
void FIR_Init_Adaptive(void)
{
	arm_fir_init_f32(&FIR_F32_Struct_Adapt_L, FIR_NUM_TAPS_ADAPTIVE, (float32_t *)&firCoeffs32_adaptive[0], &firStateF32_Adaptive_L[0], blockSize);
	arm_fir_init_f32(&FIR_F32_Struct_Adapt_R, FIR_NUM_TAPS_ADAPTIVE, (float32_t *)&firCoeffs32_adaptive[0], &firStateF32_Adaptive_R[0], blockSize);
}

/* FIR Init ------------------------------------------------------------------*/
/**
  * @important must be called before the FIR Filter is called
  */
void FIR_Init_Stereo(void)
{
	/* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&FIR_F32_Struct_L, FIR_NUM_TAPS_STEREO, (float32_t *)&firCoeffs32[0], &firStateF32_L[0], blockSize);
  arm_fir_init_f32(&FIR_F32_Struct_R, FIR_NUM_TAPS_STEREO, (float32_t *)&firCoeffs32[0], &firStateF32_R[0], blockSize);
}

/* FIR Init ------------------------------------------------------------------*/
/**
  * @important must be called before the FIR Filter is called
  */
void FIR_Init_Mono(void)
{
	arm_fir_init_f32(&FIR_F32_Struct_Mono, FIR_NUM_TAPS_MONO, (float32_t *)&firCoeffs32_long[0], &firStateF32_Mono[0], blockSize);
}

/* Process one Block through the FIR -----------------------------------------*/
/**
  * @param srcLeft  Pointer to left  Audio Channel Source Buffer
  * @param dstLeft  Pointer to left  Audio Channel Destination Buffer
  * @param srcRight Pointer to right Audio Channel Source Buffer
  * @param dstRight Pointer to right Audio Channel Destination Buffer
  * @pre   Filter must be properly initialized prior to calling this function
  */
void FIR_Filter_F32_Adaptive(float32_t *srcLeft, float32_t *dstLeft, float32_t *srcRight, float32_t *dstRight)
{
  arm_fir_f32(&FIR_F32_Struct_Adapt_L, srcLeft, dstLeft, blockSize);
  arm_fir_f32(&FIR_F32_Struct_Adapt_R, srcRight, dstRight, blockSize);
}

/* Process one Block through the FIR -----------------------------------------*/
/**
  * @param srcLeft  Pointer to left  Audio Channel Source Buffer
  * @param dstLeft  Pointer to left  Audio Channel Destination Buffer
  * @param srcRight Pointer to right Audio Channel Source Buffer
  * @param dstRight Pointer to right Audio Channel Destination Buffer
  * @pre   Filter must be properly initialized prior to calling this function
  */
void FIR_Filter_F32_Stereo(float32_t *srcLeft, float32_t *dstLeft, float32_t *srcRight, float32_t *dstRight)
{
  arm_fir_f32(&FIR_F32_Struct_L, srcLeft, dstLeft, blockSize);
  arm_fir_f32(&FIR_F32_Struct_R, srcRight, dstRight, blockSize);
}

/* Process one Block through the FIR -----------------------------------------*/
/**
  * @param srcM Pointer to Signal Source Buffer
  * @param dstM Pointer to Signal Destination Buffer
  * @pre   Filter must be properly initialized prior to calling this function
  */
void FIR_Filter_F32_Mono(float32_t* srcM, float32_t* dstM)
{
  arm_fir_f32(&FIR_F32_Struct_Mono, srcM, dstM, blockSize);
}

/* Update adaptive FIR Filter Params -----------------------------------------*/
/**
  * @param coeffs Pointer to new FIR Coefficients
  * @param N length number of new Coefficients
  * @note  if N is greater than the Filter's own buffer, the additional values are ignored
  */
void FIR_Update_Adaptive(float32_t *coeffs, uint16_t N)
{
	for(uint16_t i=0; i<N || i<FIR_NUM_TAPS_ADAPTIVE; i++){
		firCoeffs32_adaptive[i] = *(coeffs + i);
	}
	FIR_Init_Adaptive();
}

