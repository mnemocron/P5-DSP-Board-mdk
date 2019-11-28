/*******************************************************************************
 * @file        fir.c
 * @brief       C Library to process FIR Filters using CMSIS/DSP
 * @details     
 * @version     0.1
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.11.28
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
 * @see         https://arm-software.github.io/CMSIS_5/DSP/html/arm_fir_example_f32_8c-example.html#_a12
*******************************************************************************/

#include "main.h"
#include <arm_math.h>
#include "fir.h"
#include "dsp_processing.h"

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
static float32_t firStateF32_L[FIR_BLOCK_SIZE + FIR_NUM_TAPS - 1];
static float32_t firStateF32_R[FIR_BLOCK_SIZE + FIR_NUM_TAPS - 1];
/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
const float32_t firCoeffs32[FIR_NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */
uint32_t blockSize = FIR_BLOCK_SIZE;
uint32_t numBlocks = DSP_BUFFERSIZE_HALF/FIR_BLOCK_SIZE;
float32_t  snr;

/**
  * @brief  This function apply a LP FIR filter in to a F32 data signal.
  * @param  None
  * @retval None
  */
void FIR_PROCESSING_F32Process(float32_t *pSrc1, float32_t *pDst1, float32_t *pSrc2, float32_t *pDst2)
{
	uint32_t i;
  arm_fir_instance_f32 FIR_F32_Struct_L;
  arm_fir_instance_f32 FIR_F32_Struct_R;

	/* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&FIR_F32_Struct_L, FIR_NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32_L[0], blockSize);
  arm_fir_init_f32(&FIR_F32_Struct_R, FIR_NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32_R[0], blockSize);
  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */
  for(i=0; i < numBlocks; i++)
  {
    arm_fir_f32(&FIR_F32_Struct_L, pSrc1 + (i * blockSize), pDst1 + (i * blockSize), blockSize);
    arm_fir_f32(&FIR_F32_Struct_R, pSrc2 + (i * blockSize), pDst2 + (i * blockSize), blockSize);
  }
}

