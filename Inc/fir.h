/*******************************************************************************
 * @file        fir.h
 * @brief       C Library to process FIR Filters using CMSIS/DSP
 * @details     
 * @version     1.0
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.12.05
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIR_H
#define __FIR_H
/* Includes ------------------------------------------------------------------*/
#include <arm_math.h>  // float32_t
/* Private defines -----------------------------------------------------------*/

#define FIR_NUM_TAPS_MONO    29  // Anzahl Filterkoeffizienten
#define FIR_NUM_TAPS_STEREO  11  // Anzahl Filterkoeffizienten
#define FIR_NUM_TAPS_ADAPTIVE 29 // Anzahl Filterkoeffizienten für adaptives Filter
#define FIR_BLOCK_SIZE DSP_BUFFERSIZE_HALF // Anzahl zu verarbeitende Samples 

/* Private Function Prototypes -----------------------------------------------*/
void FIR_Filter_F32_Adaptive(float32_t *srcLeft, float32_t *dstLeft, float32_t *srcRight, float32_t *dstRight);
void FIR_Filter_F32_Stereo(float32_t *srcLeft, float32_t *dstLeft, float32_t *srcRight, float32_t *dstRight);
void FIR_Filter_F32_Mono(float32_t* srcM, float32_t* dstM);

void FIR_Init_Mono(void);
void FIR_Init_Stereo(void);
void FIR_Init_Adaptive(void);
void FIR_Update_Adaptive(float32_t *coeffs, uint16_t N);

#ifdef	__cplusplus
}
#endif

#endif /* __FIR_H */
