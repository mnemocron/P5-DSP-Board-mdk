/*******************************************************************************
 * @file        fir.h
 * @brief       C Library to process FIR Filters using CMSIS/DSP
 * @details     
 * @version     0.1
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.11.28
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIR_H
#define __FIR_H
/* Includes ------------------------------------------------------------------*/
#include <arm_math.h>
/* Private defines -----------------------------------------------------------*/

#define FIR_NUM_TAPS_LONG 29
#define FIR_NUM_TAPS 11
#define FIR_BLOCK_SIZE 64

/* Exported functions ------------------------------------------------------- */
void FIR_Filter_F32_Stereo(float32_t *srcLeft, float32_t *dstLeft, float32_t *srcRight, float32_t *dstRight);
void FIR_Filter_F32_Mono(float32_t* srcM, float32_t* dstM);

void FIR_Init_Mono(void);
void FIR_Init_Stereo(void);
/* Private variables ---------------------------------------------------------*/


#ifdef	__cplusplus
}
#endif

#endif /* __FIR_H */
