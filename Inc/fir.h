/*******************************************************************************
 * @file        dsp_processing.h
 * @brief       C Library for processing the received datastream
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

#define FIR_NUM_TAPS 29

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
float32_t aFIR_F32_Coeffs[FIR_NUM_TAPS] = {
-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
-0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
+0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
+0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

/* Exported functions ------------------------------------------------------- */
void FIR_PROCESSING_F32Process(void);
void FIR_PROCESSING_Q31Process(void);
void FIR_PROCESSING_Q15Process(int LP_or_HP);

/* Private variables ---------------------------------------------------------*/


#ifdef	__cplusplus
}
#endif

#endif /* __FIR_H */
