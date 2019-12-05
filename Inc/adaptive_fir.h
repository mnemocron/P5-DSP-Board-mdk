/*******************************************************************************
 * @file        adaptive_fir.h
 * @brief       C Library to derrive FIR low pass coefficients
 * @details     Uses Windowing approach (sin(x)/(x))
 * @version     1.0
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.12.05
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
 * @see         http://www.labbookpages.co.uk/audio/firWindowing.html
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADAPTIVE_FIR_H
#define __ADAPTIVE_FIR_H

#ifdef	__cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private Typedefs ----------------------------------------------------------*/
typedef enum {
	FIR_WIN_RECT,
	FIR_WIN_HAMMING,
	FIR_WIN_BLACKMAN
} FirWindow_t;

/* Private Function Prototypes -----------------------------------------------*/
float hamming (int i, int N);
float blackman (int i, int N);
void fir1(int N, float ft, float *coeffs);
void fir1_win(int N, float ft, float *coeffs, FirWindow_t win);

#ifdef	__cplusplus
}
#endif

#endif // __ADAPTIVE_FIR_H
