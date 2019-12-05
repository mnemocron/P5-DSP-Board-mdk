/*******************************************************************************
 * @file        adaptive_fir.c
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

#include "adaptive_fir.h"
#include <arm_math.h>

/**
  * @see https://github.com/kichiki/WaoN/blob/master/fft.c
	* @see "Digital Filters and Signal Processing" 2nd Ed. by L. B. Jackson. 
	*      (1989) Kluwer Academic Publishers. 
	*      (ISBN 0-89838-276-9) 
	*      Sec.7.3 - Windows in Spectrum Analysis
	*/
	
/**
  * @param i index in the window
	* @param N length of window
	*/
float hamming (int i, int N)
{
	return ( 0.54f - 0.46f * cosf (2.0f*PI*(float)i/(float)(N-1)) );
}
/**
  * @param i index in the window
	* @param N length of window
	*/
float blackman (int i, int N)
{
  return ( 0.42f - 0.5f * cosf (2.0f*PI*(float)i/(float)(N-1))
	  + 0.08f * cosf (4.0f*PI*(float)i/(float)(N-1)) );
}
/**
  * @param N  length of impulse response, FIR Filter is of order (N-1)
	* @param ft 
  * @param coeffs pointer to coefficient buffer
  * @note  MATLAB fir1(N, Wn) has order N with N+1 coefficients
  *        here fir1(N, ft, *) has order N-1 with N coefficients
  * @note  uses default Hamming window
	*/
void fir1(int N, float ft, float *coeffs)
{
	fir1_win(N, ft, coeffs, FIR_WIN_HAMMING);
}

/**
  * @param N  length of impulse response, FIR Filter is of order (N-1)
	* @param ft 
  * @param coeffs pointer to coefficient buffer
  * @param win
  * @note  MATLAB fir1(N, Wn) has order N with N+1 coefficients
  *        here fir1(N, ft, *) has order N-1 with N coefficients
	*/
void fir1_win(int N, float ft, float *coeffs, FirWindow_t win)
{
	float wn[N];
	for(int i=0; i<N;i++){
		switch(win){
			case FIR_WIN_HAMMING:
				wn[i] = hamming(i,N);
				break;
			case FIR_WIN_BLACKMAN:
				wn[i] = blackman(i,N);
				break;
			default:
				wn[i] = 1.0f;
		}
	}
	for(int n=0; n<N; n++){
		if(n == N/2){
			coeffs[n] = 2.0f*ft * wn[n];
		} else {
			coeffs[n] = sinf(2.0f*PI*ft*(n-(N/2))) / (PI*(n-(N/2))) * wn[n];
		}
	}
}
