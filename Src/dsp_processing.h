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
#ifndef __DSP_PROCESSING_H
#define __DSP_PROCESSING_H

#ifdef	__cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define DSP_BLOCK_SIZE 32
#define DSP_BUFFERSIZE 256
#define DSP_BUFFERSIZE_HALF 128

typedef struct {
	uint16_t pTxData[DSP_BUFFERSIZE],
	uint16_t pRxData[DSP_BUFFERSIZE]
} DSPBuffer_t;

enum {
	DSP_MODE_PASSTHROUGH,
	DSP_MODE_FIR,
	DSP_MODE_IIR,
	DSP_MODE_GAIN
};


void DSP_Process_Data(uint16_t *sourceBuffer, uint16_t *targetBuffer, uint16_t size);

#ifdef	__cplusplus
}
#endif

#endif // __DSP_PROCESSING_H


