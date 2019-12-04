/*******************************************************************************
 * @file        dsp_processing.c
 * @brief       C Library for processing the incomming datastream
 * @details     
 * @version     1.0
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.11.28
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
 * @note        <pre>Project Properties > C/C++ > Preprocessor Symbols: ARM_MATH_CM4</pre>
 * @note        <pre>Project Properties > Target > Floating Point Hardware > "Single Precision"</pre>
*******************************************************************************/
/*
Project Properties > C/C++ > Preprocessor Symbols: ARM_MATH_CM4
Project Properties > Target > Floating Point Hardware > "Single Precision"
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <arm_math.h>
#include "fir.h"
#include "dsp_processing.h"
#include <stdio.h>

volatile uint16_t dsp_mode;

void DSP_Process_Data(uint16_t *sourceBuffer, uint16_t *targetBuffer, uint16_t size)
{
#ifdef DEBUG_DSP_LATENCY
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#endif
	static float32_t rxLeft[DSP_BUFFERSIZE_HALF], rxRight[DSP_BUFFERSIZE_HALF];
  static float32_t txLeft[DSP_BUFFERSIZE_HALF], txRight[DSP_BUFFERSIZE_HALF];

	// copy sourceBuffer to leftSignalBuffer and rightSignalBuffer
	uint16_t index1;
	for (index1 = 0; index1 < DSP_BUFFERSIZE_HALF; index1++) {
		rxLeft [index1] = (int16_t)(sourceBuffer[2*index1  ]);  
		rxRight[index1] = (int16_t)(sourceBuffer[2*index1+1]); 
	}
	
	float32_t gain = 2.0f;
	switch(dsp_mode){
		case DSP_MODE_FIR:
			/*
			FIR_Filter_F32_Mono(rxRight, txRight);
			for (uint16_t i = 0; i < DSP_BUFFERSIZE_HALF; i++) {
					txLeft[i] = rxLeft[i];
			}
			*/
			FIR_Filter_F32_Stereo(rxLeft, txLeft, rxRight, txRight);
			break;
		case DSP_MODE_GAIN:
			// Add a gain of +6dB (gain = 2)
		  /** @Bug : The Codec internal gain works by changing the digital values.
				*        Adding too much gain both here and on the Codec makes the uint16_t values overflow
				*        changing the signal. */
			gain = 2.0f;
			for (uint16_t i = 0; i < DSP_BUFFERSIZE_HALF; i++) {
					txLeft[i] = gain*rxLeft[i];
			}
			for (uint16_t i = 0; i < DSP_BUFFERSIZE_HALF; i++) {
					txRight[i] = gain*rxRight[i];
			}
			break;
		case DSP_MODE_PASSTHROUGH:
		default: // DSP_MODE_PASSTHROUGH
			//talk through: just copy input (rx) into output (tx) 
			for (uint16_t i = 0; i < DSP_BUFFERSIZE_HALF; i++) {
					txLeft[i] = rxLeft[i];
			}
			for (uint16_t i = 0; i < DSP_BUFFERSIZE_HALF; i++) {
					txRight[i] = rxRight[i];
			}
			break;
	}
	
	// copy left and right txBuffer into targetBuffer
	for (index1 = 0; index1 < DSP_BUFFERSIZE_HALF; index1++) {
		targetBuffer[2*index1]     = (int16_t)(txLeft [index1]);  
		targetBuffer[2*index1 + 1] = (int16_t)(txRight[index1]); 
	}
#ifdef DEBUG_DSP_LATENCY
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
#endif
}


