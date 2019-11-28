
/*

Project Properties > C/C++ > Preprocessor Symbols: ARM_MATH_CM4
*/

#include "main.h"
#include <arm_math.h>
#include "fir.h"
#include "dsp_processing.h"

volatile uint16_t dsp_mode = DSP_MODE_GAIN;

void DSP_Process_Data(uint16_t *sourceBuffer, uint16_t *targetBuffer, uint16_t size)
{
	
	static float32_t rxLeft[DSP_BUFFERSIZE_HALF], rxRight[DSP_BUFFERSIZE_HALF];
  static float32_t txLeft[DSP_BUFFERSIZE_HALF], txRight[DSP_BUFFERSIZE_HALF];

	// copy sourceBuffer to leftSignalBuffer and rightSignalBuffer
	uint16_t index1, index2;
	for (index1 = 0, index2 = 0; index1 < DSP_BUFFERSIZE_HALF; index1++) {
			rxLeft[index1]  = (float32_t)sourceBuffer[index2++];
			rxRight[index1] = (float32_t)sourceBuffer[index2++];
	}
	
	float32_t gain = 2.0f;
	switch(dsp_mode){
		case DSP_MODE_FIR:
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
	
	//copy left and right txBuffer into targetBuffer
	for (index1 = 0, index2 = 0; index1 < DSP_BUFFERSIZE_HALF; index1++) {
			targetBuffer[index2++] = (uint16_t)txLeft[index1];
			targetBuffer[index2++] = (uint16_t)txRight[index1];
	}
}


