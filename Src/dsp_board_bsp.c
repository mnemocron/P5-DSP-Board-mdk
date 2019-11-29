/*******************************************************************************
 * @file        dsp_board_bsp.c
 * @brief       C Library for Board spesific functionalities (BSP)
 * @details     This file contains definitions for FHNW P5 DSP Board LEDs,
 *              push-buttons hardware resources.
 * @version     1.0
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.11.27
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
*******************************************************************************/

#include <stdio.h>
#include "main.h"
#include "dsp_board_bsp.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>

EncoderValues_t henc1;
EncoderValues_t henc2;

extern ADC_HandleTypeDef hadc1;

/* Read Jack Detect ----------------------------------------------------------*/
/**
  * @param  jack The Jack Connector of which to return the Pin State
	* @return 1 if Jack is connected, 0 if Jack is not connected
	*/
uint8_t BSP_ReadJackConnected(JackType_t jack)
{
	uint8_t value = 0;
	value = BSP_ReadJackPinState(jack);
	
	// MIC Detect Pin is ACTIVE LOW --> invert signal
	if(jack == JACK_MIC){
		if(value)
			value = 0;
		else
			value = 1;
	}
	return value;
}

/* Read Jack Pin State -------------------------------------------------------*/
/**
  * @param  jack The Jack Connector of which to return the Pin State
	* @return GPIO_PIN_SET or GPIO_PIN_RESET
	*/
GPIO_PinState BSP_ReadJackPinState(JackType_t jack)
{
	GPIO_PinState value = GPIO_PIN_RESET;
	switch(jack){
		case JACK_MIC:
			value = HAL_GPIO_ReadPin(DTC_MIC_GPIO_Port, DTC_MIC_Pin);
			break;
		case JACK_HEADPHONE:
			value = HAL_GPIO_ReadPin(DTC_HP_GPIO_Port, DTC_HP_Pin);
			break;
		case JACK_LINE_IN:
			value = HAL_GPIO_ReadPin(DTC_LIN_GPIO_Port, DTC_LIN_Pin);
			break;
		case JACK_LINE_OUT:
			value = HAL_GPIO_ReadPin(DTC_LOUT_GPIO_Port, DTC_LOUT_Pin);
			break;
		default:
			break;
	}
	return value;
}

/* Read Battery Voltage ------------------------------------------------------*/
/**
  * @param  n number of Samples to read for averaging
  * @return floating point number of converted voltage 
  * (voltage divider taken into account)
  */
float BSP_ReadBatteryVoltage(uint8_t n)
{
	uint32_t sum = 0;
	HAL_ADC_Start(&hadc1);
	uint8_t avg = 0;
	
	for(uint8_t i=0; i<n; i++){
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
			sum += HAL_ADC_GetValue(&hadc1);
			avg ++;
		}
	}
	sum /= (uint32_t)avg;
	// 4096 = 3.3V
	// 1 = 3.3/4096;
	// 4.5V correction factor = 0.7333 from Voltage Divider
	return 0.733333f * (float)(sum) * (3.3f/4096.0f);  
}

/* Set max Battery Charge Current --------------------------------------------*/
/**
  * @param  current Parameter to choose between 100mA or 500mA charge current
  */
void BSP_SetBatteryCurrent(ChargeCurrent_t current)
{
	if(current == CHARGE_CURRENT_500MA)
		HAL_GPIO_WritePin(SET_I_LIM_GPIO_Port, SET_I_LIM_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(SET_I_LIM_GPIO_Port, SET_I_LIM_Pin, GPIO_PIN_RESET);
}

/* Read Encoder Delta --------------------------------------------------------*/
/**
  * @param  encoder The encoder of which to return the difference
  * @return signed integer of encoder difference since last call of this function
  */
int16_t BSP_ReadEncoder_Difference(EncoderPosition_t encoder)
{
	BSP_ReadEncoder(encoder); // update both encoders
	int16_t delta = 0;
	if(encoder == ENCODER_LEFT){
		delta = henc1.delta; // return accumulated delta
		henc1.delta = 0;  // clear for new accumulation
	} else {
		delta = henc2.delta;
		henc2.delta = 0;
	}
	return delta;
}

/* Read Encoder Value --------------------------------------------------------*/
/**
  * @param  encoder The encoder of which to return the absolute counter value
  * @return unsigned integer of encoder's counter value
  */
uint16_t BSP_ReadEncoder(EncoderPosition_t encoder)
{
	int16_t oldval = (int16_t)henc1.value;  // remember previous value
	henc1.value = (((0xffff - TIM3->CNT)/2) +1) & 0x7fff;  // right encoder
	int16_t delta = (int16_t)henc1.value - oldval;  // calculate difference
	if(delta > 255) // overflow occured
		delta = (int16_t)henc1.value - (INT16_MAX+1) - oldval;
	if(delta < -255) // underflow occured
		delta = (int16_t)henc1.value + (INT16_MAX+1) - oldval;
	henc1.delta += delta;  // update difference
	
	oldval = (int16_t)henc2.value;  // remember previous value
	henc2.value = (((         TIM4->CNT)/2)   ) & 0x7fff;     // left encoder
	delta = (int16_t)henc2.value - oldval;  // calculate difference
	if(delta > 255) // overflow occured
		delta = (int16_t)henc2.value - (INT16_MAX+1) - oldval;
	if(delta < -255) // underflow occured
		delta = (int16_t)henc2.value + (INT16_MAX+1) - oldval;
	henc2.delta += delta;  // update difference

	if(encoder == ENCODER_LEFT)
		return henc1.value;
	else
		return henc2.value;
}

/* Select Audio Source -------------------------------------------------------*/
/**
  * @param  mode The Audio Input Mode
  * @TODO   also implement MIC as Audio Source
  */ 
void BSP_SelectAudioIn(uint8_t mode)
{
	if(mode == AUDIO_IN_EXT){
		HAL_GPIO_WritePin(SET_LIN_GPIO_Port, SET_LIN_Pin, GPIO_PIN_RESET);
	} else {
		// AUDIO_IN_LINE
		// default
		HAL_GPIO_WritePin(SET_LIN_GPIO_Port, SET_LIN_Pin, GPIO_PIN_SET);
	}
}

/* SineWave Generator --------------------------------------------------------*/
/** 
  * @param  fs Floating Point number of Sampling Rate
  * @param  fout Floating Point number of desired output frequency
  * @param  A Desired Amplitude for int16 Format
  * @param  pData Pointer to Start of Output Databuffer
  * @param  len Size of Output Buffer
  * @return n_samples The actual Buffer Size used by the Signal
  */
uint16_t BSP_SineWave(float fs, float fout, uint16_t A, uint16_t* pData, uint16_t len)
{
	float Ts   = 1.0f/fs;   // Sampling Period
	float Tsig = 1.0f/fout; // Signal Period
	float omega_sig = 2*BSP_PI*fout; // Signal Frequency in rad/s
	
	uint16_t n_samples = (uint16_t)( Tsig/Ts );  // number of Data Points
	
	// check if number of samples exceeds output buffer length
	if(len < (2*n_samples))
		n_samples = len/2;
	
	for(uint16_t i = 0; i<n_samples; i++){
		float dt = ((float) i ) * Ts;
		float sinVal = sinf( omega_sig * dt );
		pData[2*i]     = (uint16_t)(  (1 + sinVal) * ((float)A)  );  // Right Channel
		pData[2*i + 1] = (uint16_t)(  (1 + sinVal) * ((float)A)  );  // Left  Channel
	}
	return n_samples;
}

/* Print MATLAB --------------------------------------------------------------*/
/**
  * @param  vals Pointer to Input int Values
  * @param  len Size of Input Buffer
  */ 
void BSP_Print_to_Matlab(uint16_t* vals, uint16_t len)
{
	printf("x = [");
	for(uint16_t i = 0; i < len; i++){
		if(i) printf(", ");  // print colon prior to value for i>0
		printf("%d", vals[i]);  // @TODO: is this correct dereference ?
	}
	printf(" ];\n");
	printf("plot(x); grid on;\n\n");
}

/* Scan I2C ------------------------------------------------------------------*/
/**
  * @param  hi2c I2C Handle to Scan on
  */
void BSP_I2C_ScanAddresses(I2C_HandleTypeDef *hi2c)
{
	uint8_t error, address;
	uint16_t nDevices;
	nDevices = 0;
	printf("Scanning for available I2C devices...\n");
	for(address = 1; address < 127; address++ )
	{
		HAL_Delay(10);
		error = HAL_I2C_Master_Transmit(hi2c, address, 0x00, 1, 1);
		//error = HAL_I2C_Master_Receive(hi2c, address, 0x00, 1, 1);
		if (error == HAL_OK)
		{
			printf("I2C device found at address 0x");
			if (address<16)
				printf("0");
			printf("%X", address);
			printf("  !\n");
			nDevices++;
		}
	}
	if (nDevices == 0)
		printf("No I2C devices found\n");
	else
		printf("done\n");
}

