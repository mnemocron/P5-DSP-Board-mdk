
/*******************************************************************************
 * @file        dsp_board_bsp.h
 * @brief       C Library for Board spesific functionalities (BSP)
 * @details     This file contains definitions for FHNW P5 DSP Board LEDs,
 *              push-buttons hardware resources.
 * @version     0.1
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.11.27
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DSP_BOARD_BSP_H
#define __DSP_BOARD_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"


/* Private defines -----------------------------------------------------------*/
/*
#define SET_LIN_Pin GPIO_PIN_13
#define SET_LIN_GPIO_Port GPIOC
#define DTC_LIN_Pin GPIO_PIN_14
#define DTC_LIN_GPIO_Port GPIOC
#define DTC_MIC_Pin GPIO_PIN_15
#define DTC_MIC_GPIO_Port GPIOC
#define DTC_HP_Pin GPIO_PIN_0
#define DTC_HP_GPIO_Port GPIOC
#define DTC_LOUT_Pin GPIO_PIN_1
#define DTC_LOUT_GPIO_Port GPIOC
#define I2S2_DIN_Pin GPIO_PIN_2
#define I2S2_DIN_GPIO_Port GPIOC
#define I2S2_DOUT_Pin GPIO_PIN_3
#define I2S2_DOUT_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOA
#define SW1_EXTI_IRQn EXTI0_IRQn
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOA
#define SW2_EXTI_IRQn EXTI1_IRQn
#define ADC_BAT_Pin GPIO_PIN_1
#define ADC_BAT_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define BTN_ENC2_Pin GPIO_PIN_13
#define BTN_ENC2_GPIO_Port GPIOB
#define BTN_ENC2_EXTI_IRQn EXTI15_10_IRQn
#define SET_I_LIM_Pin GPIO_PIN_14
#define SET_I_LIM_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define BTN_ENC1_Pin GPIO_PIN_12
#define BTN_ENC1_GPIO_Port GPIOC
#define BTN_ENC1_EXTI_IRQn EXTI15_10_IRQn
#define SET_BOOT0_Pin GPIO_PIN_2
#define SET_BOOT0_GPIO_Port GPIOD
#define ENC2_B_Pin GPIO_PIN_4
#define ENC2_B_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_5
#define ENC2_A_GPIO_Port GPIOB
#define ENC1_A_Pin GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOB
*/

#define ADDR_CODEC 0x34
#define ADDR_OLED  0x78
#define BSP_PI 3.1415926f

typedef enum { 
	AUDIO_IN_EXT, 
	AUDIO_IN_LINE
} AudioInState_t;

typedef enum { 
	ENCODER_RIGHT, 
	ENCODER_LEFT 
} EncoderPosition_t;

typedef enum {
	CHARGE_CURRENT_100MA,
	CHARGE_CURRENT_500MA
} ChargeCurrent_t;

typedef struct {
	uint16_t value;
	 int16_t delta;
} EncoderValues_t;

void BSP_I2C_ScanAddresses(I2C_HandleTypeDef*);
void BSP_Print_to_Matlab(uint16_t*, uint16_t);
uint16_t BSP_SineWave(float, float, uint16_t, uint16_t*, uint16_t);
void BSP_SelectAudioIn(uint8_t);
uint16_t BSP_ReadEncoder(EncoderPosition_t);
int16_t BSP_ReadEncoder_Difference(EncoderPosition_t);
void BSP_SetBatteryCurrent(ChargeCurrent_t);
float BSP_ReadBatteryVoltage(uint16_t*, uint8_t);

#ifdef __cplusplus
}
#endif

#endif /* __DSP_BOARD_BSP_H */
