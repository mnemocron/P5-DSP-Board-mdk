/*******************************************************************************
 * @file        tlv320aic.h
 * @brief       C Library for TLV320aic Audio Codec
 * @details     This file implements the functionalities of the codec. 
 *              The public functions can be used to control the codec.
 * @version     1.0
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.10.09
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
*******************************************************************************/

#ifndef TLV320AIC_H
#define	TLV320AIC_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef STM32F4XX_H
#include "stm32f4xx_hal.h"
#endif

#ifndef STM32F4XX_HAL_I2C_H
#include "stm32f4xx_hal_i2c.h"
#endif

#define CODEC_SAMPLE_RATE 48000

//#define TLV320_ADDR 0x1A    // #CS = 0
//#define TLV320_ADDR 0x1B  // #CS = 1
#define TLV320_ADDR 0x34    // from I2C Scanner
#define TLV320_NUM_REGS 10

#define TLV320_LEFT_IN_VOL_REG             0x00
#define TLV320_RIGHT_IN_VOL_REG            0x01
#define TLV320_LEFT_HP_VOL_REG             0x02
#define TLV320_RIGHT_HP_VOL_REG            0x03
#define TLV320_ANALOG_AUDIO_PATH_REG       0x04
#define TLV320_DIGITAL_AUDIO_PATH_REG      0x05
#define TLV320_POWER_DOWN_REG              0x06
#define TLV320_DIGITAL_AUDIO_IF_FORMAT_REG 0x07
#define TLV320_SAMPLE_RATE_CTRL_REG        0x08
#define TLV320_DIGITAL_IF_ACTIVATION_REG   0x09
#define TLV320_RESET_REG                   0x0F

typedef enum {
    LINE,
    MIC,
} input_t; /**< @typedef input_t
 * @enum LINE   specifies the line input
 * @enum MIC    specifies the microphone input
 */
/**
 * Initialises the codec to its default values.
 */
void TLV320_Init(I2C_HandleTypeDef *);
/**
 * Resets the codec to its default values.
 */
void TLV320_Reset(void);
/**
 * Mutes the codec Input if the bool <em>on</em> is true.
 * @param mute  defines whether the coded should be muted or not
 */
void TLV320_Mute(uint8_t mute);
/**
 * Chooses the input channel for the codec.
 * @param in    is an enum, it can be LINE or MIC
 */
void TLV320_SetInput(input_t in);
/**
 * Activates/Deactivates the + 20 dB boost for the mirophone input.
 * @param boost     defines whether the boost is turned on or not
 */
void TLV320_SetMicboost(uint8_t boost);
/**
 * Sets the line input volume.
 * @param vol   should be a value between 0x1F=12dB and 0x00=-34.5dB (1.5 dB steps)
 * @bug Probably Bug? When setting the Volume to 0x1F, Codec is muted
 */
void TLV320_SetLineInVol(uint16_t vol);
/**
 * Sets the headphone output volume.
 * @param vol   should be a value between 0x4F=6dB and 0x00=-73dB (79 steps)
 * @bug Probably Bug? When setting the Volume to 0x4f Codec is muted
 */
void TLV320_SetHeadphoneVol(uint16_t vol);

#endif	/* TLV320AIC_H */

