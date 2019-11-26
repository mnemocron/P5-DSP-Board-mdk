
/*******************************************************************************
 * @file        tlv320aic.c
 * @brief       C Library for TLV320aic Audio Codec
 * @details     This file implements the functionalities of the codec. 
 *              The public functions can be used to control the codec.
 * @version     0.1
 * @author      Simon Burkhardt
 * @author      Mischa Studer
 * @date        2019.10.09
 * @copyright   (c) 2019 Fachhochschule Nordwestschweiz FHNW
 *              all rights reserved
 * @note        EIT Projekt 5 - HS19 - "DSP Board", Betreuer: Markus Hufschmid
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "tlv320aic.h"

// extern I2C_HandleTypeDef hi2c3;

/**************************************************************************
 * Global Definitions
 ***************************************************************************/

I2C_HandleTypeDef *hi2cCODEC;

// ???
// internal 12MHz oscillator : on
// --> external OSC provides clock

const uint16_t TLV320_init_data[10] = {
	0x117, // Reg 00: Left  Line In (L/R simulataneous update, Mute Off, +0dB)
	0x117, // Reg 01: Right Line In (L/R simulataneous update, Mute Off, +0dB)
  0x06C, // Reg 02: Left Headphone out (-12dB)
  0x06C, // Reg 03: Right Headphone out (-12dB)
  0x012, // Reg 04: Analog Audio Path Control (DAC sel, Mute Mic)
	0x000, // Reg 05: Digital Audio Path Control
	0x000, // Reg 06: Power Down Control (All ON)
	0x041, // Reg 07: Digital Audio Interface (Master, 16-bit, MSB-first left allign)
	0x000, // Reg 08: Sample Rate Control (1:1 MCLK out, 1:1 MCLK in, 48kHz)
	0x001  // Reg 09: Active Control
};

unsigned long samplerate=CODEC_SAMPLE_RATE;

/**************************************************************************
 * Private Functions
 ***************************************************************************/
/**
 * Writes the configuration in <em>data</em> into the <em>register</em>.
 * @param reg   is a register address, the makros defined in <em>tlv320aic.h</em> 
 *              should be used.
 * @param data  is a one byte value depending on the expected inputs of the register.
 */
void _tlv320_write8(uint8_t reg, uint16_t data)
{
    // @see Datasheet Fig 3-2. 2-Wire Compatible Timing
    // [s]  [    address   r/w] [ register     ][              data  ]
    //  .   [ 7 6 5 4 3 2 1 0 ] [ 6 5 4 3 2 1 0 8 ] [ 7 6 5 4 3 2 1 0]
    // register address = 7 bit
    // register data    = 9 bits

    // stuff the output buffer
    reg = ((reg << 1)&0xFE) | ((data >> 8)&0x01);  // append data MSB to reg
    uint8_t val = data % 0xFF;

    // Send it
    // i2c_write(wdata, 3);
    HAL_I2C_Mem_Write(hi2cCODEC, TLV320_ADDR, reg, 1, &val, 1, 10);
}

/**************************************************************************
 * Initiation
 ***************************************************************************/

void TLV320_Init(I2C_HandleTypeDef *hi2c)
{
	  hi2cCODEC = hi2c;
    uint8_t i;
    // Reset
    TLV320_Reset();
    HAL_Delay(10);
    // Configure registers
    for (i = 0; i < TLV320_NUM_REGS; i++)
    {
        _tlv320_write8(i, TLV320_init_data[i]);
    }
    TLV320_SetInput(LINE);
}

/**************************************************************************
 * Public Functions
 ***************************************************************************/

void TLV320_Mute(uint8_t on)
{
    if (on)
    {
        // Reg 00: Left Line In (-34.5dB, mute ON)
        _tlv320_write8(TLV320_LEFT_IN_VOL_REG, 0x080);
        // Reg 01: Right Line In (-34.5dB, mute ON)
        _tlv320_write8(TLV320_RIGHT_IN_VOL_REG, 0x080);
    }
    else
    {
        // Reg 00: Left Line In (6dB, mute OFF)
        _tlv320_write8(TLV320_LEFT_IN_VOL_REG, 0x01B);
        // Reg 01: Right Line In (6dB, mute OFF)
        _tlv320_write8(TLV320_RIGHT_IN_VOL_REG, 0x01B);
    }

}

void TLV320_SetInput(input_t in)
{
    switch (in)
    {
    case LINE:
        // Reg 04: Analog Audio Path Control (DAC sel, Mute Mic)
        _tlv320_write8(TLV320_ANALOG_AUDIO_PATH_REG, 0x012);
        break;

    case MIC:
    default:
        // Reg 04: Analog Audio Path Control (DAC sel, Mic, 20dB boost)
        _tlv320_write8(TLV320_ANALOG_AUDIO_PATH_REG, 0x015);
        break;
    }
}

void TLV320_SetMicboost(uint8_t boost)
{
    uint8_t value;
    value = boost ? 0x015 : 0x014;
    // Reg 04: Analog Audio Path Control (DAC sel, Mic, 20dB boost)
    _tlv320_write8(TLV320_ANALOG_AUDIO_PATH_REG, value);
}

void TLV320_SetLineInVol(uint16_t vol)
{
    if (vol > 0x01F)
        vol = 0x01F;
    _tlv320_write8(TLV320_LEFT_IN_VOL_REG, 0x100 | (vol & 0x01F));
    _tlv320_write8(TLV320_RIGHT_IN_VOL_REG, 0x100 | (vol & 0x01F));
}

void TLV320_SetHeadphoneVol(uint16_t vol)
{
    vol += 0x030;
    if (vol > 0x07F)
        vol = 0x07F;
    _tlv320_write8(TLV320_LEFT_HP_VOL_REG, 0x100 | vol);
    _tlv320_write8(TLV320_RIGHT_HP_VOL_REG, 0x100 | vol);
}

void TLV320_Reset(void)
{
    _tlv320_write8(TLV320_RESET_REG, 0);
}




