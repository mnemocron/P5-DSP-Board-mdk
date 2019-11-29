/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ssd1306.h"
#include "tlv320aic.h"
#include "dsp_board_bsp.h"
#include "dsp_processing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SSD1306_t holedR;
SSD1306_t holedL;

uint16_t pTxData[DSP_BUFFERSIZE];
uint16_t pRxData[DSP_BUFFERSIZE];
uint16_t sinWave[256];

volatile uint8_t btnLeftPressed = 0;
volatile uint8_t btnRightPressed = 0;
volatile uint8_t encLeftPressed = 0;
volatile uint8_t encRightPressed = 0;
volatile uint8_t dmaTransferComplete = 0;

extern volatile uint16_t dsp_mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_RNG_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* PRINTF REDIRECT to UART BEGIN */
// @see    http://www.keil.com/forum/60531/
// @see    https://stackoverflow.com/questions/45535126/stm32-printf-redirect

struct __FILE{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

FILE __stdout;

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int ferror(FILE *f){
  /* Your implementation of ferror(). */
  return 0;
}
/* PRINTF REDIRECT to UART END */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_I2S2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_RNG_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	printf("DSP Board!\n");
	printf("Initializing.");
	
	/* Start ADC for Battery Voltage */
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
		HAL_ADC_GetValue(&hadc1);
	}
	printf(".");
	
	/* Set up SSD1306 OLED Displays on both I2C Busses */
	holedR.hi2cx = &hi2c3;
	holedL.hi2cx = &hi2c1;
	ssd1306_Init(&holedL);
	ssd1306_Fill(&holedL, Black);
	ssd1306_SetCursor(&holedL, 2, 20);
	ssd1306_WriteString(&holedL, "Line Vol", Font_11x18, White);
	ssd1306_UpdateScreen(&holedL);
	ssd1306_Init(&holedR);
	ssd1306_Fill(&holedR, Black);
	ssd1306_SetCursor(&holedR, 2, 20);
	ssd1306_WriteString(&holedR, "Input", Font_11x18, White);
	ssd1306_UpdateScreen(&holedR);
	printf(".");
	
	/* Start both Timers in Encoder Mode for Rotary Encoders */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // start encoder mode
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	TIM3->CNT = 0; // initialize zero
	TIM4->CNT = 0;
	printf(".");

	/* Init TLV320 Audio Codec */
	TLV320_Init(&hi2c1);
	TLV320_Mute(0);       // mute off
	TLV320_SetInput(LINE);
	TLV320_SetLineInVol(0x01F);
	TLV320_SetHeadphoneVol(0x04F);
	printf(".");
	
	/* Clear Audio Rx and Tx buffer for DMA */
	for(uint8_t i=0; i<128; i++){
		pRxData[i] = 0;
		pTxData[i] = 0;
	}
	/* Start automatic DMA Transmission (Full Duplex) */
	HAL_I2SEx_TransmitReceive_DMA(&hi2s2, pTxData, pRxData, DSP_BUFFERSIZE);
	printf(".");
  
	/* Set the Analog Signal Switch to choose LINE IN as Input */
	BSP_SelectAudioIn(AUDIO_IN_LINE);
	ssd1306_SetCursor(&holedR, 10, 40);
	ssd1306_WriteString(&holedR, "Line IN  ", Font_11x18, White);
	ssd1306_UpdateScreen(&holedR);
	
	printf("\ndone!\n");
	
	/* Generate a 1kHz Sine Wave */
	uint16_t nDataPoints = BSP_SineWave(48000.0f, 1000.0f, 1000, sinWave, sizeof(sinWave)/sizeof(uint16_t));
	BSP_Print_to_Matlab(sinWave, nDataPoints*2);
	
	uint8_t input_state = 0;
	uint8_t update_counter = 0;
	
	uint16_t volume_line = 0, volume_hp = 0;
	float volume_dB = -34.5f;
	uint8_t newvolume_line = BSP_ReadEncoder(ENCODER_LEFT) % (0x1F+1);
	
	char lcd_buf[18];
	sprintf(lcd_buf, "%.1f dB  ", volume_dB);
	ssd1306_SetCursor(&holedL, 10, 40);
	ssd1306_WriteString(&holedL, lcd_buf, Font_11x18, White);
	ssd1306_UpdateScreen(&holedL);
	
	enum { STATE_LINE, STATE_HEADPHONE };
	uint8_t state_now = STATE_LINE;
	uint8_t state_next = STATE_LINE;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		float volt = BSP_ReadBatteryVoltage(10);
		printf("%.3f\n", volt);
		
		/* STATE MACHINE */
		uint16_t encoder_change = BSP_ReadEncoder_Difference(ENCODER_LEFT);
		if(encoder_change){    // only execute when something changed
			switch(state_now){
				/* LINE OUT Volume */
				case STATE_LINE:
					volume_line += encoder_change;    // add the Encoder Difference to the Volume (positive or negative)
					if(volume_line > 0x01f)           // Boundry Check
						volume_line = 0x01f;
					TLV320_SetLineInVol(volume_line); // Set the Volume on the Codec
					// from -34.5dB to +12dB in 1.5 dB Steps
					volume_dB = -34.5f + (1.5f*(float)volume_line);
					printf("[VOL LIN]:\t%.1f dB\n", volume_dB);
					break;
				/* HEADPHONE Volume */
				case STATE_HEADPHONE:
					volume_hp += encoder_change;
					if(volume_hp > 79)
						volume_hp = 79;
					TLV320_SetHeadphoneVol(volume_hp);
					// from -73dB to +6dB in 1dB Steps
					volume_dB = -73.0 + (1.0f*(float)volume_hp);
					printf("[VOL HP] :\t%.1f dB\n", volume_dB);
					break;
				default:
					state_next = STATE_LINE;
			}
			// Display new Value on 2nd line on OLED
			sprintf(lcd_buf, "%.1f dB  ", volume_dB);
			ssd1306_SetCursor(&holedL, 10, 40);
			ssd1306_WriteString(&holedL, lcd_buf, Font_11x18, White);
			ssd1306_UpdateScreen(&holedL);
		}
		
		/* LEFT USER BUTTON */
		if(btnLeftPressed){
			btnLeftPressed= 0;
			printf("[BTN] Button Left\n");
		}
		
		/* RIGHT USER BUTTON */
		if(btnRightPressed){
			btnRightPressed= 0;
			input_state = 1-input_state;
			printf("[BTN] :\tButton Right\n");
			
			/* Switch Line Input between external Board and 3.5mm Jack */
			if(input_state){
				printf("[AUDIO] :\tExt Line\n");
				BSP_SelectAudioIn(AUDIO_IN_EXT);  // select external Source
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				sprintf(lcd_buf, "EXT Line ");
				
			} else {
				printf("[AUDIO] :\tLine In\n");
				BSP_SelectAudioIn(AUDIO_IN_LINE); // select Jack Source
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				sprintf(lcd_buf, "Line IN  ");
			}
			ssd1306_SetCursor(&holedR, 10, 40);
			ssd1306_WriteString(&holedR, lcd_buf, Font_11x18, White);
			ssd1306_UpdateScreen(&holedR);
		}
		
		/* LEFT ENCODER BUTTON */
		if(encLeftPressed){
			encLeftPressed = 0;
			printf("[BTN] :\tEncoder Left\n");
			
			/* Switch Volume Control State */
			ssd1306_SetCursor(&holedL, 2, 20);
			if(state_now == STATE_LINE){
				state_next = STATE_HEADPHONE;
				ssd1306_WriteString(&holedL, "HP   Vol", Font_11x18, White);
				printf("[MODE]:\tHeadphone\n");
				volume_dB = -73.0 + (1.0f*(float)volume_hp);
			} else {
				state_next = STATE_LINE;
				ssd1306_WriteString(&holedL, "Line Vol", Font_11x18, White);
				printf("[MODE]:\tLine\n");
				volume_dB = -34.5f + (1.5f*(float)volume_line);
			}
			
			sprintf(lcd_buf, "%.1f dB  ", volume_dB);
			ssd1306_SetCursor(&holedL, 10, 40);
			ssd1306_WriteString(&holedL, lcd_buf, Font_11x18, White);
			ssd1306_UpdateScreen(&holedL);
			BSP_ReadEncoder_Difference(ENCODER_LEFT); // make sure delta is empty on state change
		}
		
		/* RIGHT ENCODER BUTTON */
		if(encRightPressed){
			encRightPressed = 0;
			printf("[BTN] :\tEncoder Right\n");
			if(dsp_mode == DSP_MODE_FIR){
				dsp_mode = DSP_MODE_PASSTHROUGH;
				sprintf(lcd_buf, "Passthru");
			} else {
				dsp_mode = DSP_MODE_FIR;
				sprintf(lcd_buf, "FIR     ");
			}
			ssd1306_SetCursor(&holedR, 10, 0);
			ssd1306_WriteString(&holedR, lcd_buf, Font_11x18, White);
			ssd1306_UpdateScreen(&holedR);
		}
		
		/* do something every 1s without user interaction */
		if(update_counter > 100){
			update_counter = 0;
			
			printf("[JACK] :\t[LIN]\t[MIC]\t[HP]\t[LOUT]\n");
			uint8_t val = 0;
			val = BSP_ReadJackConnected(JACK_LINE_IN);
			printf("       :\t%d \t", val);
			val = BSP_ReadJackConnected(JACK_MIC);
			printf("%d \t", val);
			val = BSP_ReadJackConnected(JACK_HEADPHONE);
			printf("%d \t", val);
			val = BSP_ReadJackConnected(JACK_LINE_OUT);
			printf("%d \n", val);
			
			float volt = BSP_ReadBatteryVoltage(10);
			printf("[VBAT]: %.3f V", volt);
			sprintf(lcd_buf, "%.2f V  ", volt);
			ssd1306_SetCursor(&holedL, 50, 0);
			ssd1306_WriteString(&holedL, lcd_buf, Font_7x10, White);
			ssd1306_UpdateScreen(&holedL);
		}
		
		HAL_Delay(10);
		state_now = state_next;
		update_counter ++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_EXT;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */
	// @TODO change 16BIT extended
  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_SLAVE_TX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_EXTERNAL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SET_LIN_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SET_I_LIM_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SET_BOOT0_GPIO_Port, SET_BOOT0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SET_LIN_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SET_LIN_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DTC_LIN_Pin DTC_MIC_Pin DTC_HP_Pin DTC_LOUT_Pin */
  GPIO_InitStruct.Pin = DTC_LIN_Pin|DTC_MIC_Pin|DTC_HP_Pin|DTC_LOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_ENC2_Pin */
  GPIO_InitStruct.Pin = BTN_ENC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_ENC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SET_I_LIM_Pin LED2_Pin */
  GPIO_InitStruct.Pin = SET_I_LIM_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_ENC1_Pin */
  GPIO_InitStruct.Pin = BTN_ENC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_ENC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SET_BOOT0_Pin */
  GPIO_InitStruct.Pin = SET_BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SET_BOOT0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
