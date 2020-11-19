/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

#define Gain				1
#define Trigger_Level_Set	2
#define Start_Stream		3
#define Start_Trace 		4
#define Start_Trigger 		5
#define Stop 				6
#define Request				7
#define Acknowledge			8

char RX_Str[64];

volatile uint8_t Mode = 0;
volatile uint16_t Value = 0;

volatile uint16_t DMA_Triggered_Part_Flag = 0;
volatile uint16_t DMA_Triggered_I_Part_Cplt = 0;

volatile uint16_t DMA_Stream_Part_Flag = 0;
volatile uint16_t DMA_Tracer_Part_Flag = 0;

volatile uint16_t Tracer_Lock = 0;

volatile uint16_t Trace_Triggered = 0;

volatile uint16_t Trigger_Cplt = 0;
volatile uint16_t Tracer_Cplt = 0;
volatile uint16_t Stream_Cplt = 0;

#define ADC_DMA_Len 100
#define ADC_DMA_Half_Len (ADC_DMA_Len / 2)
uint16_t ADC_DMA_Buf[ADC_DMA_Len];



#define ADC_CDC_Transmit_Len  5000
uint16_t ADC_CDC_Transmit_Buf[ADC_CDC_Transmit_Len];


uint16_t Trigger_Level = 1000;

uint16_t CDC_RX_Len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Ok_Mess()
{
	char mess[] = "Done!\n";
	CDC_Transmit_FS(&mess, strlen(mess));
}

void Start_Symbol()
{
	uint16_t mess = 0xFFFF;
	while(CDC_Transmit_FS(&mess, sizeof(mess)) == USBD_BUSY);
}

void Stop_Symbol()
{
	uint16_t mess = 0xFFFE;
	while(CDC_Transmit_FS(&mess, sizeof(mess)) == USBD_BUSY);
}

void ADC_Trigger_Mod()
{
	DMA_Triggered_Part_Flag = 1;
	//DMA_Triggered_I_Part_Cplt = 0;
		if((HAL_ADC_GetValue(&hadc1)>Trigger_Level)&&(DMA_Triggered_I_Part_Cplt == 1))
		{
			DMA_Triggered_Part_Flag = 2;
			while(DMA_Triggered_Part_Flag != 0){}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(1);
			Start_Symbol();
			while(CDC_Transmit_FS((uint8_t*)&ADC_CDC_Transmit_Buf, sizeof(uint16_t)*ADC_DMA_Half_Len*3) == USBD_BUSY);
			//DMA_Triggered_Part_Flag = 0;
			DMA_Triggered_I_Part_Cplt = 0;
			Stop_Symbol();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		}
}

void ADC_Stream_Mod()
{
	DMA_Stream_Part_Flag = 1;
	while(DMA_Stream_Part_Flag != 0){}
	Start_Symbol();
	while(CDC_Transmit_FS((uint8_t*)&ADC_CDC_Transmit_Buf, sizeof(uint16_t)*ADC_CDC_Transmit_Len) == USBD_BUSY);
	Stop_Symbol();
	DMA_Stream_Part_Flag = 0;
}

void ADC_Trace_Mod()
{
	if(DMA_Tracer_Part_Flag == 1)
	{
		Tracer_Lock = 1;
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_Buf, sizeof(uint16_t)*ADC_DMA_Half_Len);

		//Trace_Triggered = 1;
		while(DMA_Tracer_Part_Flag != 0){}
		Start_Symbol();
		while(CDC_Transmit_FS((uint8_t*)&ADC_CDC_Transmit_Buf, sizeof(uint16_t)*ADC_CDC_Transmit_Len) == USBD_BUSY);
		Stop_Symbol();

		Tracer_Lock = 0;


		//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_Buf, sizeof(uint16_t)*ADC_DMA_Half_Len);
	}

}

void Send_data_mode() {
	uint16_t mess = 0xFFFF;
	while(CDC_Transmit_FS(&mess, sizeof(mess)) == USBD_BUSY);
	while(CDC_Transmit_FS((uint8_t*)&ADC_CDC_Transmit_Buf, sizeof(uint16_t)*ADC_CDC_Transmit_Len) == USBD_BUSY);
	mess = 0xFFFE;
	while(CDC_Transmit_FS(&mess, sizeof(mess)) == USBD_BUSY);
}

void DAC_Write(uint16_t val)
{
  uint16_t dac_data = 0x7000 | (val&0x0FFF);
  uint8_t data[2];
  memcpy(&data, &dac_data, sizeof(data));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, data, sizeof(data), 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}



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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  RX_Str[0] = 3;

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_Buf, sizeof(uint16_t)*ADC_DMA_Half_Len);
  //DAC_Write(1800);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch(Mode)
	{
		case Gain:
			DAC_Write(Value);
			break;

		case Trigger_Level_Set:
			Trigger_Level = Value;
			break;

		case Start_Stream:
			ADC_Stream_Mod();
			break;

		case Start_Trace:
			ADC_Trace_Mod();
			break;

		case Start_Trigger:
			ADC_Trigger_Mod();
			break;

		case Stop:
			break;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void CDC_ReciveCallBack(uint8_t *Buf, uint32_t *Len)
{
	//CDC_Transmit_FS(Buf, Len);// debug
	for(int i = 0; i < 64; i++)
		RX_Str[i] = 0;

	memcpy(RX_Str, Buf, Len);
	CDC_RX_Len = Len;
	Mode = RX_Str[0];

	Value = atoi(&RX_Str[1]);

	//DAC_Write(atoi(RX_Str));
	//RX_Str[CDC_RX_Len] = '\n';
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == GPIO_PIN_1)&&(Tracer_Lock == 0))
	{
		DMA_Tracer_Part_Flag = 1;
	}
	else
	{
		__NOP();
	}
}


 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	 if(DMA_Triggered_Part_Flag == 1)
	 {
		 memcpy(&ADC_CDC_Transmit_Buf[0], &ADC_DMA_Buf[ADC_DMA_Half_Len], sizeof(uint16_t)*ADC_DMA_Half_Len);
		 DMA_Triggered_I_Part_Cplt = 1;
	 }

	if(DMA_Triggered_Part_Flag > 1)
	{
		memcpy(&ADC_CDC_Transmit_Buf[ADC_DMA_Half_Len*(DMA_Triggered_Part_Flag-1)], &ADC_DMA_Buf[ADC_DMA_Half_Len], sizeof(uint16_t)*ADC_DMA_Half_Len);
		if((DMA_Triggered_Part_Flag < 3))
		{DMA_Triggered_Part_Flag++;}
		else
		{DMA_Triggered_Part_Flag = 0;}
	}

	if(DMA_Stream_Part_Flag > 0)
	{
		memcpy(&ADC_CDC_Transmit_Buf[ADC_DMA_Half_Len*(DMA_Stream_Part_Flag-1)], &ADC_DMA_Buf[ADC_DMA_Half_Len], sizeof(uint16_t)*ADC_DMA_Half_Len);
		if(DMA_Stream_Part_Flag < (int)(ADC_CDC_Transmit_Len/ADC_DMA_Half_Len))
		{DMA_Stream_Part_Flag++;}
		else
		{DMA_Stream_Part_Flag = 0;}
	}

	if(DMA_Tracer_Part_Flag > 0)
		{
			memcpy(&ADC_CDC_Transmit_Buf[ADC_DMA_Half_Len*(DMA_Tracer_Part_Flag-1)], &ADC_DMA_Buf[ADC_DMA_Half_Len], sizeof(uint16_t)*ADC_DMA_Half_Len);
			if(DMA_Tracer_Part_Flag < (int)(ADC_CDC_Transmit_Len/ADC_DMA_Half_Len))
			{DMA_Tracer_Part_Flag++;}
			else
			{DMA_Tracer_Part_Flag = 0;}
		}


}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(DMA_Triggered_Part_Flag == 1)
	{
		memcpy(&ADC_CDC_Transmit_Buf[0], &ADC_DMA_Buf[0], sizeof(uint16_t)*ADC_DMA_Half_Len);
		DMA_Triggered_I_Part_Cplt = 1;
	}

	if(DMA_Triggered_Part_Flag > 1)
	{
		memcpy(&ADC_CDC_Transmit_Buf[ADC_DMA_Half_Len*(DMA_Triggered_Part_Flag-1)], &ADC_DMA_Buf[0], sizeof(uint16_t)*ADC_DMA_Half_Len);
		if((DMA_Triggered_Part_Flag < 3))
		{DMA_Triggered_Part_Flag++;}
		else
		{DMA_Triggered_Part_Flag = 0;}
	}

	if(DMA_Stream_Part_Flag > 0)
	{
		memcpy(&ADC_CDC_Transmit_Buf[ADC_DMA_Half_Len*(DMA_Stream_Part_Flag-1)], &ADC_DMA_Buf[0], sizeof(uint16_t)*ADC_DMA_Half_Len);
		if(DMA_Stream_Part_Flag < (int)(ADC_CDC_Transmit_Len/ADC_DMA_Half_Len))
		{DMA_Stream_Part_Flag++;}
		else
		{DMA_Stream_Part_Flag = 0;}
	}

	if(DMA_Tracer_Part_Flag > 0)
	{
		memcpy(&ADC_CDC_Transmit_Buf[ADC_DMA_Half_Len*(DMA_Tracer_Part_Flag-1)], &ADC_DMA_Buf[0], sizeof(uint16_t)*ADC_DMA_Half_Len);
		if(DMA_Tracer_Part_Flag < (int)(ADC_CDC_Transmit_Len/ADC_DMA_Half_Len))
		{DMA_Tracer_Part_Flag++;}
		else
		{DMA_Tracer_Part_Flag = 0;}
	}
}

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
