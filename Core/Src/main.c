/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CC1201_PA_CFG1 0x2B
#define CC1201_PA_CFG0	0x2C
#define CC1201_FS_DSM1 0x2F1A
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
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
	uint16_t  regAddr;
	uint8_t   value;
}registerSetting_t;



static const registerSetting_t preferredSettings[]=
{
  {CC1201_IOCFG2,            0x06},
  {CC1201_SYNC1,             0x6F},
  {CC1201_SYNC0,             0x4E},
  {CC1201_SYNC_CFG1,         0xE8},
  {CC1201_SYNC_CFG0,         0x13},
  {CC1201_DEVIATION_M,       0xCC},
  {CC1201_MODCFG_DEV_E,      0x08},
  {CC1201_DCFILT_CFG,        0x26},
  {CC1201_PREAMBLE_CFG0,     0x8A},
  {CC1201_IQIC,              0x00},
  {CC1201_CHAN_BW,           0x02},
  {CC1201_MDMCFG1,           0x42},
  {CC1201_MDMCFG0,           0x05},
  {CC1201_SYMBOL_RATE2,      0xA4},
  {CC1201_SYMBOL_RATE1,      0x7A},
  {CC1201_SYMBOL_RATE0,      0xE1},
  {CC1201_AGC_REF,           0x2F},
  {CC1201_AGC_CS_THR,        0xEC},
  {CC1201_AGC_CFG1,          0x16},
  {CC1201_AGC_CFG0,          0x84},
  {CC1201_FIFO_CFG,          0x00},
  {CC1201_FS_CFG,            0x1A},
  {CC1201_PKT_CFG2,          0x20},
  {CC1201_PKT_CFG0,          0x20},
  {CC1201_PA_CFG1,           0x3F},
  {CC1201_PKT_LEN,           0xFF},
  {CC1201_IF_MIX_CFG,        0x18},
  {CC1201_TOC_CFG,           0x03},
  {CC1201_MDMCFG2,           0x00},
  {CC1201_FREQ2,             0x57},
  {CC1201_FREQ1,             0x59},
  {CC1201_FREQ0,             0x99},
  {CC1201_IF_ADC1,           0xEE},
  {CC1201_IF_ADC0,           0x10},
  {CC1201_FS_DIG1,           0x07},
  {CC1201_FS_DIG0,           0x50},
  {CC1201_FS_CAL1,           0x40},
  {CC1201_FS_CAL0,           0x0E},
  {CC1201_FS_DIVTWO,         0x03},
  {CC1201_FS_DSM0,           0x33},
  {CC1201_FS_DVC0,           0x17},
  {CC1201_FS_PFD,            0x00},
  {CC1201_FS_PRE,            0x6E},
  {CC1201_FS_REG_DIV_CML,    0x1C},
  {CC1201_FS_SPARE,          0xAC},
  {CC1201_FS_VCO0,           0xB5},
  {CC1201_IFAMP,             0x0D},
  {CC1201_XOSC5,             0x0E},
  {CC1201_XOSC1,             0x03},
};


void halRfWriteReg(uint16_t regAddr, uint8_t value) {
    CC1200_CS_LOW(); // Активируем чип, устанавливая CS в низкое состояние

    uint8_t tempExt = (uint8_t)(regAddr >> 8);
    uint8_t tempAddr = (uint8_t)(regAddr & 0x00FF);
    uint8_t  statusByte;

    if (tempExt)
    	{
    		uint8_t command = CC1200_WRITE | CC1200_EXT_ADDR;
    		HAL_SPI_TransmitReceive(&hspi2, &command, &statusByte, 1, HAL_MAX_DELAY);
    		HAL_SPI_Transmit(&hspi2, &tempAddr, 1, HAL_MAX_DELAY);
    	}
    	else
    	{
    		tempAddr = CC1200_WRITE | tempAddr;
    		HAL_SPI_TransmitReceive(&hspi2, &tempAddr, &statusByte, 1, HAL_MAX_DELAY);
    	}


    HAL_SPI_Transmit(&hspi2, &value, 1, HAL_MAX_DELAY);

    CC1200_CS_HIGH();
}


void  CC1200_init(){
	 for (int i = 0; i < sizeof(preferredSettings) / sizeof(registerSetting_t); ++i) {
	        halRfWriteReg(preferredSettings[i].regAddr, preferredSettings[i].value);
	    }
}

void halRfWriteFifo(const uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        halRfWriteReg(CC120X_TXFIFO, 1);
    }
}

void transmitMessage(const uint8_t* message, uint8_t length) {
    // Переход в режим IDLE перед настройкой для отправки
    halRfWriteReg(CC120X_SIDLE, CC120X_WRITE_SINGLE_BYTE);
    //while (halRfReadReg(CC120X_MARCSTATE) != MARCSTATE_IDLE);

    // Очистка TX FIFO перед загрузкой данных
    halRfWriteReg(CC120X_SFTX, CC120X_WRITE_SINGLE_BYTE);

    // Запись сообщения в TX FIFO
    halRfWriteFifo(message, length);

    // Начало передачи
    halRfWriteReg(CC120X_STX, CC120X_WRITE_SINGLE_BYTE);

    // Ожидание завершения передачи может потребовать дополнительной логики,
    // в зависимости от настройки прерываний или проверки статусных регистров.
}


uint8_t readRegisterEx(const uint8_t regAddr)
{
	 	uint8_t readCommand = 0x80 | CC1200_EXT_ADDR;
	    uint8_t statusByte = 0;
	    uint8_t regValue = 0;

	    CC1200_CS_LOW();


	    HAL_SPI_TransmitReceive(&hspi2, &readCommand, &statusByte, 1, HAL_MAX_DELAY);
	    HAL_SPI_TransmitReceive(&hspi2, &regAddr, &regValue, 1, HAL_MAX_DELAY);
	    HAL_SPI_Receive(&hspi2, &regValue, 1, HAL_MAX_DELAY);

	    CC1200_CS_HIGH();


	    return regValue;
}
uint8_t readRegister(const uint8_t regAddr)
{
	 uint8_t readCommand = regAddr | 0x80 |0x00;
	    uint8_t statusByte = 0;
	    uint8_t regValue = 0;

	    CC1200_CS_LOW();


	    HAL_SPI_TransmitReceive(&hspi2, &readCommand, &statusByte, 1, HAL_MAX_DELAY);
	    //HAL_SPI_TransmitReceive(&hspi2, (uint8_t[]){0xFF}, &regValue, 1, HAL_MAX_DELAY);
	    HAL_SPI_Receive(&hspi2, &regValue, 1, HAL_MAX_DELAY);

	    CC1200_CS_HIGH();


	    return regValue;
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  CC1200_CS_HIGH();
  HAL_GPIO_WritePin(RES_CC_GPIO_Port, RES_CC_Pin, 0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RES_CC_GPIO_Port, RES_CC_Pin, 1);


  halRfWriteReg(0x1B,0x22);
  uint8_t partNum = readRegister(0x1B);
  printf("Part Number: %d\n", partNum);

  uint8_t version = readRegister(VERSION_REG);
  printf("Version: %d\n", version);


  CC1200_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t message[] = "Hello, RF World!";
  transmitMessage(message, sizeof(message) - 1);
  while (1)
  {


	  CC1200_CS_LOW();
	  uint8_t temp=0x3B;
	  HAL_SPI_Transmit(&hspi2, &temp, 1, HAL_MAX_DELAY);
	  CC1200_CS_HIGH();
	  /*
	  CC1200_CS_LOW();
	  temp= CC1200_ENQUEUE_TX_FIFO | CC1200_BURST;
	  uint8_t status;
	  HAL_SPI_Transmit(&hspi2, &temp, 1, HAL_MAX_DELAY);
	  temp=0x00;
	  	for (int i = 0; i < 100; i++)
	  	{

	  		HAL_SPI_TransmitReceive(&hspi2, &temp, &status, 1, HAL_MAX_DELAY);
	  	}
	  	CC1200_CS_HIGH();*/
	  	CC1200_CS_LOW();
	  	temp=0x35;
	  	HAL_SPI_Transmit(&hspi2, &temp, 1, HAL_MAX_DELAY);
	  	CC1200_CS_HIGH();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RES_CC_GPIO_Port, RES_CC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IO_GPIO_Port, IO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NCS2_GPIO_Port, NCS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NCS_Pin */
  GPIO_InitStruct.Pin = NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RES_CC_Pin */
  GPIO_InitStruct.Pin = RES_CC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RES_CC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_Pin */
  GPIO_InitStruct.Pin = IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NCS2_Pin */
  GPIO_InitStruct.Pin = NCS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NCS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
