/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "ADS1256.h"

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
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t _pga = 1;
float _VREF = 2.5;
float _conversionFactor = 1;
volatile int read_flag;
float reading;
//int ignore;

/* USER CODE BEGIN 0 */

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim6) < us);  // wait for the counter to reach the us input in the parameter
}


void ADS_CSON() {
//	  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET);
}

void ADS_CSOFF() {
//  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);
}

void ADS_waitDRDY() {
//	if(ignore == 0){
	  while (HAL_GPIO_ReadPin(ADS_DDRY_GPIO_Port, ADS_DDRY_Pin));
//	}
}

void ADS_sendCommand(uint8_t reg) {
	ADS_CSON();
	ADS_waitDRDY();

	HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
  delay_us(1);              //  t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  ADS_CSOFF();
}

uint8_t ADS_readRegister(uint8_t reg) {
  uint8_t readValue = 0;
  ADS_CSON();
  uint8_t buff[] = {ADS1256_CMD_RREG | reg, 0};
  HAL_SPI_Transmit(&hspi2, buff, 2, HAL_MAX_DELAY);
  delay_us(7);              //  t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  HAL_SPI_Receive(&hspi2, &readValue, 1, HAL_MAX_DELAY);
  delay_us(1);              //  t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  ADS_CSOFF();
  return readValue;
}

void ADS_writeRegister(uint8_t reg, uint8_t wdata) {
  ADS_CSON();
  uint8_t buff[3] = {ADS1256_CMD_WREG | reg, 0, wdata};
  HAL_SPI_Transmit(&hspi2, buff, 3, HAL_MAX_DELAY);
//  SPI.transfer(); // opcode1 Write registers starting from reg
//  SPI.transfer(0);  // opcode2 Write 1+0 registers
//  SPI.transfer(wdata);  // write wdata
  delay_us(1);
  ADS_CSOFF();
}

uint8_t ADS_getStatus() {
	ADS_sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  return ADS_readRegister(ADS1256_RADD_STATUS);
}

int ADS_begin(){
	ADS_sendCommand(ADS1256_CMD_SDATAC);
	uint8_t status = ADS_readRegister(ADS1256_RADD_STATUS);
	//musi byc 0x30
	ADS_sendCommand(ADS1256_CMD_SELFCAL);
	ADS_waitDRDY();
	return status;
}

int ADS_begin_drate(uint8_t drate, uint8_t gain, uint8_t buffenable) {
  _pga = 1 << gain;
  ADS_sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  ADS_writeRegister(ADS1256_RADD_DRATE, drate);  // write data rate register
  uint8_t bytemask = 0x07;
  uint8_t adcon = ADS_readRegister(ADS1256_RADD_ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  ADS_writeRegister(ADS1256_RADD_ADCON, byte2send);
  uint8_t status = ADS_readRegister(ADS1256_RADD_STATUS);
  if (buffenable) {
	status |= (1<<0);//bitSet(status, 1);
	ADS_writeRegister(ADS1256_RADD_STATUS, status);
  }
  ADS_sendCommand(ADS1256_CMD_SELFCAL);  // perform self calibration

  ADS_waitDRDY();  // wait ADS1256 to settle after self calibration
  return status;
}

void ADS_setChannel(uint8_t AIN_P, uint8_t AIN_N) {
  uint8_t MUX_CHANNEL;
  uint8_t MUXP;
  uint8_t MUXN;

  switch (AIN_P) {
    case 0:
      MUXP = ADS1256_MUXP_AIN0;
      break;
    case 1:
      MUXP = ADS1256_MUXP_AIN1;
      break;
    case 2:
      MUXP = ADS1256_MUXP_AIN2;
      break;
    case 3:
      MUXP = ADS1256_MUXP_AIN3;
      break;
    case 4:
      MUXP = ADS1256_MUXP_AIN4;
      break;
    case 5:
      MUXP = ADS1256_MUXP_AIN5;
      break;
    case 6:
      MUXP = ADS1256_MUXP_AIN6;
      break;
    case 7:
      MUXP = ADS1256_MUXP_AIN7;
      break;
    default:
      MUXP = ADS1256_MUXP_AINCOM;
  }

  switch (AIN_N) {
    case 0:
      MUXN = ADS1256_MUXN_AIN0;
      break;
    case 1:
      MUXN = ADS1256_MUXN_AIN1;
      break;
    case 2:
      MUXN = ADS1256_MUXN_AIN2;
      break;
    case 3:
      MUXN = ADS1256_MUXN_AIN3;
      break;
    case 4:
      MUXN = ADS1256_MUXN_AIN4;
      break;
    case 5:
      MUXN = ADS1256_MUXN_AIN5;
      break;
    case 6:
      MUXN = ADS1256_MUXN_AIN6;
      break;
    case 7:
      MUXN = ADS1256_MUXN_AIN7;
      break;
    default:
      MUXN = ADS1256_MUXN_AINCOM;
  }

  MUX_CHANNEL = MUXP | MUXN;

  ADS_CSON();
  ADS_writeRegister(ADS1256_RADD_MUX, MUX_CHANNEL);
  ADS_sendCommand(ADS1256_CMD_SYNC);
  ADS_sendCommand(ADS1256_CMD_WAKEUP);
  ADS_CSOFF();
}

void ADS_setPChannel(uint8_t channel) { ADS_setChannel(channel, -1); }



uint32_t ADS_read_uint24() {
	uint8_t inbuff[3] = {0,0,0};
  uint32_t value = 0;
  HAL_SPI_Receive(&hspi2, inbuff, 3, HAL_MAX_DELAY);
//  delay_us(10);
//  value = ((long)inbuff[0] << 16) + ((long)inbuff[1] << 8) + ((long)inbuff[2]);
  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Convert the signed 24bit stored in an unsigned 32bit to a signed 32bit
long ADS_read_int32() {
  long value = ADS_read_uint24();

  if (value & 0x00800000) { // if the 24 bit value is negative reflect it to 32bit
    value |= 0xff000000;
  }

  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Cast as a float
float ADS_read_float32() {
  long value = ADS_read_int32();
  return (float)value;
}


float ADS_readCurrentChannel() {
	ADS_CSON();
//	uint8_t buff = ADS1256_CMD_RDATA;
//	  HAL_SPI_Transmit(&hspi2, &buff, 1, HAL_MAX_DELAY);
//  delay_us(7);              //  t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  float adsCode = ADS_read_float32();
  ADS_CSOFF();
  return ((adsCode / (float)(0x7FFFFF)) * ((2.0 * _VREF) / (float)_pga)) *
         _conversionFactor;
}

uint8_t someinbuff[3] = {0,0,0};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADS_DDRY_Pin) {
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  int isSpiBusy = HAL_SPI_GetState(&hspi2);

		  if(isSpiBusy != HAL_SPI_STATE_BUSY) {
			  someinbuff[0]=0; //najstarszy
			  someinbuff[1]=0;
			  someinbuff[2]=0; //najmlodszy
			  HAL_SPI_TransmitReceive_DMA(&hspi2, someinbuff,someinbuff, 3);
		  }
  }
}

//void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
//void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
	}
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
  MX_DMA_Init();
  #define MX_DMA_Init() do {} while(0)
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	#undef MX_DMA_Init
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start(&htim6);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  HAL_Delay(1);
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  if(ADS_getStatus() == 0x30)
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  else
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

//  ADS_begin_drate(ADS1256_DRATE_500SPS, ADS1256_GAIN_1, 0);
//  ADS_begin_drate(ADS1256_DRATE_5SPS, ADS1256_GAIN_1, 0);
//  ADS_begin_drate(ADS1256_DRATE_30000SPS, ADS1256_GAIN_1, 0);
  ADS_begin_drate(ADS1256_DRATE_15000SPS, ADS1256_GAIN_1, 0);
  ADS_setPChannel(7);

  ADS_sendCommand(ADS1256_CMD_RDATAC);
  delay_us(7);
  HAL_SPI_Receive(&hspi2, someinbuff, 3, HAL_MAX_DELAY);

  while (1)
  {
//	  int isSpiBusy = HAL_SPI_GetState(&hspi2);
//
//	  if(isSpiBusy != HAL_SPI_STATE_BUSY){
//		  someinbuff[0]=0;
//		  someinbuff[1]=0;
//		  someinbuff[2]=0;
//		  HAL_SPI_TransmitReceive_DMA(&hspi2, someinbuff, someinbuff, 3);
//	  }

//	  currentDRDY = HAL_GPIO_ReadPin(ADS_DDRY_GPIO_Port, ADS_DDRY_Pin);
//
//	  int isSpiBusy = HAL_SPI_GetState(&hspi2);
//
//	  if(currentDRDY == 0 && lastDRDY == 1 && isSpiBusy != HAL_SPI_STATE_BUSY){
//		  someinbuff[0]=0;
//		  someinbuff[1]=0;
//		  someinbuff[2]=0;
//		  ADS_CSON();
//		  HAL_SPI_TransmitReceive_DMA(&hspi2, someinbuff, someinbuff, 3);
//	  }
//
//	  lastDRDY = currentDRDY;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : ADS_CS_Pin */
  GPIO_InitStruct.Pin = ADS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_RST_Pin */
  GPIO_InitStruct.Pin = ADS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADS_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_DDRY_Pin */
  GPIO_InitStruct.Pin = ADS_DDRY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADS_DDRY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
