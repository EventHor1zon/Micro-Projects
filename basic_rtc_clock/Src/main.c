/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SECONDS_REG 0x0
#define MINS_REG	0x1
#define HOURS_REG	0x2
#define RTC_ADDRESS 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

osThreadId defaultTaskHandle;
osThreadId write_SRHandle;
osThreadId readRTCHandle;
/* USER CODE BEGIN PV */

uint8_t active_display;  // active display counter
uint8_t TIME_READ=0;	 // see if clock has been read yet

typedef struct {
	// private //
	uint8_t address;
	uint8_t twentyfour_hour;
	// public //
	// individual digits of current time for display //
	uint8_t time_hour_large;
	uint8_t time_hour_small;
	uint8_t	time_min_large;
	uint8_t time_min_small;
	uint8_t time_sec_large;
	uint8_t time_sec_small;
} clockinfo;

clockinfo clock;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RTC_Config(clockinfo *clock, I2C_HandleTypeDef *Handle);
//uint8_t Read_Time(clockinfo *clock, I2C_HandleTypeDef *Handle);
//uint8_t Read_Hours(clockinfo *clock, I2C_HandleTypeDef *Handle);
//uint8_t Write_to_Reg(uint8_t device, uint8_t reg, uint8_t data, I2C_HandleTypeDef *Handle);
//uint8_t Read_from_Register(uint8_t device, uint8_t reg, uint8_t *buffer, uint8_t len, I2C_HandleTypeDef *Handle);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// set the clock address
  clock.address = (uint8_t) RTC_ADDRESS;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  RTC_Config(&clock, &hi2c1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of write_SR */
  osThreadDef(display_next_digit, StartTask03, osPriorityAboveNormal, 0, 128);
  write_SRHandle = osThreadCreate(osThread(display_next_digit), NULL);

  /* definition and creation of readRTC */
  osThreadDef(readRTC, StartTask04, osPriorityNormal, 0, 128);
  readRTCHandle = osThreadCreate(osThread(readRTC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{


  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|LD4_Pin 
                          |LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : alarm_in_Pin */
  GPIO_InitStruct.Pin = alarm_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(alarm_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC6 PC7 LD4_Pin 
                           LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|LD4_Pin 
                          |LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

uint8_t RTC_Config(clockinfo *clock, I2C_HandleTypeDef *Handle){

	I2C_HandleTypeDef *hi2c = Handle;
	// set the 24 hour mode

	uint8_t time_buffer[3];
	uint8_t hours, mins, secs, temp;
	// first need to write bit 6 of the hours register to 0
	// so read from the register, then write it back in?
	//if(Read_from_Register((uint8_t) clock->address, (uint8_t)HOURS_REG, &temp, 1, hi2c)) { return HAL_ERROR; }
	HAL_I2C_Master_Transmit(hi2c, (uint16_t)clock->address, (uint8_t *)HOURS_REG, 1, 50);
	HAL_I2C_Master_Receive(hi2c, (uint16_t)clock->address, &temp, 1, 50);
	// NAND temp register to clear bit 6
	temp &= ~(1 << 6);   // temp & ~(0100 0000) = temp & (1011 1111)
	// write the value back in - this should set the 24 hour mode on
	// write_to_Reg((uint8_t) clock->address, (uint8_t)HOURS_REG, temp, hi2c);
	HAL_I2C_Mem_Write (hi2c,				//i2c handle
							 (uint16_t)clock->address,		// device address
							 (uint16_t)HOURS_REG,			// register address
							 1,								// size of address
							 &temp,							// data to write
							 1,								// len of data
							 50);
	/*
	uint8_t zero = 0b000000000000000000000000;
	HAL_I2C_Mem_Write(hi2c,
					(uint16_t)clock->address,
					(uint16_t)SECONDS_REG,
					1,
					3,
					100);*/
	// read current time from the RTC
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)clock->address, (uint8_t)SECONDS_REG, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)clock->address, (uint8_t *)time_buffer, 3, 50);

	 // sort the data into the clock struct
	hours=time_buffer[2];// & ~(0b11000000);
	clock->time_hour_small =(uint8_t) hours & ~(0b11110000);
	if(hours & (0b00010000)) { clock->time_hour_large = 1; }
	else if(hours & (0b00100000)) { clock->time_hour_large = 2; }
	else { clock->time_hour_large = 0; }
	mins=time_buffer[1];
	clock->time_min_large = mins >>4;
	clock->time_min_small = mins & ~(0b11110000);
	secs=time_buffer[0];
	clock->time_sec_large = secs >>4;
	clock->time_sec_small = secs & ~(0b11110000);
		 // declare time read = 1
	if(!TIME_READ) { TIME_READ=1; }
	return 0;

}

/*
uint8_t Read_Time(clockinfo *clock, I2C_HandleTypeDef *Handle){

	I2C_HandleTypeDef *hi2c = Handle;
	uint8_t time_buffer[3];

	if(Read_from_Register((uint8_t) clock->address, (uint8_t)SECONDS_REG, (uint8_t *)time_buffer, 3, hi2c) != HAL_OK){ return 1; }

	uint8_t hours=time_buffer[2];// & ~(0b11000000);
	clock->time_hour_small =(uint8_t) hours & ~(0b11110000);
	if(hours & (0b00010000)) { clock->time_hour_large = 1; }
	else if(hours & (0b00100000)) { clock->time_hour_large = 2; }
	else { clock->time_hour_large = 0; }
	uint8_t mins=time_buffer[1];
	clock->time_min_large = mins >>4;
	clock->time_min_small = mins & ~(0b11110000);
	uint8_t secs=time_buffer[0];
	clock->time_sec_large = secs >>4;
	clock->time_sec_small = secs & ~(0b11110000);

	return 0;
}
*/



uint8_t Digit_to_Display(uint8_t digit){

		// if rtc not yet read, would output garbage. Skip it.
	if(!TIME_READ){ return 0; }

	switch(digit){
		// assuming SR wired with outputs QA-G to digit LEDs, QH = dot
		case 1:		return 0b00100100;
		case 2:		return 0b10111010;
		case 3:		return 0b10101110;
		case 4: 	return 0b01100110;
		case 5:		return 0b11001110;
		case 6:		return 0b11011110;
		case 7:		return 0b10100100;
		case 8:		return 0b11111110;
		case 9:		return 0b11101110;
		case 0:		return 0b11111100;
		default:	return 0;
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the write_SR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */

  if(active_display > 3) { active_display=0; }

  uint8_t data, bin_data;

  for(;;)
  {
	// select appropriate transistor & fill appropriate data
	if(active_display > 3) { active_display=0; }
	switch(active_display){
		case 0:
			data = clock.time_hour_large;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			break;
		case 1:
			data = clock.time_hour_small;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			break;
		case 2:
			data = clock.time_min_large;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		case 3:
			data = clock.time_min_small;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		default:		// something done gone wrong... switch 'em all off
			data=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
	}
	// get binary data to write to shift register
	bin_data=Digit_to_Display(data);
	// now write the shift register data out - set latch pin low
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &bin_data, 1, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

	// increment the active display
	active_display++;

    osDelay(2);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the readRTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */

	uint8_t time_buffer[3];
	uint8_t hours, mins, secs;
	for(;;)
	{

	 // read current time from the RTC
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)clock.address, (uint8_t)SECONDS_REG, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)clock.address, (uint8_t *)time_buffer, 3, 50);

	 // sort the data into the clock struct
	hours=time_buffer[2];// & ~(0b11000000);
	clock.time_hour_small =(uint8_t) hours & ~(0b11110000);
	if(hours & (0b00010000)) { clock.time_hour_large = 1; }
	else if(hours & (0b00100000)) { clock.time_hour_large = 2; }
	else { clock.time_hour_large = 0; }
	mins=time_buffer[1];
	clock.time_min_large = mins >>4;
	clock.time_min_small = mins & ~(0b11110000);
	secs=time_buffer[0];
	clock.time_sec_large = secs >>4;
	clock.time_sec_small = secs & ~(0b11110000);
	// declare time read = 1
	if(!TIME_READ) { TIME_READ=1; }
	 // chill
	osDelay(500);
  }
  /* USER CODE END StartTask04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
