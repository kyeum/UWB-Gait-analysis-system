/* USER CODE BEGIN Header */

/*Definition in INSOLE Device Comm system
INSOLE_L : 10BYTE(FSR) + 4BYTE(ST/ED) + 6BYTE(UWB) + 2BYTE(CRC) = 22BYTE
***INSOLE_L : 10BYTE(FSR) + 4BYTE(ST/ED) + 12BYTE(UWB) + 2BYTE(CRC) = 28BYTE

INSOLE_R : 20BYTE(FSR) + 4BYTE(ST/ED) + 12BYTE(UWB) + 2BYTE(CRC)= 38BYTE
***INSOLE_R : 20BYTE(FSR) + 4BYTE(ST/ED) + 24BYTE(UWB) + 2BYTE(CRC)= 50BYTE

IMU      : 2Byte(TMR) + 20BYTE(FSR) + 18BYTE(IMU) + 12BYTE(UWB) + 4BYTE(ST/ED) + 2BYTE(CRC) = 58BYTE
***IMU      : 2Byte(TMR) + 20BYTE(FSR) + 18BYTE(IMU) + 24BYTE(UWB) + 4BYTE(ST/ED) + 2BYTE(CRC) = 70BYTE
*/
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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
CRC_HandleTypeDef hcrc;
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	//Communication buffers
	//DMA comm
	bool dma_connect = false;
	bool data_received_dma = true;
	
	uint8_t txdata[50]; //rx byffer
	uint8_t rxBuf_[32] = {0};
	uint8_t rxBuf_crc[32] = {0};

	//Inturrupt comm
	uint8_t buff_index = 0;
	bool received_flag = false;
	bool received_end = false;
	
	uint8_t rxBuf_sd[9];

	//UWB comm
	uint8_t Rxbuf_ino[16] = {0}; //rx from arduino
	uint8_t Rxbuf_ino_p[16] = {0}; //rx from arduino parsed

	//ADC DMA
	uint16_t adcValArray[5] = {0,};

	//crc
	uint32_t crcArray[8] = {0,}; // crc for save
	uint32_t crcArray_send[12] = {0,}; // crc for send
	uint16_t crcval = 0; // calculate crc data for test
	
	//buff calcualtion
	uint16_t cal_val = 0;
	uint16_t receive_val =0;

	//SD card variables 
	extern char SDPath[4];   /* SD logical drive path */
	extern FATFS SDFatFS;    /* File system object for SD logical drive */
	extern FIL SDFile;       /* File object for SD */

	//FILE I/O operation
	FRESULT res;                                          /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "test"; 			      /* File write start buffer */
	uint8_t rtext[100];                                   /* File read buffer */
	
	FATFS myFATAS;
	FIL myFILE;
	UINT testByte; 										  // error detection 
	bool datasave_flg = true;
	bool ongoing_flg = false;
	
	
	bool sdcard_save = false;
	bool sdcard_save_check = true;
	bool sdcard_save_ongoing = false;
	
	//Timer
	bool _10ms_flg = false;
	bool _5ms_flg = false;
	bool _1000ms_flg = false;

	uint16_t ms_tmr = 0;
	uint16_t _100ms_tmr = 0;
	uint16_t ms_sv_tmr = 0;
	
	//wdg cnt
	uint8_t wdg_cnt = 0;
	uint8_t wdg_tmr = 0;
	
	//test val
	uint16_t test_val = 0;
	uint8_t k = 0;
	char str_testset[10];

	
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_SDIO_SD_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxBuf_crc, 32); //interrupt dma mode only in the mcu data
  //HAL_UART_Receive_DMA(&huart6, (uint8_t *)Rxbuf_ino, 16); //interrupt dma mode for arduino
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuf_sd, 9); //receive data from sd card


  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcValArray, 5);
  HAL_TIM_Base_Start_IT(&htim4);
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  /* NOTE: This function should not be modified, when the callback is needed, 
           the HAL_UART_RxCpltCallback could be implemented in the user file
  */
		//from 
		if(huart->Instance == USART2){
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
				if(rxBuf_crc[0] == 0xFF && rxBuf_crc[1] == 0xFF){
					received_flag = true;
					dma_connect = true;
					memcpy(&rxBuf_[0], &rxBuf_crc[0], 32 );
				}
				else
				{
					received_flag = false;
					dma_connect = false;
					HAL_DMA_Abort(&hdma_usart2_rx);	
				}
				test_val++;
		}
//		//from pc
//		if(huart->Instance == USART1){

//		uint8_t rxBuf_sd_p[9];

//		char rxlen = sizeof(rxBuf_sd);
//		for(char i =0; i < rxlen; i++)
//		{
//			char stx = i;
//			char etx = (i + rxlen - 1)%rxlen;
//			if(rxBuf_sd[stx] == '*' && rxBuf_sd[(etx)] == ';')
//			{
//					for(char j =0; j < rxlen; j++){
//						char k = (stx + j)%rxlen;
//						rxBuf_sd_p[j] = rxBuf_sd[k]; // data parse AND SAVE CUR
//					}
//				break;
//			}
//		}

//				if(rxBuf_sd_p[0] == '*' && rxBuf_sd_p[8] ==';'){
//						if(rxBuf_sd_p[1] == 's'){
//							char _txt[] = ".txt";
//							strncpy(str_testset, (const char*)rxBuf_sd+2,6);
//							strcat(str_testset,_txt);			
//						//	sdcard_save = true;
//						HAL_UART_Transmit_IT(&huart2,rxBuf_sd_p,9); // data send to other bluetooth!	
//						} 
//						else if(rxBuf_sd_p[1] == 'e'){		
//							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//						//	sdcard_save = false;
//						}
//				}
//				else{
//					memset(rxBuf_sd,0,sizeof(uint8_t)*9);
//				}
//	HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuf_sd, 9);	
//			
//		}
			// from arduino
		if(huart->Instance == USART6){

			}	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	if(huart->Instance == USART2){  

	}
	/* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
	*/
}

void wdg_activate(){
	HAL_NVIC_SystemReset();
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
	//sd
	f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);

//	if(f_open(&SDFile, "test32.txt", FA_CREATE_ALWAYS | FA_WRITE )== FR_OK){
//	}
//	

  /* Infinite loop */
  for(;;)
  {
//	  	if(sdcard_save){
//				if(sdcard_save_check){
//					//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//					sdcard_save_check = false;	
//					if(f_open(&SDFile, str_testset , FA_CREATE_ALWAYS | FA_WRITE ) == FR_OK){
//					sdcard_save_ongoing = true;
//					//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//					}
//				}
//			}
//		else{
//			if(!sdcard_save_check) {
//				f_close(&SDFile);
//				sdcard_save_check = true;
//				sdcard_save_ongoing = false;
//			}
//		}
	  if(_5ms_flg){
		_5ms_flg = false;
	  
		txdata[0] = 0xFF;
		txdata[1] = 0xFF;
		//Right leg insole
		for(int i =0; i<5; i++){ // right / left -- insole + Imu dataset :
		char a = i*2;
		txdata[2+a] = adcValArray[i] >> 8;  //hi
		txdata[3+a] = adcValArray[i] & 0xFF;//lo
		}
		//Left leg insole from usart

		if(received_flag) { 
		received_flag = false;
		uint8_t cprxBuf[32];
		memcpy(&cprxBuf[0], &rxBuf_[0], sizeof(uint8_t)*32); 

		memset(crcArray,0,8*sizeof(crcArray[0]));
		memmove(&crcArray[0], &cprxBuf[2], sizeof(uint8_t)*22); 
			char crc_begin = 24;
			char crc_end = 25;   //LLSB	 20 = FF 21 = FE 28 - 27 26
			receive_val = (int16_t)(((int16_t)cprxBuf[crc_begin] << 8) | cprxBuf[crc_end]);
			cal_val = (HAL_CRC_Calculate(&hcrc,crcArray,7)&0xffff);
			if(cal_val == receive_val)
			{
				memmove(&txdata[12], &cprxBuf[2], sizeof(uint8_t)*26);
			}			
			else if( cal_val != receive_val)
			{
 				wdg_cnt ++;
				continue;
			}
		}
		//right foot
		//txdata [34] = 
		
//		for(int i =0; i < 12; i ++){
//		txdata [34+i]  = Rxbuf_ino_p[2+i]; // signed short uwb data to txdata
//		}

		memset(crcArray_send,0,12*sizeof(crcArray_send[0]));
		memcpy(&crcArray_send[0], &txdata[2] , 44); 
		crcval = HAL_CRC_Calculate(&hcrc,crcArray_send,11)& 0xffff;
	
		txdata[46] = crcval >> 8; 
		txdata[47] = crcval & 0xff;
		txdata[48] = 0xFF;
		txdata[49] = 0xFE;

		//	DATA SAVE TO LEFT LEG
			//5ms data save
//			for(int i =0; i<26; i++) {
//			f_printf(&SDFile, "%02x", txdata[i]);
//			}
//			f_printf(&SDFile, "\n");
//	
//			f_close(&SDFile);

			// received data from pc
			
			// crc error
			HAL_UART_Transmit_IT(&huart1,txdata,50); //			
		}
//	TRANSMIT DATA TO LEFT LEG	
	}
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
 	if(htim ->Instance  == TIM4){
		ms_tmr ++;
		_100ms_tmr++;
		ms_sv_tmr++;
		
 		if(ms_tmr % 5 == 0){
			_5ms_flg = true;
			ms_tmr = 0;
		}
		if(_100ms_tmr % 100 == 0){ // watchdog tmr
			ongoing_flg = true;
			_100ms_tmr = 0;
		}
		
		if(ms_sv_tmr % 100 == 0){ // 10sec later - data save -> will change mode in the pc data
			datasave_flg = false;		
			if(!dma_connect) HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxBuf_crc, 32); //interrupt dma mode only in the mcu data

			ms_sv_tmr = 0;
		}
	}
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/