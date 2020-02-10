/* USER CODE BEGIN Header */
/*Definition in INSOLE_R_KR
DATASET :
INSOLE_L : 10BYTE(FSR) + 4BYTE(ST/ED) + 2BYTE(CRC) = 16BYTE
INSOLE_R : 20BYTE(FSR) + 4BYTE(ST/ED) + 2BYTE(CRC)= 26BYTE
IMU 		 : 20BYTE(FSR) + 18BYTE(IMU) + 4BYTE(ST/ED) + 2BYTE(CRC) = 44BYTE

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
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

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
	//dma circular buffer
	DMA_Event_t dma_uart_rx = {0,0,DMA_BUF_SIZE};
	uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */
	uint8_t data[DMA_BUF_SIZE] = {'\0'};    /* Data buffer that contains newly received data */

	//Communication buffers
	uint8_t txdata[26]; // 4 from FFFF,FFFE 18 from Imu, 10 byte from insole 2byte crc : 34 byte 
	uint8_t rxBuf[16] = {0};
	
	
	
	uint8_t rxBuf2[24] = {0};

	uint8_t rxBuf_[24] = {0};
	uint8_t rxBuf_crc[24] = {0};
	uint8_t cp_rxBuf_[24] = {0,};	\

	bool useBuff = false;
	bool useBuff_mx = true;

	uint8_t buff_index = 0;
	uint8_t rxBuf_transmit[24] = {0};
	uint16_t adcValArray[5] = {0,};
	//crcval
	uint32_t crcArray[8] = {0,};
	uint32_t crcArray_send[9] = {0,};

	uint16_t crcval = 0;

	//test
	uint16_t cal_val = 0;
	uint16_t receive_val =0;
	
	//buff calcualtion
	//uint8_t rxBuf_crc[24] = {0};
	bool crc_test = true;
	bool crc_test2 = false;

	bool rxBuf_trans = true;
	
	//SD card variables 
	extern char SDPath[4];   /* SD logical drive path */
	extern FATFS SDFatFS;    /* File system object for SD logical drive */
	extern FIL SDFile;       /* File object for SD */

	//FILE I/O operation

	FRESULT res;                                          /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "Hello from kyeum!"; 			      /* File write start buffer */
	uint8_t rtext[100];                                   /* File read buffer */
	
	FATFS myFATAS;
	FIL myFILE;
	UINT testByte; 										  // error detection 
	bool datasave_flg = true;
	
	//Timer
	bool _10ms_flg = false;
	bool _100ms_flg = false;

	uint16_t ms_tmr = 0;
	uint16_t _100ms_tmr = 0;
	uint16_t ms_sv_tmr = 0;
	
	//wdg cnt
	uint8_t wdg_cnt = 0;
	
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
  /* USER CODE BEGIN 2 */

  /* Start DMA */
//    if(HAL_UART_Receive_DMA(&huart2, (uint8_t*)dma_rx_buf, DMA_BUF_SIZE) != HAL_OK)
//    {        
//        Error_Handler();
//    }
		
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcValArray, 5);
  HAL_TIM_Base_Start_IT(&htim4);
//  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  htim4.Init.Prescaler = 1000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
/** DMA Rx Complete AND DMA Rx Timeout function
 * Timeout event: generated after UART IDLE IT + DMA Timeout value
 * Scenarios:
 *  - Timeout event when previous event was DMA Rx Complete --> new data is from buffer beginning till (MAX-currentCNDTR)
 *  - Timeout event when previous event was Timeout event   --> buffer contains old data, new data is in the "middle": from (MAX-previousCNDTR) till (MAX-currentCNDTR)
 *  - DMA Rx Complete event when previous event was DMA Rx Complete --> entire buffer holds new data
 *  - DMA Rx Complete event when previous event was Timeout event   --> buffer entirely filled but contains old data, new data is from (MAX-previousCNDTR) till MAX
 * Remarks:
 *  - If there is no following data after DMA Rx Complete, the generated IDLE Timeout has to be ignored!
 *  - When buffer overflow occurs, the following has to be performed in order not to lose data:
 *      (1): DMA Rx Complete event occurs, process first part of new data till buffer MAX.
 *      (2): In this case, the currentCNDTR is already decreased because of overflow.
 *              However, previousCNDTR has to be set to MAX in order to signal for upcoming Timeout event that new data has to be processed from buffer beginning.
 *      (3): When many overflows occur, simply process DMA Rx Complete events (process entire DMA buffer) until Timeout event occurs.
 *      (4): When there is no more overflow, Timeout event occurs, process last part of data from buffer beginning till currentCNDTR.
*/


 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	//add ring buffer
	// data pacing and save buff to transmit array
	if(huart->Instance == USART2){
		/*
						for(int i = 0; i< 24; i++)
				{	
					rxBuf[i] = rxBuf_crc[i];
				}
				memcpy(rxBuf2, rxBuf_crc,sizeof(uint8_t)*24);		
				//memmove(rxBuf, rxBuf_crc,sizeof(uint8_t)*24);
				 // disconnect -> 
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
				char buff_maxsize = sizeof(rxBuf) / sizeof(uint8_t);
				bool data_received = false;
				for(char i = buff_index; i<buff_maxsize; i++){				
				
				char buff_end = (i+buff_maxsize-1)%buff_maxsize;
				char buff_index_2 = (i+1)%buff_maxsize;
				char buff_end_2 = (i+buff_maxsize-2)%buff_maxsize;
					
					if(rxBuf[i] == 0xFF && rxBuf[buff_index_2] == 0xFF&& rxBuf[buff_end_2] == 0xFF&& rxBuf[buff_end] == 0xFE){	
					buff_index = i; // head
					//rxBuf  transmit	
					useBuff = true;
					memset(rxBuf_,0,24*sizeof(rxBuf_[0]));	
					for(int k = 0; k<buff_maxsize; k++){
					char c = (i + k)%buff_maxsize;	
					rxBuf_[k] = rxBuf[c];
					}
					useBuff = false;
					data_received = true;	
					break;
					}
			}
			if(!data_received) buff_index = 0;
					memset(rxBuf_crc,0, 24*sizeof(rxBuf_crc[0]));		

		*/

	uint16_t i, pos, start, length;
	uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);
	
 /* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
    if(dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE)
    { 
        dma_uart_rx.flag = 0;
        return;
    }
 /* Determine start position in DMA buffer based on previous CNDTR value */
	start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;
	
	 if(dma_uart_rx.flag)    /* Timeout event */
    {
        /* Determine new data length based on previous DMA_CNDTR value:
         *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
         *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
        */
        length = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
        dma_uart_rx.prevCNDTR = currCNDTR;
        dma_uart_rx.flag = 0;
    }
    else                /* DMA Rx Complete event */
    {
        length = DMA_BUF_SIZE - start;
        dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
    }
	 /* Copy and Process new data */
//	useBuff = false;
    for(i=0,pos=start; i<length; ++i,++pos)
    {           
        rxBuf[i] = dma_rx_buf[pos];
    }
//	useBuff = true; 
		
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	if(huart->Instance == USART2){  
		
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14); // 200ms Orange BLINKING

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
             
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
	f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	
	char testname[20];
	char filetxt[5] = ".txt"; // directory
	f_mkdir(testname);
	//strcat(testnum,filetxt);
	
	if(f_open(&SDFile, "test.txt", FA_CREATE_ALWAYS | FA_WRITE )== FR_OK){
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
	}
	else{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
		osDelay(2000);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
		}
	
  /* Infinite loop */
  for(;;)
  {
	 // if(wdg_cnt > 30) wdg_activate();
		memset(txdata,0,26 * sizeof(txdata[0])); // txbuffer clear // local vari later!
		
		txdata[0] = 0xFF;
		txdata[1] = 0xFF;
		for(int i =0; i<5; i++){ // right / left -- insole + Imu dataset :
		char a = i*2;
		txdata[2+a] = adcValArray[i] >> 8;  //hi
		txdata[3+a] = adcValArray[i] & 0xFF;//lo
		}
		
		for(int i = 12; i< 21; i++){
			txdata[i] = i;		
		}
		
		/*		char buff_maxsize = sizeof(cp_rxBuf_) / sizeof(uint8_t);
				bool data_received = false;
				for(char i = 0; i<buff_maxsize; i++){				
				
				char buff_end = (i+buff_maxsize-1)%buff_maxsize;
				char buff_index_2 = (i+1)%buff_maxsize;
				char buff_end_2 = (i+buff_maxsize-2)%buff_maxsize;
					
					if(cp_rxBuf_[i] == 0xFF && cp_rxBuf_[buff_index_2] == 0xFF&& cp_rxBuf_[buff_end_2] == 0xFF&& cp_rxBuf_[buff_end] == 0xFE){	
					//buff_index = i; // head
					//rxBuf  transmit	
					memset(rxBuf_,0,24*sizeof(rxBuf_[0]));	
					for(int k = 0; k<buff_maxsize; k++){
					char c = (i + k)%buff_maxsize;	
					rxBuf_[k] = cp_rxBuf_[c];
					}
					break;
					}
				}
		memset(crcArray,0,8*sizeof(crcArray[0]));
		memcpy(&crcArray[0],&rxBuf_[2], sizeof(uint8_t)*18); 
		
				
		char crc_begin = 20; //MLSB
		char crc_end = 21;   //LLSB	
	  receive_val = (int16_t)(((int16_t)rxBuf_[crc_begin] << 8) | rxBuf_[crc_end]);
		cal_val = (HAL_CRC_Calculate(&hcrc,crcArray,5)&0xffff);
	    if(cal_val == receive_val)
		{
			memmove(&txdata[12], &rxBuf_[2], sizeof(uint8_t)*18); 
		}			
		else
		{
			wdg_cnt ++;
			continue;
		}
		*/
		
		memset(crcArray_send,0,8*sizeof(crcArray_send[0]));
		memcpy(&crcArray_send[0], &txdata[2] , 20); 
		crcval = HAL_CRC_Calculate(&hcrc,crcArray_send,5)& 0xffff;
		txdata[22] = crcval >> 8; 
		txdata[23] = crcval & 0xff;
		txdata[24] = 0xFF;
		txdata[25] = 0xFE;
		
//	DATA SAVE TO LEFT LEG	
/*
			if(datasave_flg){
			for(int i =0; i<24; i++) {
			f_printf(&SDFile, "%02x", cp_rxBuf_[i]);
			}
			f_printf(&SDFile, "\n");
			for(int i =0; i<24; i++) {
			f_printf(&SDFile, "%02x", rxBuf_[i]);
			}
			f_printf(&SDFile, "\n");
			
			for(int i =0; i<24; i++) {
			f_printf(&SDFile, "%02x", rxBuf[i]);
			}
			f_printf(&SDFile, "\n");
			}
			else
			{
			f_close(&SDFile);
			}
*/
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);

//	TRANSMIT DATA TO LEFT LEG
		if(_100ms_flg){
			
			_10ms_flg = false;
			HAL_UART_Transmit_IT(&huart1,txdata,26); // 4byte + 18 + 10 byte(insole right)+ CRC 2 : 34 byte				
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
		}
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

		if(ms_tmr % 10 == 0){
			//_100ms_flg = true;
			_10ms_flg = true;
			ms_tmr = 0;
		}
		if(_100ms_tmr % 100 == 0){
			_100ms_tmr = 0;
			_100ms_flg = true;
			if(!datasave_flg) HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14); // 200ms Orange BLINKING
		}
		
		if(ms_sv_tmr % 20000 == 0){ // 10sec later - date
			datasave_flg = false;
			ms_sv_tmr = 0;
		}
	
	// save data 10sec	
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
