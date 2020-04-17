/* USER CODE BEGIN Header */
/*Definition in INSOLE_R_KR
DATASET :
INSOLE_L : 10BYTE(FSR) + 4BYTE(UWB) + 4BYTE(ST/ED) + 2BYTE(CRC) = 20BYTE
INSOLE_R : 20BYTE(FSR) + 8BYTE(UWB) + 4BYTE(ST/ED) + 2BYTE(CRC)= 34BYTE
IMU 		 : 20BYTE(FSR) + 18BYTE(IMU) + 4BYTE(ST/ED) + 2BYTE(CRC) = 54BYTE
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
//define : MPU9250
	#define MPU9250_ADDRESS	0xD2
	#define AK8963_ADDRESS   0x0C<<1
	#define AK8963_ST1       0x02  // data ready status bit 0	
	#define AK8963_XOUT_L    0x03  // data
	#define ACCEL_XOUT_H     0x3B
	#define GYRO_XOUT_H      0x43
	#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
	#define ACCEL_CONFIG     0x1C
	#define CONFIG           0x1A
	#define INT_PIN_CFG      0x37
	#define INT_ENABLE       0x38
	#define ACCEL_CONFIG2    0x1D
	#define GYRO_CONFIG      0x1B
	#define SMPLRT_DIV       0x19
	#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
	#define PWR_MGMT_2       0x6C
	#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR  
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;
osThreadId defaultTaskHandle2;

/* USER CODE BEGIN PV */
	
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartDefaultTask2(void const * argument);

void wdg_activate(void);
void initMPU9250(void);
void readMagData(int8_t * destination);
void readGyroData(int8_t * destination);
void readAccelData(int8_t * destination);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
char readByte(uint8_t address, uint8_t subAddress);

void initAK8963(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	//watchdog
	
	uint8_t wdg_cnt = 0;
	// I2C and IMU
	int8_t acc_[6] = {0,};
	int8_t gy_[6] = {0,};
	int8_t mag_[6] = {0,};;
	
	//recieved data
	uint16_t receive_val;
	uint16_t cal_val;
	
	
	// Communication
	bool received_flag = false;
	uint8_t txdata[54];
	uint8_t rxBuf[34];
	uint8_t rxBuf_[34];
	uint8_t rxBuf_crc[34];

	uint8_t i2cBuf[7];
	uint8_t Imu_dataBuf[18] = {0,};\
	uint8_t buff_index = 0;
	
	bool data_received_dma = true;
		
	//crc
	uint32_t crcArray[8] = {0,};
	uint32_t crcArray_send[12] = {0,};

	uint16_t crcval = 0;
	
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
	
	//data save
	bool datasave_flg = false;
 

	//Timer
	uint16_t ms_tmr = 0;
	uint16_t _10ms_tmr = 0;
	uint16_t send_10ms_tmr = 0;

	uint16_t ms_sv_tmr = 0;
	bool _100ms_flg = false;
	bool _10ms_flg = false;
	bool dma_connect = false;
	
	uint16_t test_val = 0;
	
	
	
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SDIO_SD_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxBuf_crc, 34); //interrupt mode only in the mcu data
		HAL_TIM_Base_Start_IT(&htim4);
	
	initMPU9250();
	initAK8963();

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
  osThreadDef(defaultTaskHandle, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTaskHandle), NULL);
	
	//osThreadDef(defaultTaskHandle2, StartDefaultTask2, osPriorityNormal, 0, 128);
  //defaultTaskHandle2 = osThreadCreate(osThread(defaultTaskHandle2), NULL);

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
  hi2c1.Init.ClockSpeed = 1000000;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
   uint8_t data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
   HAL_I2C_Master_Transmit(&hi2c1, address, data_write, 2, 10);
}
char readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data[1]; // `data` will store the register data     
    uint8_t data_write[1];
    data_write[0] = subAddress;
	HAL_I2C_Master_Transmit(&hi2c1, address, data_write, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, address, data, 1, 10); // Data receive sequencing from i2cBuf[1]
    return data[0]; 
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{     
    uint8_t data[14];
    uint8_t data_write[1];
    data_write[0] = subAddress;
	HAL_I2C_Master_Transmit(&hi2c1, address, data_write, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, address, data, count, 10); // Data receive sequencing from i2cBuf[1]
    for(int ii = 0; ii < count; ii++) {
     dest[ii] = data[ii];
    }
} 



void readAccelData(int8_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  uint8_t data_write[1];
  data_write[0] = ACCEL_XOUT_H;
  HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, data_write, 1,10);
  HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDRESS, rawData, 6, 10); // Data receive sequencing from i2cBuf[1]

  memcpy(&destination[0], &rawData[0],6); 
}

void readGyroData(int8_t * destination)
{
	
  uint8_t rawData[6];  // x/y/z accel register data stored here
  uint8_t data_write[1];
  data_write[0] = GYRO_XOUT_H;
  HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, data_write, 1,10);
  HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDRESS, rawData, 6, 10); // Data receive sequencing from i2cBuf[1]
  memcpy(&destination[0], &rawData[0],6); 
 
}

void  readMagData(int8_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
	 uint8_t data_write[1];
	data_write[0] = AK8963_XOUT_L;
	HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, data_write, 1,10);
	HAL_I2C_Master_Receive(&hi2c1, AK8963_ADDRESS, rawData, 7, 10); // Data receive sequencing from i2cBuf[1] 
	uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
		memcpy(&destination[0], &rawData[0],6); 
		
   }
  }
}

 void initAK8963()
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  HAL_Delay(100);	
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(100);	
//  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
//  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
//  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
//  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  HAL_Delay(100);	
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  HAL_Delay(100);	
}

void initMPU9250()
{  
// Initialize MPU9250 device
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  
   HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
   HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
    HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
    HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
  HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
     HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
     HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	if(huart->Instance == USART1){  
		
	}
	/* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
	*/
}


 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	//add ring buffer
	// data pacing and save buff to transmit array
	if(huart->Instance == USART2){
				//if(test_val !=0){
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				if(rxBuf_crc[0] == 0xFF && rxBuf_crc[1] == 0xFF){
					received_flag = true;
					dma_connect = true;
					memcpy(&rxBuf_[0], &rxBuf_crc[0], 34 );
				}
				else
				{
					received_flag = false;
					HAL_DMA_Abort(&hdma_usart2_rx);	
				}
				test_val++;
	}
					
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
//	f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);

//	if(f_open(&SDFile, "test.txt", FA_CREATE_ALWAYS | FA_WRITE )== FR_OK){
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		osDelay(500);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		osDelay(500);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		osDelay(500);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//	}
//	else{
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		osDelay(2000);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		osDelay(2000);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		osDelay(2000);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
//		}


  /* Infinite loop */
  for(;;)
  {
		if(_10ms_flg){
		_10ms_flg = false;

		if(received_flag) { 
			uint8_t cprxBuf[34];
			memcpy(&cprxBuf[0], &rxBuf_[0], sizeof(uint8_t)*34); 
			received_flag = false;
			memset(crcArray,0,8*sizeof(crcArray[0]));
			memmove(&crcArray[0], &cprxBuf[2], sizeof(uint8_t)*28); 
			char crc_begin = 30; //MLSB 32 = FF 33 = FE c
			char crc_end = 31;   //LLSB	
			receive_val = (int16_t)(((int16_t)cprxBuf[crc_begin] << 8) | cprxBuf[crc_end]);
			cal_val = (HAL_CRC_Calculate(&hcrc,crcArray,7)&0xffff);
			if(cal_val == receive_val)
			{
				memmove(&txdata[4], &cprxBuf[2], sizeof(uint8_t)*28); 
			}			
			else
			{
				wdg_cnt ++;
				continue;
			}
		}
		
		readAccelData(&acc_[0]);
		readMagData(&mag_[0]);
		readGyroData(&gy_[0]); 
	
		memmove(&txdata[32], &acc_[0],6); 
		memmove(&txdata[38], &mag_[0],6); 
		memmove(&txdata[44], &gy_[0],6); 

		txdata[0] = 0xFF;
		txdata[1] = 0xFF;
		txdata[2] = send_10ms_tmr >> 8;
		txdata[3] = send_10ms_tmr & 0xff;	
		memset(crcArray_send,0,12*sizeof(crcArray_send[0]));
		//50byte crc
		for(int i =0; i <12; i++)
		{
			char c = 4*i;
			crcArray_send[i] = ((uint32_t)txdata[c+2] << 24) | ((uint32_t)txdata[c+3] << 16) | ((uint32_t)txdata[c+4] << 8) | ((uint32_t)txdata[c+5]);
		}
		
		crcval = HAL_CRC_Calculate(&hcrc,crcArray_send,12)& 0xffff;
	
		txdata[50] = crcval >> 8; 
		txdata[51] = crcval & 0xff;
		txdata[52] = 0xFF;
		txdata[53] = 0xFE;
	
		HAL_UART_Transmit_IT(&huart1,txdata,54);
	}
  }
  /* USER CODE END 5 */ 
}



void StartDefaultTask2(void const * argument)
{                
  /* init code for FATFS */

  /* Infinite loop */
  for(;;)
  {

	

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
		_10ms_tmr++;
		ms_sv_tmr ++;
		if(ms_tmr % 10 == 0){
			_10ms_flg = true;
			ms_tmr = 0;
		}
		if(_10ms_tmr % 10 == 0){
			send_10ms_tmr++;
			_10ms_tmr = 0;
		}
		
		if(ms_sv_tmr % 100 == 0){ // 10sec later - date
			datasave_flg = false;
		if(!dma_connect) HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxBuf_crc, 34); //interrupt mode only in the mcu data
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




//	SD card Data transmit

//   	if(datasave_flg){
//	for(int i =0; i<24; i++) {
//	f_printf(&SDFile, "%02x", txdata[i]);	
//	}
//	f_printf(&SDFile, "\n");	

//	}
//	else
//	{
//		f_close(&SDFile);
//	}
//	

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
