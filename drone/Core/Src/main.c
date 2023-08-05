/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Federico Nanni 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"
#include "accel.h"
#include "attitude.h"
#include "baro.h"
#include "control_motor.h"
#include "gyro.h"
#include "mag.h"

#include "string.h"
#include <stdio.h>
#include "iks01a2_env_sensors.h"
#include "iks01a2_motion_sensors.h"
#include "lsm6dsl_reg.h"
#include "lsm303agr_reg.h"
#include "lps22hb_reg.h"

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

UART_HandleTypeDef huart3;

/* Definitions for tControlMotor */
osThreadId_t tControlMotorHandle;
const osThreadAttr_t tControlMotor_attributes = {
  .name = "tControlMotor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for tAttitude */
osThreadId_t tAttitudeHandle;
const osThreadAttr_t tAttitude_attributes = {
  .name = "tAttitude",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for tAltitude */
osThreadId_t tAltitudeHandle;
const osThreadAttr_t tAltitude_attributes = {
  .name = "tAltitude",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for printUart */
osThreadId_t printUartHandle;
osThreadAttr_t printUart_attributes = {
  .name = "printUart",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for taskI2cRead */
osThreadId_t taskI2cReadHandle;
const osThreadAttr_t taskI2cRead_attributes = {
  .name = "taskI2cRead",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for semaphoreDataStore */
osSemaphoreId_t semaphoreDataStoreHandle;
const osSemaphoreAttr_t semaphoreDataStore_attributes = {
  .name = "semaphoreDataStore"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void startTaskControlMotor(void *argument);
void startTaskAttitude(void *argument);
void startTaskAltitude(void *argument);
void startPrintUart(void *argument);
void taskI2cR(void *argument);

/* USER CODE BEGIN PFP */
static void sensorGyroInit(struct gyroDev_s *gyro);
static bool sensorGyroRead(struct gyroDev_s *gyro);
static void sensorAccInit(struct accDev_s *acc);
static bool sensorAccRead(struct accDev_s *acc);
static void sensorMagInit(struct magDev_s *mag);
static bool sensorMagRead(struct magDev_s *mag);
static void sensorBaroInit(struct baroDev_s *baro);
static bool sensorBaroRead(struct baroDev_s *baro);


static void gyroDataRead();
static void accDataRead();
static void magDataRead();
static void baroDataRead();

struct data_s {
	float baro;
	float gyroX;
	float gyroY;
	float gyroZ;
	float acc6DSL_X;
	float acc6DSL_Y;
	float acc6DSL_Z;
	float acc303AGR_X;
	float acc303AGR_Y;
	float acc303AGR_Z;
	float accX;
	float accY;
	float accZ;
	float magX;
	float magY;
	float magZ;
} sensorData;

static const float mdpsToRadDivSec1 = 0.0000174;
static const float mgToMeterDivSec2 = 0.0098066;
static const float mGaussToMicroT = 0.1;

//struct rawData dataStore;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  printf(" \r\n");
  // initializing and enabling LPS22HB BAROMETER
  IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);
  IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);

  // initializing and enabling LSM303AGR ACCELEROMETER
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);

  // initializing and enabling LSM303AGR MAGNETOMETER
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

  // initializing and enabling LSM6DSL ACCELLEROMETER
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);

  // initializing and enabling LSM6DSL GYROSCOPE
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_GYRO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_GYRO);

  //Setting LPS22HB ODR to its max value 50.0hz (75.0hz visible in lps22hb.c but not compliant (??))
  IKS01A2_ENV_SENSOR_SetOutputDataRate(IKS01A2_LPS22HB_0, ENV_PRESSURE, 50.0f);
  //Setting LSM303AGR's accelerometer ODR to 200hz
  IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, 200.0f);
  //Setting LSM303AGR's magnetometer ODR to 100hz (MAX ODR)
  IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, 100.0f);
  //Setting LSM6DSL's accelerometer ODR to 833hz
  IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, 833.0f);
  //Setting LSM6DSL's gyroscope ODR to 833hz
  IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_GYRO, 833.0f);

  /* initialization of control task */
  gyroInit(sensorGyroInit, sensorGyroRead);
  controlMotorInit();

  /* initialization of attitude task*/
  accInit(sensorAccInit, sensorAccRead);
  magInit(sensorMagInit, sensorMagRead);
  attitudeInit();

  /* initialization of altitude task */
  baroInit(sensorBaroInit, sensorBaroRead);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semaphoreDataStore */
  semaphoreDataStoreHandle = osSemaphoreNew(1, 1, &semaphoreDataStore_attributes);

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
  /* creation of tControlMotor */
  tControlMotorHandle = osThreadNew(startTaskControlMotor, NULL, &tControlMotor_attributes);

  /* creation of tAttitude */
  tAttitudeHandle = osThreadNew(startTaskAttitude, NULL, &tAttitude_attributes);

  /* creation of tAltitude */
  tAltitudeHandle = osThreadNew(startTaskAltitude, NULL, &tAltitude_attributes);

  /* creation of printUart */
  printUartHandle = osThreadNew(startPrintUart, NULL, &printUart_attributes);

  /* creation of taskI2cRead */
  taskI2cReadHandle = osThreadNew(taskI2cR, NULL, &taskI2cRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*uint32_t start = HAL_GetTick();
	for (int i = 0; i < 100000; ++i) {*/
		/* first task: control */
	    /*gyroUpdate();
		controlMotorUpdate();*/

		/* second task: attitude */
		/*accUpdate();
		magUpdate();
		attitudeUpdate();*/

		/* third task: altitude */
		/*baroUpdate();
	}*/
	/*volatile uint32_t last = HAL_GetTick() - start;
	Error_Handler();*/
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


int _write(int file, char *ptr, int len){

	HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

static void sensorGyroInit(struct gyroDev_s *gyro) { }

static bool sensorGyroRead(struct gyroDev_s *gyro) {
	gyro->gyroADC[0] = sensorData.gyroX;
	gyro->gyroADC[1] = sensorData.gyroY;
	gyro->gyroADC[2] = sensorData.gyroZ;
	return true;
}

static void sensorAccInit(struct accDev_s *acc) { }

static bool sensorAccRead(struct accDev_s *acc) {
	acc->accADC[0] = sensorData.accX;
	acc->accADC[1] = sensorData.accY;
	acc->accADC[2] = sensorData.accZ;
	return true;
}

static void sensorMagInit(struct magDev_s *mag) { }

static bool sensorMagRead(struct magDev_s *mag) {
	mag->magADC[0] = sensorData.magX;
	mag->magADC[1] = sensorData.magY;
	mag->magADC[2] = sensorData.magZ;
	return true;
}

static void sensorBaroInit(struct baroDev_s *baro) {
	baro->baroADC = 0;
}

static bool sensorBaroRead(struct baroDev_s *baro) {
	baro->baroADC = sensorData.baro;
	return true;
}

static void gyroDataRead() {

	IKS01A2_MOTION_SENSOR_AxesRaw_t gyroAxes;
	IKS01A2_MOTION_SENSOR_GetAxesRaw(IKS01A2_LSM6DSL_0, MOTION_GYRO, &gyroAxes);

	// converting Raw Data to mdps and multiplying by 0.0000174 to get the value in radiant
	sensorData.gyroX = (lsm6dsl_from_fs2000dps_to_mdps(gyroAxes.x))* mdpsToRadDivSec1;
	sensorData.gyroY = (lsm6dsl_from_fs2000dps_to_mdps(gyroAxes.x))* mdpsToRadDivSec1;
	sensorData.gyroZ = (lsm6dsl_from_fs2000dps_to_mdps(gyroAxes.x))* mdpsToRadDivSec1;

}

static void accDataRead() {

	IKS01A2_MOTION_SENSOR_AxesRaw_t  acclsm303agrAxes;
	IKS01A2_MOTION_SENSOR_AxesRaw_t  acclsm6dslAxes;
	IKS01A2_MOTION_SENSOR_GetAxesRaw(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO, &acclsm303agrAxes);
	IKS01A2_MOTION_SENSOR_GetAxesRaw(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &acclsm6dslAxes);

	//Converting Raw data to milligravity and then converting to m/sec^2
	sensorData.acc303AGR_X = (lsm303agr_from_fs_2g_nm_to_mg(acclsm303agrAxes.x)) * mgToMeterDivSec2;
	sensorData.acc303AGR_Y = (lsm303agr_from_fs_2g_nm_to_mg(acclsm303agrAxes.y)) * mgToMeterDivSec2;
	sensorData.acc303AGR_Z = (lsm303agr_from_fs_2g_nm_to_mg(acclsm303agrAxes.z)) * mgToMeterDivSec2;

	sensorData.acc6DSL_X = (lsm6dsl_from_fs2g_to_mg(acclsm6dslAxes.x)) * mgToMeterDivSec2;
	sensorData.acc6DSL_Y = (lsm6dsl_from_fs2g_to_mg(acclsm6dslAxes.y)) * mgToMeterDivSec2;
	sensorData.acc6DSL_Z = (lsm6dsl_from_fs2g_to_mg(acclsm6dslAxes.z)) * mgToMeterDivSec2;

	sensorData.accX  = (sensorData.acc6DSL_X + sensorData.acc303AGR_X)/2;
	sensorData.accY  = (sensorData.acc6DSL_Y + sensorData.acc303AGR_Y)/2;
	sensorData.accZ  = (sensorData.acc6DSL_Z + sensorData.acc303AGR_Z)/2;

}

static void magDataRead() {

	IKS01A2_MOTION_SENSOR_AxesRaw_t  magAxes;
	IKS01A2_MOTION_SENSOR_GetAxesRaw(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO, &magAxes);

	//Dividing raw value as unit/LSB and converting the result to the desired measurement unit
	sensorData.magX = (lsm303agr_from_lsb_to_mgauss(magAxes.x)) * mGaussToMicroT;
	sensorData.magY = (lsm303agr_from_lsb_to_mgauss(magAxes.y)) * mGaussToMicroT;
	sensorData.magZ = (lsm303agr_from_lsb_to_mgauss(magAxes.z)) * mGaussToMicroT;

}

static void baroDataRead() {
	float pressure;

	IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LPS22HB_0, ENV_PRESSURE, &pressure);

	//The absolute LPS22HB pressure range is 260 to 1260 but it occasionally output 0.00f
	if(pressure >= 260){
		sensorData.baro = pressure;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startTaskControlMotor */
/**
  * @brief  Function implementing the tControlMotor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTaskControlMotor */
void startTaskControlMotor(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	gyroUpdate();
	controlMotorUpdate();
    osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTaskAttitude */
/**
* @brief Function implementing the tAttitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAttitude */
void startTaskAttitude(void *argument)
{
  /* USER CODE BEGIN startTaskAttitude */
  /* Infinite loop */
  for(;;)
  {
    accUpdate();
    magUpdate();
    attitudeUpdate();
    osDelay(10);
  }
  /* USER CODE END startTaskAttitude */
}

/* USER CODE BEGIN Header_startTaskAltitude */
/**
* @brief Function implementing the tAltitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAltitude */
void startTaskAltitude(void *argument)
{
  /* USER CODE BEGIN startTaskAltitude */
  /* Infinite loop */
  for(;;)
  {
    baroUpdate();
    osDelay(25);
  }
  /* USER CODE END startTaskAltitude */
}

/* USER CODE BEGIN Header_startPrintUart */
/**
* @brief Function implementing the printUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPrintUart */
void startPrintUart(void *argument)
{
  /* USER CODE BEGIN startPrintUart */
  /* Infinite loop */
  for(;;)
  {
	  printUart_attributes.priority = osPriorityNormal7;

	  osSemaphoreAcquire(semaphoreDataStoreHandle, osWaitForever);

	  printf("Gyroscope LSM6DSL (rad/sec): X= %.2f Y= %.2f Z= %.2f\r\n", sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
	  printf("Accelerometer LSM6DSL (m/sec^2): X= %.2f Y= %.2f Z= %.2f\r\n", sensorData.acc6DSL_X, sensorData.acc6DSL_Y, sensorData.acc6DSL_Z);
	  printf("Accelerometer LSM303AGR (m/sec^2): X= %.2f Y= %.2f Z= %.2f\r\n", sensorData.acc303AGR_X, sensorData.acc303AGR_Y, sensorData.acc303AGR_Z);
	  printf("Accelerometer Average (m/sec^2): X= %.2f Y= %.2f Z= %.2f\r\n", sensorData.accX, sensorData.accY, sensorData.accZ);
	  printf("Magnetometer LSM303AGR (Microtesla uT): X= %.2f Y= %.2f Z= %.2f\r\n", sensorData.magX, sensorData.magY, sensorData.magZ);
	  printf("Barometer= %.1fhPa \r\n", sensorData.baro);

	  printf("\r\n");

	osSemaphoreRelease(semaphoreDataStoreHandle);

	printUart_attributes.priority = osPriorityNormal3;

    osDelay(1500);
  }
  /* USER CODE END startPrintUart */
}

/* USER CODE BEGIN Header_taskI2cR */
/**
* @brief Function implementing the taskI2cRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskI2cR */
void taskI2cR(void *argument)
{
  /* USER CODE BEGIN taskI2cR */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(semaphoreDataStoreHandle, osWaitForever);
	  gyroDataRead();
	  accDataRead();
	  magDataRead();
	  baroDataRead();
	  osSemaphoreRelease(semaphoreDataStoreHandle);

    osDelay(2);
  }
  /* USER CODE END taskI2cR */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
