/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MASK_CALENTADOR_OK 		 0x00000001U
#define MASK_CALENTADOR_FRIO	 0x00000010U
#define MASK_CALENTADOR_CALIENTE 0x00000100U

#define MASK_RIEGO_OK 		 		 0x00000001U
#define MASK_RIEGO_DEPOSITO_VACIO	 0x00000010U
#define MASK_RIEGO_REGANDO 			 0x00000100U

#define AGUA_LIM 1100
#define TEMP_LIM_CALIENTE 1300
#define TEMP_LIM_FRIO 800
#define LUM_LIM 600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tGestion */
osThreadId_t tGestionHandle;
const osThreadAttr_t tGestion_attributes = {
  .name = "tGestion",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tLectura */
osThreadId_t tLecturaHandle;
const osThreadAttr_t tLectura_attributes = {
  .name = "tLectura",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tSalidas */
osThreadId_t tSalidasHandle;
const osThreadAttr_t tSalidas_attributes = {
  .name = "tSalidas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for tMonitorizar */
osThreadId_t tMonitorizarHandle;
const osThreadAttr_t tMonitorizar_attributes = {
  .name = "tMonitorizar",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for colaValues */
osMessageQueueId_t colaValuesHandle;
const osMessageQueueAttr_t colaValues_attributes = {
  .name = "colaValues"
};
/* Definitions for mutexGestor */
osMutexId_t mutexGestorHandle;
const osMutexAttr_t mutexGestor_attributes = {
  .name = "mutexGestor"
};
/* Definitions for FlagLectura */
osEventFlagsId_t FlagLecturaHandle;
const osEventFlagsAttr_t FlagLectura_attributes = {
  .name = "FlagLectura"
};
/* Definitions for evFlgCalentador */
osEventFlagsId_t evFlgCalentadorHandle;
const osEventFlagsAttr_t evFlgCalentador_attributes = {
  .name = "evFlgCalentador"
};
/* Definitions for evFlgRiego */
osEventFlagsId_t evFlgRiegoHandle;
const osEventFlagsAttr_t evFlgRiego_attributes = {
  .name = "evFlgRiego"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartGestion(void *argument);
void StartLectura(void *argument);
void StartSalidas(void *argument);
void StartMonitorizar(void *argument);

/* USER CODE BEGIN PFP */

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
	tGestionHandle = osThreadNew(StartGestion, NULL, &tGestion_attributes);
	tLecturaHandle = osThreadNew(StartLectura, NULL, &tLectura_attributes);
	tSalidasHandle = osThreadNew(StartSalidas, NULL, &tSalidas_attributes);
	tMonitorizarHandle = osThreadNew(StartMonitorizar, NULL,
			&tMonitorizar_attributes);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	FlagLecturaHandle = osEventFlagsNew(&FlagLectura_attributes);

	evFlgCalentadorHandle = osEventFlagsNew(&evFlgCalentador_attributes);
	evFlgRiegoHandle = osEventFlagsNew(&evFlgRiego_attributes);

	colaValuesHandle = osMessageQueueNew(1, 16, NULL);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/**
* @}
*/
/**
* @}
*/

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.Mode = UART_MODE_TX;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB13 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGestion */
/**
 * @brief Function implementing the tGestion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGestion */
void StartGestion(void *argument)
{
  /* USER CODE BEGIN StartGestion */
	uint32_t agua = 0, temp = 0, luz = 0;
	uint32_t valuesGestion[3];
	/* Infinite loop */
	for (;;) {
		osMutexAcquire(mutexGestorHandle, osWaitForever);
		osEventFlagsWait(FlagLecturaHandle, 0x00000001U, osFlagsWaitAny,
		osWaitForever);
		osMessageQueueGet(colaValuesHandle, valuesGestion, 4, 0);
		agua = valuesGestion[0], temp = valuesGestion[1], luz = valuesGestion[2];

		//riego
		if (luz <= LUM_LIM && agua > AGUA_LIM)
			osEventFlagsSet(evFlgRiegoHandle, MASK_RIEGO_REGANDO);
		else if (agua <= AGUA_LIM)
			osEventFlagsSet(evFlgRiegoHandle, MASK_RIEGO_DEPOSITO_VACIO);
		else
			osEventFlagsSet(evFlgRiegoHandle, MASK_RIEGO_OK);

		//calentador
		if (temp <= TEMP_LIM_FRIO)
			osEventFlagsSet(evFlgCalentadorHandle, MASK_CALENTADOR_FRIO);
		else if (temp > TEMP_LIM_CALIENTE)
			osEventFlagsSet(evFlgCalentadorHandle, MASK_CALENTADOR_CALIENTE);
		else
			osEventFlagsSet(evFlgCalentadorHandle, MASK_CALENTADOR_OK);

		osEventFlagsClear(FlagLecturaHandle, 0x00000001U);
		osMutexRelease(mutexGestorHandle);
		osDelay(200);
	}
  /* USER CODE END StartGestion */
}

/* USER CODE BEGIN Header_StartLectura */
/**
 * @brief Function implementing the tLectura thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLectura */
void StartLectura(void *argument)
{
  /* USER CODE BEGIN StartLectura */
	int32_t agua = 0, luz = 0, temp = 0;
	int32_t values[3];
	/* Infinite loop */
	for (;;) {
		osMutexAcquire(mutexGestorHandle, 0);
		osMutexRelease(mutexGestorHandle);
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			//lectura canal 0
			agua = HAL_ADC_GetValue(&hadc1);
		}
		//lectura canal 2
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			luz = HAL_ADC_GetValue(&hadc1);
		}
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			//lectura canal 3
			temp = HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);
		values[0] = agua, values[1] = temp, values[2] = luz;
		osMessageQueuePut(colaValuesHandle, values, 4, 0);
		osEventFlagsSet(FlagLecturaHandle, 0x00000001U);
		osDelay(90);
	}
  /* USER CODE END StartLectura */
}

/* USER CODE BEGIN Header_StartSalidas */
/**
 * @brief Function implementing the tSalidas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSalidas */
void StartSalidas(void *argument)
{
  /* USER CODE BEGIN StartSalidas */
	uint32_t e_calentador = 0, e_riego = 0;
	/* Infinite loop */
	for (;;) {
		osMutexAcquire(mutexGestorHandle, 0);
		osMutexRelease(mutexGestorHandle);

		e_calentador = osEventFlagsGet(evFlgCalentadorHandle);
		e_riego = osEventFlagsGet(evFlgRiegoHandle);

		switch (e_calentador) {
		case MASK_CALENTADOR_CALIENTE:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			osEventFlagsClear(evFlgCalentadorHandle, MASK_CALENTADOR_CALIENTE);
			break;
		case MASK_CALENTADOR_FRIO:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
			osEventFlagsClear(evFlgCalentadorHandle, MASK_CALENTADOR_FRIO);
			break;
		default:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			osEventFlagsClear(evFlgCalentadorHandle, MASK_CALENTADOR_OK);
			break;
		}

		switch (e_riego) {
		case MASK_RIEGO_DEPOSITO_VACIO:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
			osEventFlagsClear(evFlgRiegoHandle, MASK_RIEGO_DEPOSITO_VACIO);
			break;
		case MASK_RIEGO_REGANDO:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
			osEventFlagsClear(evFlgRiegoHandle, MASK_RIEGO_REGANDO);
			break;
		default:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
			osEventFlagsClear(evFlgRiegoHandle, MASK_RIEGO_OK);
			break;
		}
		osDelay(100);
	}
  /* USER CODE END StartSalidas */
}

/* USER CODE BEGIN Header_StartMonitorizar */
/**
 * @brief Function implementing the tMonitorizar thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMonitorizar */
void StartMonitorizar(void *argument)
{
  /* USER CODE BEGIN StartMonitorizar */
	uint32_t valuesGestion[3];
	/* Infinite loop */
	for (;;) {
		osMutexAcquire(mutexGestorHandle, 0);
		osMutexRelease(mutexGestorHandle);
		osMessageQueueGet(colaValuesHandle, valuesGestion, 4, 0);

		HAL_UART_Transmit(&huart1, valuesGestion, sizeof(valuesGestion), 80);
		osDelay(100);
	}
  /* USER CODE END StartMonitorizar */
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
	while (1) {
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
