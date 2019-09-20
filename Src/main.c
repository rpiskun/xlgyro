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
#include <string.h>
#include <stdio.h>
#include "lsm9ds1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DATA_BUF_SIZE               (1024)
#define DATA_BUFS_NUM               (2)
#define CLOCK_NUM_TO_RELEASE_I2C    (9)

typedef struct DATA_BUF_STRUCT
{
    uint32_t readySamplesNum;
    struct
    {
        RAW_DATA_S aBuf[DATA_BUF_SIZE];
        RAW_DATA_S gBuf[DATA_BUF_SIZE];
    } agBufs;
} DATA_BUF_S;

typedef struct CIRCULAR_BUF_STRUCT
{
    uint32_t activeBufIdx;
    DATA_BUF_S buf[DATA_BUFS_NUM];
} CIRCULAR_BUF_S;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
static const LSM9DS1_CONFIG_S lsm9ds1Config = {
    .linearAccelerationRate = E_LINEAR_ACCELERATION_RANGE_2,
    .angularRate = E_ANGULAR_RATE_RANGE_245,
    .magneticRange = E_MAGNETIC_RANGE_4,
    .operatingMode = E_OPERATING_MODE_XL_GYRO,
    .xlPowerMode = E_XL_POWER_MODE_952HZ,
    .gyroPowerMode = E_GYRO_POWER_MODE_952HZ,
    .fifoMode = E_FIFO_MODE_CONTINUOUS
};

CIRCULAR_BUF_S data = { 0 };

RAW_DATA_S accelAveraged = { 0 };
RAW_DATA_S gyroAveraged = { 0 };

static volatile bool timeElapsed = false;
static volatile bool printData = false;
static char printBuf[1024] = { 0 };

static volatile uint32_t debug_var = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static bool dataBufSend(CIRCULAR_BUF_S *pBuf);
static void releaseI2cBus();
static void printAveragedData();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Sends 9 clocks to release I2C bus
 *
 */
static void releaseI2cBus()
{
    GPIO_InitTypeDef i2cSclGpio = {0};
    /* Send 9 clocks (SCL) to put slave device
     * into initial state and release bus */
    i2cSclGpio.Pin = (GPIO_PIN_6 | GPIO_PIN_9);
    i2cSclGpio.Mode = GPIO_MODE_OUTPUT_OD;
    i2cSclGpio.Pull = GPIO_PULLUP;
    i2cSclGpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &i2cSclGpio);
    /* Release SDA */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    for (uint8_t i = 0; i < CLOCK_NUM_TO_RELEASE_I2C; ++i)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(10);
    }
    /* Release SCL */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
 * @brief Sets flags to notify that time is elapsed
 *
 * @param[in] htim Pointer to timer handle
 * @note This function is called from interrupt context
 *       so it should be short and fast
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        timeElapsed = true;
    }
    else if (htim == &htim7)
    {
        printData = true;
    }
}

void DataBufValuesAppend(RAW_DATA_S *pAccelValue, RAW_DATA_S *pGyroValue)
{
    const uint32_t idx = data.activeBufIdx;

    if (pAccelValue != NULL && pGyroValue != NULL)
    {
        if (data.buf[idx].readySamplesNum < DATA_BUF_SIZE)
        {
            memcpy(
                &data.buf[idx].agBufs.aBuf,
                pAccelValue,
                sizeof(RAW_DATA_S));

            memcpy(
                &data.buf[idx].agBufs.gBuf,
                pGyroValue,
                sizeof(RAW_DATA_S));

            data.buf[idx].readySamplesNum++;
        }
    }
}

static bool dataBufSend(CIRCULAR_BUF_S *pData)
{
    bool ret = false;
    HAL_StatusTypeDef status = HAL_ERROR;
    const uint32_t idx = pData->activeBufIdx;

    if (pData->buf[idx].readySamplesNum > 0)
    {

        status = HAL_UART_Transmit_DMA(
                        &huart6,
                        (uint8_t*)&pData->buf[idx],
                        pData->buf[idx].readySamplesNum);

        pData->buf[idx].readySamplesNum = 0;

        pData->activeBufIdx++;
        if (pData->activeBufIdx >= DATA_BUFS_NUM)
        {
            pData->activeBufIdx = 0;
        }

        if (status == HAL_OK)
        {
            ret = true;
        }
    }
    else
    {
        ret = true;
    }

    return ret;
}

static void printAveragedData()
{
    uint32_t strLen = 0;
    RAW_DATA_S aAverData = { 0 };
    RAW_DATA_S gAverData = { 0 };
    AXIS_FLOAT_VALUE_S accelValue;
    AXIS_FLOAT_VALUE_S gyroValue;
    (void)LSM9DS1_AccelRawDataAveraged(&aAverData);
    (void)LSM9DS1_GyroRawDataAveraged(&gAverData);

    LSM9DS1_AccelCalc(&aAverData, &accelValue);
    LSM9DS1_GyroCalc(&gAverData, &gyroValue);

    memset(printBuf, 0, sizeof(printBuf));
    sprintf(printBuf, "A: [%+.6f]; [%+.6f]; [%+.6f]\r\n",
                accelValue.axisValue[E_X_AXIS],
                accelValue.axisValue[E_Y_AXIS],
                accelValue.axisValue[E_Z_AXIS]);
    // sprintf(printBuf, "A: [%d]; [%d]; [%d]\r\n",
    //             aAverData.rawData[E_X_AXIS],
    //             aAverData.rawData[E_Y_AXIS],
    //             aAverData.rawData[E_Z_AXIS]);

    strLen = strlen(printBuf);
    HAL_UART_Transmit(&huart6, (uint8_t*)printBuf, strLen, HAL_MAX_DELAY);

    // memset(printBuf, 0, sizeof(printBuf));
    // sprintf(printBuf, "G: [%.6f]; [%.6f]; [%.6f]\r\n",
    //             gyroValue.axisValue[E_X_AXIS],
    //             gyroValue.axisValue[E_Y_AXIS],
    //             gyroValue.axisValue[E_Z_AXIS]);

    // strLen = strlen(printBuf);
    // HAL_UART_Transmit(&huart6, (uint8_t*)printBuf, strLen, HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  LSM9DS1_OPERATION_STATUS_E status = E_LSM9DS1_FAIL;
  uint32_t samples = 0;
  bool dataSent = false;

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    MX_GPIO_Init();
    releaseI2cBus();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
    /* Init LSM9DS1 with given config */
    status = LSM9DS1_Init(&hi2c1, &lsm9ds1Config);
    if (E_LSM9DS1_SUCCESS != status)
    {
        return 1;
    }

    status = LSM9DS1_Calibrate();
    if (E_LSM9DS1_SUCCESS != status)
    {
        return 1;
    }

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        HAL_TIM_Base_Start(&htim6);
        samples = LSM9DS1_PollDataBlocking(DataBufValuesAppend);
        HAL_TIM_Base_Stop(&htim6);

        // if (samples > 0)
        // {
        //     /* Blocking polling finished. Now we can get averaged data */
        //     status = LSM9DS1_AccelRawDataAveraged(&accelAveraged);
        //     if (E_LSM9DS1_SUCCESS != status)
        //     {
        //         return 1;
        //     }

        //     status = LSM9DS1_GyroRawDataAveraged(&gyroAveraged);
        //     if (E_LSM9DS1_SUCCESS != status)
        //     {
        //         return 1;
        //     }

        //     DataBufValuesAppend(&accelAveraged, &gyroAveraged);
        // }

        // if (timeElapsed == true)
        // {
        //     dataSent = dataBufSend(&data);
        //     if (dataSent == true)
        //     {
        //         ENTER_CRITICAL_SECTION();
        //         timeElapsed = false;
        //         EXIT_CRITICAL_SECTION();
        //     }
        // }
        if (printData == true)
        {
            printAveragedData();
            ENTER_CRITICAL_SECTION();
            printData = false;
            EXIT_CRITICAL_SECTION();
        }

    // HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  htim6.Init.Prescaler = 42000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 42000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 200;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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

  /* DMA interrupt init */
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
