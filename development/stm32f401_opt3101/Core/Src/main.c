/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "opt3101.h"
#include <stdarg.h>
#include <stdio.h>

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static OPT3101_Device opt_front;
static OPT3101_Device opt_bottom;
static OPT3101_Measurement opt_measurement;
static uint32_t opt_front_last_attempt = 0U;
static uint32_t opt_bottom_last_attempt = 0U;
static uint8_t opt_front_attempted = 0U;
static uint8_t opt_bottom_attempted = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void APP_Printf(const char *format, ...)
{
    char buffer[192];
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0) return;
    if ((size_t)length >= sizeof(buffer)) length = (int)sizeof(buffer) - 1;
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buffer,
                            (uint16_t)length, 100U);
}
static void APP_ReportOptInit(const char *name,
                              const OPT3101_Device *dev,
                              OPT3101_Status status)
{
    if (status == OPT3101_STATUS_OK)
    {
        APP_Printf(
            "%s OPT3101 initialized at address 0x%02X\r\n",
            name,
            (unsigned int)dev->address_7bit
        );
    }
    else
    {
        APP_Printf(
            "%s OPT3101 init failed: %s "
            "(HAL_Status=%d, I2C_State=0x%02X, I2C_Error=0x%08lX)\r\n",
            name,
            OPT3101_StatusString(status),
            (int)OPT3101_GetLastHALStatus(dev),
            (unsigned int)HAL_I2C_GetState(dev->hi2c),
            (unsigned long)HAL_I2C_GetError(dev->hi2c)
        );
    }
}

static void APP_ReportMeasurement(const char *name,
                                  const OPT3101_Measurement *m)
{
    APP_Printf("%s phase=%u distance_nominal=%lu mm amplitude=%u ambient=%u "
               "temp_nominal=%d C channel=%u frame=%u valid=%u "
               "(phase_ovl=%u phase_f2_ovl=%u signal_ovl=%u ambient_ovl=%u)\r\n",
               name,
               (unsigned int)m->phase_raw,
               (unsigned long)m->distance_mm_nominal,
               (unsigned int)m->amplitude,
               (unsigned int)m->ambient,
               (int)m->temperature_c_nominal,
               (unsigned int)m->channel,
               (unsigned int)m->frame_counter,
               (unsigned int)m->distance_valid,
               (unsigned int)m->phase_overflow,
               (unsigned int)m->phase_overflow_f2,
               (unsigned int)m->signal_overflow,
               (unsigned int)m->ambient_overflow);
}

static void APP_MarkForReinitialization(OPT3101_Device *dev)
{
    dev->initialized = false;
    dev->timing_generator_enabled = false;
    dev->sample_pending = false;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OPT3101_Status attach_status;

  attach_status = OPT3101_Attach(&opt_front, &hi2c1,
                                 OPT3101_ADDR_FRONT_7BIT,
                                 OPT3101_DEFAULT_I2C_TIMEOUT_MS);
  APP_Printf("Front OPT3101 attach: %s\r\n",
             OPT3101_StatusString(attach_status));

  attach_status = OPT3101_Attach(&opt_bottom, &hi2c1,
                                 OPT3101_ADDR_BOTTOM_7BIT,
                                 OPT3101_DEFAULT_I2C_TIMEOUT_MS);
  APP_Printf("Bottom OPT3101 attach: %s\r\n",
             OPT3101_StatusString(attach_status));

  APP_Printf("\r\n=== NIAT OPT3101 diagnostic ===\r\n");
  APP_Printf("I2C1 ready; front=0x5E, bottom=0x5F (7-bit addresses)\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    OPT3101_Status status;

    /* Retry initialization of each unavailable sensor once per second. */
    if (!opt_front.initialized &&
        ((!opt_front_attempted) ||
         ((uint32_t)(now - opt_front_last_attempt) >= 1000U)))
    {
        opt_front_attempted = 1U;
        opt_front_last_attempt = now;
        status = OPT3101_Init(&opt_front);
        APP_ReportOptInit("Front", &opt_front, status);
    }

    now = HAL_GetTick();
    if (!opt_bottom.initialized &&
        ((!opt_bottom_attempted) ||
         ((uint32_t)(now - opt_bottom_last_attempt) >= 1000U)))
    {
        opt_bottom_attempted = 1U;
        opt_bottom_last_attempt = now;
        status = OPT3101_Init(&opt_bottom);
        APP_ReportOptInit("Bottom", &opt_bottom, status);
    }

    if (opt_front.initialized)
    {
        status = OPT3101_Sample(&opt_front, &opt_measurement);
        if (status == OPT3101_STATUS_OK)
        {
            APP_ReportMeasurement("Front", &opt_measurement);
        }
        else
        {
            APP_Printf("Front sample error: %s\r\n", OPT3101_StatusString(status));
            if ((status == OPT3101_STATUS_I2C_ERROR) ||
                (status == OPT3101_STATUS_INIT_TIMEOUT))
            {
                APP_MarkForReinitialization(&opt_front);
            }
        }
    }

    if (opt_bottom.initialized)
    {
        status = OPT3101_Sample(&opt_bottom, &opt_measurement);
        if (status == OPT3101_STATUS_OK)
        {
            APP_ReportMeasurement("Bottom", &opt_measurement);
        }
        else
        {
            APP_Printf("Bottom sample error: %s\r\n", OPT3101_StatusString(status));
            if ((status == OPT3101_STATUS_I2C_ERROR) ||
                (status == OPT3101_STATUS_INIT_TIMEOUT))
            {
                APP_MarkForReinitialization(&opt_bottom);
            }
        }
    }

    HAL_Delay(10U);
    /* USER CODE END 3 */
  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
#ifdef USE_FULL_ASSERT
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
