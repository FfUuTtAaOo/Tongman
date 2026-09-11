/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lha7668.h"
#include "filter.h"
#include "flash.h"
#include "function.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SAMPLING_RATE   3

#define APPINFO_VERSION (uint8_t*)"v0.1.2"
#define APPINFO_BUILD_DATE (uint8_t*)"2026-04-29"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
lhl_lha7668_ctx_t lha7668_ctx = { }; // LHA7668驱动句柄

/* Being externally cited variables */
uint8_t data_flag = 0;
uint8_t rev_data[29] = { 0 };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LHA7668_Platform_Init(void *handle)
{
  lhl_lha7668_ctx_t *dev_ctx = (lhl_lha7668_ctx_t *)handle;
  dev_ctx->io.cs.port = GPIOA;
  dev_ctx->io.cs.pin = GPIO_PIN_4;
}

void LHA7668_Platform_Set(const void *port, const uint32_t pin)
{
  HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin, GPIO_PIN_SET);
}

void LHA7668_Platform_Reset(const void *port, const uint32_t pin)
{
  HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin, GPIO_PIN_RESET);
}

int32_t LHA7668_Platform_ReadWrite(uint8_t *txdata, uint8_t *rxdata, const uint16_t size)
{
  HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, size, 0xFFFF);

  return 0;
}

float adcData(void)
{
    LHL_LHA7668_Start(&lha7668_ctx, LHA7668_MODE_SINGLE_SHOT);

    while (LHL_LHA7668_Get_Flag(&lha7668_ctx, LHA7668_STATUS_RDY_FLAG) == LHA7668_SET);

    LHL_LHA7668_Get_Data(&lha7668_ctx);

    return LHL_LHA7668_Get_mVoltage(lha7668_ctx.data, LHA7668_BIPOLAR, LHA7668_PGA_X128, 2500);
}

void lha7668_init(uint8_t rate)
{
    LHL_LHA7668_Init(&lha7668_ctx);
    LHL_LHA7668_Reset(&lha7668_ctx);

    uint8_t whoamI = LHL_LHA7668_Get_ID(&lha7668_ctx);
    if (whoamI != LHA7668B_8) {
        printf("Equipment error!\n");
        while(1) {
        }
    }

    LHL_LHA7668_Stop(&lha7668_ctx);

    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN0;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN1;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_0);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN2;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN3;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_1);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN4;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN5;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_2);

    lha7668_ctx.SETUP.BIPOLAR = LHA7668_BIPOLAR;
    lha7668_ctx.SETUP.AIN_BUFP = LHA7668_ENABLE;
    lha7668_ctx.SETUP.AIN_BUFM = LHA7668_ENABLE;
    lha7668_ctx.SETUP.REF_SEL = LHA7668_REF_REFIN1;
    lha7668_ctx.SETUP.REF_BUFP = LHA7668_DISABLE;
    lha7668_ctx.SETUP.REF_BUFM = LHA7668_DISABLE;
    lha7668_ctx.SETUP.PGA = LHA7668_PGA_X128;
    lha7668_ctx.SETUP.FILTER = LHA7668_FILTER_SINC3;
    lha7668_ctx.SETUP.FS = rate;
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_0);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_1);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_2);
    lha7668_ctx.ADC_CTRL.POWER_MODE = LHA7668_FULL_POWER;
    lha7668_ctx.ADC_CTRL.CLK_SEL = LHA7668_CLK_INTL;
    lha7668_ctx.ADC_CTRL.DATA_STATUS = LHA7668_ENABLE;
    lha7668_ctx.ADC_CTRL.CS_EN = LHA7668_DISABLE;
    lha7668_ctx.ADC_CTRL.DOUT_RDY_DEL = LHA7668_DISABLE;
    LHL_LHA7668_Set_ADC(&lha7668_ctx);
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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    lha7668_init(ADC_SAMPLING_RATE);
    FloatFilter_Init();
    configure_CAN_filter();
    CAN_Wheel_Init();
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //开启接收中断
    HAL_TIM_Base_Start_IT(&htim2);

    zero_clearing();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    	send_ret();
    	HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
