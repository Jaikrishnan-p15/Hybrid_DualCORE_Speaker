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
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
// --- AUDIO BUFFER CONFIGURATION ---
#define BUFFER_SIZE 8192      // 8KB Buffer (approx 0.2 sec audio)
uint8_t rx_buffer[BUFFER_SIZE]; // Tank to store audio from ESP32

// Pointers to track playback
volatile uint16_t read_ptr = 0;  // Position we are playing to Speaker
volatile uint16_t write_ptr = 0; // Position DMA is writing from ESP32
/* USER CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

    // --- 1. START THE METRONOME (Timer 6) ---
    // Starts the 44.1kHz heartbeat interrupt
    HAL_TIM_Base_Start_IT(&htim6);

    // --- 2. START THE VOICE (DAC) ---
    // Enables DAC Channel 1 (PA4)
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    // --- 3. OPEN THE GATES (SPI3 + DMA) ---
    // Starts receiving data into rx_buffer in CIRCULAR mode.
    // NOTE: We use &hspi3 now (not hspi1)
    HAL_SPI_Receive_DMA(&hspi3, rx_buffer, BUFFER_SIZE);

    // --- 4. INITIAL HANDSHAKE ---
    // Tell ESP32 "I am ready, send data!" (High)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    /* USER CODE END 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /* USER CODE END WHILE */

	      /* USER CODE BEGIN 3 */
	      // --- CALCULATE BUFFER STATUS ---
	      // DMA Counter counts DOWN (Remaining bytes).
	      // So: Total Size - Remaining = How much is written.
	      write_ptr = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hspi3.hdmarx);

	      // Calculate distance between Write and Read pointers
	      int16_t distance = write_ptr - read_ptr;

	      // Handle "Wrap Around" (e.g., Write is at 10, Read is at 8000)
	      if (distance < 0) {
	          distance += BUFFER_SIZE;
	      }

	      // --- TRAFFIC POLICE (Handshake) ---
	      // If buffer is getting too FULL (> 80%), tell ESP32 to PAUSE.
	      if (distance > (BUFFER_SIZE * 0.8)) {
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	      }
	      // If buffer has space again (< 40%), tell ESP32 to RESUME.
	      else if (distance < (BUFFER_SIZE * 0.4)) {
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	      }

	      // Small delay to save power, logic doesn't need to run at MHz speeds
	      HAL_Delay(1);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
// This Interrupt runs exactly 44,100 times per second (controlled by TIM6)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check if it is Timer 6 calling us
  if (htim->Instance == TIM6)
  {
    // 1. Get the current audio byte
    uint8_t sample = rx_buffer[read_ptr];

    // 2. Scale 8-bit (0-255) to 12-bit (0-4095) for STM32 DAC
    // Simple Math: Multiply by 16 (or Left Shift by 4)
    uint16_t dac_value = (uint16_t)sample << 4;

    // 3. Send to DAC Register (Plays the sound!)
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

    // 4. Advance the Read Pointer
    read_ptr++;

    // 5. Wrap around if we hit the end of the buffer
    if (read_ptr >= BUFFER_SIZE) {
        read_ptr = 0;
    }
  }
}

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
