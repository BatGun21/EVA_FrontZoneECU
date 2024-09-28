/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws28xx.h"
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Enum to define different LED states
typedef enum {
    DRL_MODE,          // Daytime running lights
    TURN_SIGNAL_MODE,  // Turn signal active
    HAZARD_LIGHT_MODE, // Hazard/parking light active
    STARTUP_BLINK_MODE // Startup blinking mode
} LED_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WS28XX_PIXEL_COUNT 28   // Trial strip with 28 pixels
#define WAVE_PACKET_SIZE 7      // Number of LEDs in one wave packet
#define WAVE_SPEED 20           // Delay between each wave step in ms
#define WAVE_STEP_SIZE 1        // Number of pixels the wave moves per frame
#define WAVE_PAUSE 100          // Pause between waves

#define STARTUP_BLINK_DELAY 500 // 500ms for blink in startup
#define DRL_BRIGHTNESS 255      // Full white brightness for DRL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws_pa8;    // Left strip (PA8)
int frame_pa8 = 0;              // Frame counter for wave effect on PA8
LED_Mode current_mode = DRL_MODE;  // Default mode is DRL
int startup_state = 0;          // 0 = off, 1 = blinking, 2 = complete
              // Frame counter for wave effect on PA8
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame);  // Wave effect function for PA8
void UpdateDRLMode(WS28XX_HandleTypeDef* ws);                // DRL mode (white light)
void UpdateStartupBlink(WS28XX_HandleTypeDef* ws, int blink_on);  // Startup blinking mode
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  WS28XX_Init(&ws_pa8, &htim1, 36, TIM_CHANNEL_1, WS28XX_PIXEL_COUNT);  // Initialize for PA8
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      switch (current_mode)
      {
          case TURN_SIGNAL_MODE:
              // Call the wave effect function for the left strip (PA8) if turn signal is active
              UpdateWaveEffect(&ws_pa8, frame_pa8);
              frame_pa8 += WAVE_STEP_SIZE;
              if (frame_pa8 >= WS28XX_PIXEL_COUNT) frame_pa8 = 0;
              HAL_Delay(WAVE_SPEED);  // Delay for smooth wave animation
              break;

          case HAZARD_LIGHT_MODE:
              // Wave pattern if parking/hazard lights are on
              UpdateWaveEffect(&ws_pa8, frame_pa8);
              frame_pa8 += WAVE_STEP_SIZE;
              if (frame_pa8 >= WS28XX_PIXEL_COUNT) frame_pa8 = 0;
              HAL_Delay(WAVE_SPEED);
              break;

          case STARTUP_BLINK_MODE:
              // Run startup sequence (blinking in amber)
              UpdateStartupBlink(&ws_pa8, 1);  // Blink on
              HAL_Delay(STARTUP_BLINK_DELAY);
              UpdateStartupBlink(&ws_pa8, 0);  // Blink off
              HAL_Delay(STARTUP_BLINK_DELAY);
              // Continue startup sequence for a few cycles (e.g., 5 blinks)
              startup_state++;  // Move to next state after blinking
              if (startup_state >= 5) {
                  current_mode = DRL_MODE;  // Switch to DRL after startup
              }
              break;

          case DRL_MODE:
          default:
              // Default to DRL mode if no signals are active and startup is complete
              UpdateDRLMode(&ws_pa8);
              break;
      }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Update the PA8 LED strip with a wave effect (turn signal / hazard light)
  * @param ws: Pointer to the WS28XX handle
  * @param frame: Current frame number for the wave effect
  * @retval None
  */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame)
{
    for (int i = 0; i < WS28XX_PIXEL_COUNT; i++) {
        if (i >= frame && i < frame + WAVE_PACKET_SIZE) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_AMBER, 255);  // Use Amber color
        } else {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);    // Turn off (black)
        }
    }
    WS28XX_Update(ws);  // Refresh the LED strip
}

/**
  * @brief Update the PA8 LED strip in DRL mode (100% white brightness)
  * @param ws: Pointer to the WS28XX handle
  * @retval None
  */
void UpdateDRLMode(WS28XX_HandleTypeDef* ws)
{
    for (int i = 0; i < WS28XX_PIXEL_COUNT; i++) {
//        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Full white light
        WS28XX_SetPixel_RGB_565(ws,  i,  COLOR_RGB565_WHITE);
    }
    WS28XX_Update(ws);  // Refresh the LED strip
}

/**
  * @brief Update the PA8 LED strip with a blinking pattern for startup (amber)
  * @param ws: Pointer to the WS28XX handle
  * @param blink_on: 1 to blink on, 0 to blink off
  * @retval None
  */
void UpdateStartupBlink(WS28XX_HandleTypeDef* ws, int blink_on)
{
    for (int i = 0; i < WS28XX_PIXEL_COUNT; i++) {
        if (blink_on) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_AMBER, 255);  // Amber blinking on
        } else {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);  // LEDs off for blink off
        }
    }
    WS28XX_Update(ws);  // Refresh the LED strip
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