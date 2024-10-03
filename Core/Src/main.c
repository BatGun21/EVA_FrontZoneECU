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
    STARTUP_MODE // Startup blinking mode
} LED_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WS28XX_LEFT_LED_COUNT 28   // Left strip on PA8 with 13 LEDs
#define WS28XX_RIGHT_LED_COUNT 28  // Right strip on PA10 with 13 LEDs

#define WAVE_PACKET_SIZE 7         // Number of LEDs in one wave packet
#define WAVE_SPEED 20              // Delay between each wave step in ms
#define WAVE_STEP_SIZE 1           // Number of pixels the wave moves per frame
#define WAVE_PAUSE 100             // Pause between waves

#define HAZARD_BLINK_DELAY 500    // 500ms for blink in startup
#define DRL_BRIGHTNESS 255         // Full white brightness for DRL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws_pa8;    // Left strip (PA8)
WS28XX_HandleTypeDef ws_pa10;   // Right strip (PA10)

int frame_pa8 = 0;              // Frame counter for wave effect on PA8
int frame_pa10 = 0;             // Frame counter for wave effect on PA10
LED_Mode current_mode_pa8 = STARTUP_MODE;  // Default mode for PA8 is TURN_SIGNAL_MODE
LED_Mode current_mode_pa10 = STARTUP_MODE; // Default mode for PA10 is TURN_SIGNAL_MODE

int wave_count = 0;             // Wave count for startup sequence
int drl_wave_complete = 0;  // Variable to track if the DRL wave transition is complete
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count);  // Wave effect function for PA8 and PA10
void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count);                // DRL mode (white light)
void UpdateHazardBlink(int led_count);
void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count);
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
  WS28XX_Init(&ws_pa8, &htim1, 36, TIM_CHANNEL_1, WS28XX_LEFT_LED_COUNT);   // Initialize for PA8
  WS28XX_Init(&ws_pa10, &htim1, 36, TIM_CHANNEL_3, WS28XX_RIGHT_LED_COUNT); // Initialize for PA10
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // Switch-case for left LED strip (PA8)
	    switch (current_mode_pa8)
	    {
	        case TURN_SIGNAL_MODE:
	            // Call the wave effect for the left strip (PA8)
	            UpdateWaveEffect(&ws_pa8, frame_pa8, WS28XX_LEFT_LED_COUNT);
	            frame_pa8 += WAVE_STEP_SIZE;
	            if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) frame_pa8 = 0;
	            HAL_Delay(WAVE_SPEED);
	            break;

	        case HAZARD_LIGHT_MODE:
	            // Call the blink effect for both strips in sync (hazard lights)
	            UpdateHazardBlink(WS28XX_LEFT_LED_COUNT);  // Blink left strip (PA8)
	            HAL_Delay(HAZARD_BLINK_DELAY);  // Delay for the blink effect
	            break;

	        case STARTUP_MODE:
	            if (wave_count < 5) {
	                // First, run the Amber wave effect for 5 cycles
	                UpdateWaveEffect(&ws_pa8, frame_pa8, WS28XX_LEFT_LED_COUNT);  // Amber wave on left strip
	                UpdateWaveEffect(&ws_pa10, frame_pa10, WS28XX_RIGHT_LED_COUNT);  // Amber wave on right strip
	                frame_pa8 += WAVE_STEP_SIZE;
	                frame_pa10 += WAVE_STEP_SIZE;

	                if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) {
	                    frame_pa8 = 0;
	                    frame_pa10 = 0;
	                    wave_count++;  // Increment wave count after each full cycle
	                }
	                HAL_Delay(WAVE_SPEED);
	            } else if (!drl_wave_complete) {
	                // Transition to DRL with a wave-like effect
	                for (int i = 0; i <= frame_pa8; i++) {
	                    WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Light up DRL white on left strip
	                    WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS); // Light up DRL white on right strip
	                }
	                WS28XX_Update(&ws_pa8);
	                WS28XX_Update(&ws_pa10);

	                frame_pa8 += WAVE_STEP_SIZE;
	                frame_pa10 += WAVE_STEP_SIZE;

	                // Once the entire strip is lit, set DRL as complete
	                if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) {
	                    drl_wave_complete = 1;  // Mark the DRL transition as complete
	                }
	                HAL_Delay(WAVE_SPEED);
	            } else {
	                // Switch to full DRL mode once the transition is complete
	                current_mode_pa8 = DRL_MODE;
	                current_mode_pa10 = DRL_MODE;
	                wave_count = 0;  // Reset wave count for future use
	                drl_wave_complete = 0;  // Reset DRL wave completion flag
	            }
	            break;



	        case DRL_MODE:
	        default:
	        {
	            // Avoid constant updates for DRL mode; refresh periodically
	            static int drl_last_update_time = 0;
	            int drl_current_time = HAL_GetTick();

	            // Update only if more than 100ms has passed
	            if (drl_current_time - drl_last_update_time >= 10000) {
	                UpdateDRLMode(&ws_pa8, WS28XX_LEFT_LED_COUNT);  // Left strip DRL update
	                UpdateDRLMode(&ws_pa10, WS28XX_RIGHT_LED_COUNT); // Right strip DRL update
	                drl_last_update_time = drl_current_time;  // Update last refresh time
	            }
	            break;
	        }
	    }

	    // Switch-case for right LED strip (PA10)
	    switch (current_mode_pa10)
	    {
	        case TURN_SIGNAL_MODE:
	            // Call the wave effect for the right strip (PA10)
	            UpdateWaveEffect(&ws_pa10, frame_pa10, WS28XX_RIGHT_LED_COUNT);
	            frame_pa10 += WAVE_STEP_SIZE;
	            if (frame_pa10 >= WS28XX_RIGHT_LED_COUNT) frame_pa10 = 0;
	            HAL_Delay(WAVE_SPEED);
	            break;

	        case HAZARD_LIGHT_MODE:
	            // Already handled in the PA8 case to ensure synchronization
	            break;

	        case STARTUP_MODE:
	            // Already handled in the PA8 case for synchronized wave effect
	            break;

	        case DRL_MODE:
	        default:
	        {
	            // Avoid constant updates for DRL mode; refresh periodically
	            static int drl_last_update_time = 0;
	            int drl_current_time = HAL_GetTick();

	            // Update only if more than 100ms has passed
	            if (drl_current_time - drl_last_update_time >= 10000) {
	                UpdateDRLMode(&ws_pa8, WS28XX_LEFT_LED_COUNT);  // Left strip DRL update
	                UpdateDRLMode(&ws_pa10, WS28XX_RIGHT_LED_COUNT); // Right strip DRL update
	                drl_last_update_time = drl_current_time;  // Update last refresh time
	            }
	            break;
	        }
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
  * @brief Update the LED strip with a wave effect (turn signal / hazard light)
  * @param ws: Pointer to the WS28XX handle
  * @param frame: Current frame number for the wave effect
  * @param pixel_count: Number of pixels in the strip
  * @retval None
  */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count)
{
    ResetLEDStrip(ws, pixel_count);  // Ensure all LEDs are reset to black before updating

    for (int i = 0; i < pixel_count; i++) {
        if (i >= frame && i < frame + WAVE_PACKET_SIZE) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_AMBER, 255);  // Amber color
        } else {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);    // Turn off (black)
        }
    }
    WS28XX_Update(ws);  // Refresh the LED strip
}

/**
  * @brief Update the LED strip in DRL mode (100% white brightness)
  * @param ws: Pointer to the WS28XX handle
  * @param pixel_count: Number of pixels in the strip
  * @retval None
  */
void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count)
{
    static int last_update_time = 0;
    int current_time = HAL_GetTick();  // Get the current system time in milliseconds

    // Update only every 50ms to avoid flickering
    if (current_time - last_update_time >= 50) {
        ResetLEDStrip(ws, pixel_count);  // Ensure no residual data on the strip

        for (int i = 0; i < pixel_count; i++) {
            WS28XX_SetPixel_RGB_565(ws, i, COLOR_RGB565_WHITE);  // Full white light
        }
        WS28XX_Update(ws);  // Refresh the LED strip

        last_update_time = current_time;  // Update the last time we refreshed
    }
}


/**
  * @brief Blink both LED strips for hazard mode
  * @param led_count: Number of LEDs in each strip
  * @retval None
  */
void UpdateHazardBlink(int led_count)
{
    static int blink_on = 0;  // Track blink state (on/off)

    for (int i = 0; i < led_count; i++) {
        if (blink_on) {
            WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_AMBER, 255);  // Amber on for left strip
            WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_AMBER, 255); // Amber on for right strip
        } else {
            WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_BLACK, 0);    // Off for left strip
            WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_BLACK, 0);   // Off for right strip
        }
    }

    blink_on = !blink_on;  // Toggle the blink state
    WS28XX_Update(&ws_pa8);  // Refresh left strip
    WS28XX_Update(&ws_pa10); // Refresh right strip
}

/**
  * @brief Reset all LEDs in the strip to black before updating
  * @param ws: Pointer to the WS28XX handle
  * @param pixel_count: Number of pixels in the strip
  * @retval None
  */
void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count)
{
    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);  // Turn off all LEDs
    }
    WS28XX_Update(ws);  // Refresh the strip
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
