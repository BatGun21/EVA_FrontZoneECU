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
    STARTUP_MODE, // Startup blinking mode
	CHARGING_MODE
} LED_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WS28XX_LEFT_LED_COUNT 13   // Left strip on PA8 with 13
#define WS28XX_MIDDLE_LED_COUNT 15   // Middle strip on PA8 with 15 LEDs
#define WS28XX_RIGHT_LED_COUNT 13  // Right strip on PA10 with 13 LEDs

#define WAVE_PACKET_SIZE 5        // Number of LEDs in one wave packet
#define WAVE_SPEED 20              // Delay between each wave step in ms
#define WAVE_STEP_SIZE 1           // Number of pixels the wave moves per frame
#define WAVE_PAUSE 100             // Pause between waves

#define HAZARD_BLINK_DELAY 500    // 500ms for blink in startup
#define DRL_BRIGHTNESS 155         // Full white brightness for DRL

#define WAVE_PACKET_SIZE_MIDDLE 1  // Number of LEDs in one wave packet for middle strip (PA9)
#define MIDDLE_LED_MID_INDEX (WS28XX_MIDDLE_LED_COUNT / 2)  // Midpoint of the middle strip
#define CHARGING_COLOR_AC 0x07FF  // Cyan for AC charging
#define CHARGING_COLOR_DC 0x2444  // Forest Green for DC charging
#define AC_CHARGING 1
#define DC_CHARGING 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws_pa8;    // Left strip (PA8)
WS28XX_HandleTypeDef ws_pa9;    // Middle strip (PA9)
WS28XX_HandleTypeDef ws_pa10;   // Right strip (PA10)

int frame_pa8 = 0;
int frame_pa10 = 0;
int frame_pa9 = 0;
LED_Mode current_mode_pa8 = TURN_SIGNAL_MODE;  // Default mode for PA8 is DRL_MODE
LED_Mode current_mode_pa9 = STARTUP_MODE;  // Default mode for PA9 is DRL_MODE
LED_Mode current_mode_pa10 = STARTUP_MODE; // Default mode for PA10 is DRL_MODE

int wave_count = 0;             // Wave count for startup sequence
int drl_wave_complete = 0;      // Variable to track if the DRL wave transition is complete
int soc_percentage = 50;        // Example SOC value for charging mode (can be updated dynamically)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count);  // Wave effect function for PA8 and PA10
void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count);                // DRL mode (white light)
void UpdateHazardBlink(int led_count);
void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count);
void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws);
void UpdateSOCIndication(WS28XX_HandleTypeDef* ws, int soc_percentage, int charging_type);
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
  WS28XX_Init(&ws_pa9, &htim1, 36, TIM_CHANNEL_2, WS28XX_MIDDLE_LED_COUNT); // Initialize for PA9 (middle strip)
  WS28XX_Init(&ws_pa10, &htim1, 36, TIM_CHANNEL_3, WS28XX_RIGHT_LED_COUNT); // Initialize for PA10
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Switch-case for middle LED strip (PA9)
	  switch (current_mode_pa9)
	  {
	      case DRL_MODE:
	          {
	              static int drl_last_update_time_middle = 0;
	              int drl_current_time_middle = HAL_GetTick();

	              // Ensure updates are spaced out to avoid flickering, but react quickly to mode change
	              if (drl_current_time_middle - drl_last_update_time_middle >= 1000) {  // Reduced time to 1 second for faster response
	                  UpdateDRLMode(&ws_pa9, WS28XX_MIDDLE_LED_COUNT);  // Middle strip DRL update
	                  drl_last_update_time_middle = drl_current_time_middle;
	              }
	              break;
	          }

	      case STARTUP_MODE:
	          UpdateStartupWaveForMiddle(&ws_pa9);  // Slower, aesthetic startup animation for PA9
	          break;

	      case CHARGING_MODE:
	          UpdateSOCIndication(&ws_pa9, soc_percentage, DC_CHARGING);  // SOC indication with blinking LEDs
	          break;

	      default:
	          current_mode_pa9 = DRL_MODE;
	          break;
	  }

	  // Switch-case for left LED strip (PA8)
	  switch (current_mode_pa8)
	  {
	      case TURN_SIGNAL_MODE:
	          UpdateWaveEffect(&ws_pa8, frame_pa8, WS28XX_LEFT_LED_COUNT);
	          frame_pa8 += WAVE_STEP_SIZE;
	          if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) frame_pa8 = 0;
	          HAL_Delay(WAVE_SPEED);
	          break;

	      case HAZARD_LIGHT_MODE:
	          UpdateHazardBlink(WS28XX_LEFT_LED_COUNT);  // Sync blink for hazard mode
	          HAL_Delay(HAZARD_BLINK_DELAY);  // Maintain a consistent delay
	          break;

	      case STARTUP_MODE:
	          if (wave_count < 5) {
	              UpdateWaveEffect(&ws_pa8, frame_pa8, WS28XX_LEFT_LED_COUNT);  // Amber wave on left strip
	              UpdateWaveEffect(&ws_pa10, frame_pa10, WS28XX_RIGHT_LED_COUNT);  // Amber wave on right strip
	              frame_pa8 += WAVE_STEP_SIZE;
	              frame_pa10 += WAVE_STEP_SIZE;

	              if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) {
	                  frame_pa8 = 0;
	                  frame_pa10 = 0;
	                  wave_count++;
	              }
	              HAL_Delay(WAVE_SPEED);
	          } else if (!drl_wave_complete) {
	              // Transition to DRL mode with smooth lighting effect
	              for (int i = 0; i <= frame_pa8; i++) {
	                  WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Left strip DRL update
	                  WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Right strip DRL update
	              }
	              WS28XX_Update(&ws_pa8);
	              WS28XX_Update(&ws_pa10);

	              frame_pa8 += WAVE_STEP_SIZE;
	              frame_pa10 += WAVE_STEP_SIZE;

	              if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) {
	                  drl_wave_complete = 1;
	              }
	              HAL_Delay(WAVE_SPEED);
	          } else {
	              current_mode_pa8 = DRL_MODE;
	              current_mode_pa10 = DRL_MODE;
	              wave_count = 0;
	              drl_wave_complete = 0;
	          }
	          break;

	      case DRL_MODE:
	      default:
	      {
	          static int drl_last_update_time = 0;
	          int drl_current_time = HAL_GetTick();

	          // Sync DRL mode update with the same timing as the middle strip
	          if (drl_current_time - drl_last_update_time >= 1000) {  // 1-second update interval
	              UpdateDRLMode(&ws_pa8, WS28XX_LEFT_LED_COUNT);  // Left strip DRL update
	              UpdateDRLMode(&ws_pa10, WS28XX_RIGHT_LED_COUNT); // Right strip DRL update
	              drl_last_update_time = drl_current_time;
	          }
	          break;
	      }
	  }

	  // Switch-case for right LED strip (PA10)
	  switch (current_mode_pa10)
	  {
	      case TURN_SIGNAL_MODE:
	          UpdateWaveEffect(&ws_pa10, frame_pa10, WS28XX_RIGHT_LED_COUNT);
	          frame_pa10 += WAVE_STEP_SIZE;
	          if (frame_pa10 >= WS28XX_RIGHT_LED_COUNT) frame_pa10 = 0;
	          HAL_Delay(WAVE_SPEED);
	          break;

	      case HAZARD_LIGHT_MODE:
	          // Sync with PA8's hazard light mode
	          break;

	      case STARTUP_MODE:
	          // Sync with PA8's startup mode
	          break;

	      case DRL_MODE:
	      default:
	      {
	          static int drl_last_update_time = 0;
	          int drl_current_time = HAL_GetTick();

	          if (drl_current_time - drl_last_update_time >= 1000) {  // 1-second update interval
	              UpdateDRLMode(&ws_pa8, WS28XX_LEFT_LED_COUNT);  // Left strip DRL update
	              UpdateDRLMode(&ws_pa10, WS28XX_RIGHT_LED_COUNT); // Right strip DRL update
	              drl_last_update_time = drl_current_time;
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
            WS28XX_SetPixel_RGB_565(&ws_pa8, i, COLOR_RGB565_AMBER);  // Amber on for left strip
            WS28XX_SetPixel_RGB_565(&ws_pa10, i, COLOR_RGB565_AMBER); // Amber on for right strip
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

void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws)
{
    static int wave_count_middle = 0;
    static int frame_left = MIDDLE_LED_MID_INDEX;  // Start from the center
    static int frame_right = MIDDLE_LED_MID_INDEX;
    static uint8_t wave_direction = 0;  // 0 for outward, 1 for inward
    static int drl_wave_complete_middle = 0;  // Variable to track DRL transition

    if (wave_count_middle < 4) {
        // We want 2 full passes (each pass consists of outward and inward wave)
        ResetLEDStrip(ws, WS28XX_MIDDLE_LED_COUNT);  // Turn off LEDs to create wave movement effect

        if (wave_direction == 0) {  // Wave moving outward
            for (int i = 0; i < WAVE_PACKET_SIZE_MIDDLE; i++) {
                if (frame_left - i >= 0) {
                    WS28XX_SetPixel_RGBW_565(ws, frame_left - i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Left side
                }
                if (frame_right + i < WS28XX_MIDDLE_LED_COUNT) {
                    WS28XX_SetPixel_RGBW_565(ws, frame_right + i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Right side
                }
            }
            WS28XX_Update(ws);

            frame_left--;
            frame_right++;

            if (frame_left < 0 && frame_right >= WS28XX_MIDDLE_LED_COUNT) {
                wave_direction = 1;  // Reverse direction when it hits the edges
            }

        } else if (wave_direction == 1) {  // Wave moving inward
            for (int i = 0; i < WAVE_PACKET_SIZE_MIDDLE; i++) {
                if (frame_left + i < MIDDLE_LED_MID_INDEX) {
                    WS28XX_SetPixel_RGBW_565(ws, frame_left + i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Left side returning
                }
                if (frame_right - i >= MIDDLE_LED_MID_INDEX) {
                    WS28XX_SetPixel_RGBW_565(ws, frame_right - i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Right side returning
                }
            }
            WS28XX_Update(ws);

            frame_left++;
            frame_right--;

            if (frame_left >= MIDDLE_LED_MID_INDEX && frame_right <= MIDDLE_LED_MID_INDEX) {
                frame_left = MIDDLE_LED_MID_INDEX;
                frame_right = MIDDLE_LED_MID_INDEX;
                wave_direction = 0;  // Reset for next outward pass
                wave_count_middle++;  // Complete one full pass
            }
        }

        HAL_Delay(WAVE_SPEED * 2);  // Slower speed for a luxury aesthetic

    } else if (!drl_wave_complete_middle) {
        // Transition to DRL mode with a wave-like effect after two full wave passes
        for (int i = 0; i <= frame_left; i++) {
            WS28XX_SetPixel_RGBW_565(ws, MIDDLE_LED_MID_INDEX - i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Light from center outward to left
            WS28XX_SetPixel_RGBW_565(ws, MIDDLE_LED_MID_INDEX + i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Light from center outward to right
        }
        WS28XX_Update(ws);

        frame_left++;
        if (frame_left >= MIDDLE_LED_MID_INDEX) {
            drl_wave_complete_middle = 1;  // DRL transition complete
        }

        HAL_Delay(WAVE_SPEED);  // Smooth transition speed

    } else {
        // Finally, settle into DRL mode with full brightness
        for (int i = 0; i < WS28XX_MIDDLE_LED_COUNT; i++) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);  // Full DRL brightness
        }
        WS28XX_Update(ws);

        current_mode_pa9 = DRL_MODE;  // Switch to DRL mode
        wave_count_middle = 0;  // Reset wave count for future use
        drl_wave_complete_middle = 0;  // Reset DRL wave completion flag
    }
}


void UpdateSOCIndication(WS28XX_HandleTypeDef* ws, int soc_percentage, int charging_type) {
    static int last_update_time = 0;
    static uint8_t charging_blink_state = 0;  // Blink state, 0 = off, 1 = on
    int current_time = HAL_GetTick();

    // Update every 100ms to avoid flickering
    if (current_time - last_update_time >= 1000) {
        last_update_time = current_time;

        int led_count_to_light = (WS28XX_MIDDLE_LED_COUNT * soc_percentage) / 100;
        int left_led = MIDDLE_LED_MID_INDEX;
        int right_led = MIDDLE_LED_MID_INDEX;

        for (int i = 0; i < led_count_to_light / 2; i++) {
            if (left_led - i >= 0) {
                WS28XX_SetPixel_RGBW_565(ws, left_led - i, (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, DRL_BRIGHTNESS);
            }
            if (right_led + i < WS28XX_MIDDLE_LED_COUNT) {
                WS28XX_SetPixel_RGBW_565(ws, right_led + i, (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, DRL_BRIGHTNESS);
            }
        }

        // Blink the next LED for charging animation
        charging_blink_state = !charging_blink_state;  // Toggle the blink state
        uint8_t blink_brightness = charging_blink_state ? DRL_BRIGHTNESS : 0;

        if (left_led - (led_count_to_light / 2) >= 0) {
            WS28XX_SetPixel_RGBW_565(ws, left_led - (led_count_to_light / 2), (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, blink_brightness);
        }
        if (right_led + (led_count_to_light / 2) < WS28XX_MIDDLE_LED_COUNT) {
            WS28XX_SetPixel_RGBW_565(ws, right_led + (led_count_to_light / 2), (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, blink_brightness);
        }

        WS28XX_Update(ws);  // Refresh the LED strip
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
