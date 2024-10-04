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
	CHARGING_MODE //Charging Mode for middle strip
} LED_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCK_FREQ_MHZ 36
#define WS28XX_LEFT_LED_COUNT 28   // Left strip on PA8
#define WS28XX_RIGHT_LED_COUNT 28  // Right strip on PA10
#define WS28XX_MIDDLE_LED_COUNT 28 // Middle strip on PA9
#define WAVE_SPEED 20              // Wave speed in ms
#define WAVE_STEP_SIZE 1           // Number of pixels per wave step
#define WAVE_PACKET_SIZE 7         // Number of Pixels per wave
#define WAVE_PACKET_SIZE_MIDDLE 3  // Number of LEDs in one wave packet for middle strip
#define HAZARD_BLINK_DELAY 500     // Blink delay for hazard
#define DRL_BRIGHTNESS 255         // Full white brightness for DRL
#define MIDDLE_LED_MID_INDEX (WS28XX_MIDDLE_LED_COUNT / 2)
#define AC_CHARGING 1
#define DC_CHARGING 2
#define CHARGING_COLOR_AC 0x07FF   // Cyan for AC charging
#define CHARGING_COLOR_DC 0x2444   // Forest Green for DC charging

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef *ws_pa8 = NULL;
WS28XX_HandleTypeDef *ws_pa9 = NULL;
WS28XX_HandleTypeDef *ws_pa10 = NULL;

int frame_pa8 = 0;
int frame_pa9 = 0;
int frame_pa10 = 0;
LED_Mode current_mode_pa8 = STARTUP_MODE;
LED_Mode current_mode_pa9 = STARTUP_MODE;
LED_Mode current_mode_pa10 = STARTUP_MODE;

int wave_count = 0;
int drl_wave_complete = 0;
int soc_percentage = 50;  // Example SOC value for charging mode
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count);
void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count);
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

  // Memory Allocation
  ws_pa8 = (WS28XX_HandleTypeDef *)malloc(sizeof(WS28XX_HandleTypeDef));
  ws_pa9 = (WS28XX_HandleTypeDef *)malloc(sizeof(WS28XX_HandleTypeDef));
  ws_pa10 = (WS28XX_HandleTypeDef *)malloc(sizeof(WS28XX_HandleTypeDef));

  WS28XX_Init(ws_pa8, &htim1, CLOCK_FREQ_MHZ, TIM_CHANNEL_1, WS28XX_LEFT_LED_COUNT);   // PA8
  WS28XX_Init(ws_pa9, &htim1, CLOCK_FREQ_MHZ, TIM_CHANNEL_2, WS28XX_MIDDLE_LED_COUNT); // PA9
  WS28XX_Init(ws_pa10, &htim1, CLOCK_FREQ_MHZ, TIM_CHANNEL_3, WS28XX_RIGHT_LED_COUNT); // PA10

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   // Middle Strip (PA9) Logic
	        switch (current_mode_pa9)
	        {
	            case DRL_MODE:
	                UpdateDRLMode(ws_pa9, WS28XX_MIDDLE_LED_COUNT);
	                break;

	            case STARTUP_MODE:
	                UpdateStartupWaveForMiddle(ws_pa9);
	                break;

	            case CHARGING_MODE:
	                UpdateSOCIndication(ws_pa9, soc_percentage, AC_CHARGING);
	                break;

	            default:
	                current_mode_pa9 = DRL_MODE;
	                break;
	        }

	        // Left Strip (PA8) Logic
	        switch (current_mode_pa8)
	        {
	            case TURN_SIGNAL_MODE:
	                UpdateWaveEffect(ws_pa8, frame_pa8, WS28XX_LEFT_LED_COUNT);
	                frame_pa8 += WAVE_STEP_SIZE;
	                if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) frame_pa8 = 0;
	                HAL_Delay(WAVE_SPEED);
	                break;

	            case HAZARD_LIGHT_MODE:
	                UpdateHazardBlink(WS28XX_LEFT_LED_COUNT);
	                HAL_Delay(HAZARD_BLINK_DELAY);
	                break;

	            case STARTUP_MODE:
	                if (wave_count < 5) {
	                    UpdateWaveEffect(ws_pa8, frame_pa8, WS28XX_LEFT_LED_COUNT);
	                    UpdateWaveEffect(ws_pa10, frame_pa10, WS28XX_RIGHT_LED_COUNT);
	                    frame_pa8 += WAVE_STEP_SIZE;
	                    frame_pa10 += WAVE_STEP_SIZE;

	                    if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) {
	                        frame_pa8 = 0;
	                        frame_pa10 = 0;
	                        wave_count++;
	                    }
	                    HAL_Delay(WAVE_SPEED);
	                } else if (!drl_wave_complete) {
	                    for (int i = 0; i <= frame_pa8; i++) {
	                        WS28XX_SetPixel_RGBW_565(ws_pa8, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
	                        WS28XX_SetPixel_RGBW_565(ws_pa10, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
	                    }
	                    WS28XX_Update(ws_pa8);
	                    WS28XX_Update(ws_pa10);

	                    frame_pa8 += WAVE_STEP_SIZE;
	                    frame_pa10 += WAVE_STEP_SIZE;

	                    if (frame_pa8 >= WS28XX_LEFT_LED_COUNT) drl_wave_complete = 1;
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
	                static int drl_last_update_time = 0;
	                int drl_current_time = HAL_GetTick();

	                if (drl_current_time - drl_last_update_time >= 10000) {
	                    UpdateDRLMode(ws_pa8, WS28XX_LEFT_LED_COUNT);
	                    UpdateDRLMode(ws_pa10, WS28XX_RIGHT_LED_COUNT);
	                    drl_last_update_time = drl_current_time;
	                }
	                break;
	        }

	        // Right Strip (PA10) Logic
	        switch (current_mode_pa10)
	        {
	            case TURN_SIGNAL_MODE:
	                UpdateWaveEffect(ws_pa10, frame_pa10, WS28XX_RIGHT_LED_COUNT);
	                frame_pa10 += WAVE_STEP_SIZE;
	                if (frame_pa10 >= WS28XX_RIGHT_LED_COUNT) frame_pa10 = 0;
	                HAL_Delay(WAVE_SPEED);
	                break;

	            case HAZARD_LIGHT_MODE:
	            case STARTUP_MODE:
	                break;  // Already handled by PA8 for synchronization

	            case DRL_MODE:
	            default:
	                break;  // DRL logic already handled
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

void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count) {
    ResetLEDStrip(ws, pixel_count);
    for (int i = 0; i < pixel_count; i++) {
        if (i >= frame && i < frame + WAVE_PACKET_SIZE) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_AMBER, 255);
        } else {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);
        }
    }
    WS28XX_Update(ws);
}

void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count) {
    ResetLEDStrip(ws, pixel_count);
    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGB_565(ws, i, COLOR_RGB565_WHITE);
    }
    WS28XX_Update(ws);
}

void UpdateHazardBlink(int led_count) {
    static int blink_on = 0;
    for (int i = 0; i < led_count; i++) {
        if (blink_on) {
            WS28XX_SetPixel_RGBW_565(ws_pa8, i, COLOR_RGB565_AMBER, 255);
            WS28XX_SetPixel_RGBW_565(ws_pa10, i, COLOR_RGB565_AMBER, 255);
        } else {
            WS28XX_SetPixel_RGBW_565(ws_pa8, i, COLOR_RGB565_BLACK, 0);
            WS28XX_SetPixel_RGBW_565(ws_pa10, i, COLOR_RGB565_BLACK, 0);
        }
    }
    blink_on = !blink_on;
    WS28XX_Update(ws_pa8);
    WS28XX_Update(ws_pa10);
}

void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count) {
    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);
    }
    WS28XX_Update(ws);
}

void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws) {
    static int wave_count_middle = 0;
    static int frame_left = MIDDLE_LED_MID_INDEX;
    static int frame_right = MIDDLE_LED_MID_INDEX;

    if (wave_count_middle < 2) {
        for (int i = 0; i < WAVE_PACKET_SIZE_MIDDLE; i++) {
            if (frame_left - i >= 0) {
                WS28XX_SetPixel_RGBW_565(ws, frame_left - i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
            }
            if (frame_right + i < WS28XX_MIDDLE_LED_COUNT) {
                WS28XX_SetPixel_RGBW_565(ws, frame_right + i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
            }
        }
        WS28XX_Update(ws);
        frame_left--;
        frame_right++;

        if (frame_left < 0 && frame_right >= WS28XX_MIDDLE_LED_COUNT) {
            frame_left = MIDDLE_LED_MID_INDEX;
            frame_right = MIDDLE_LED_MID_INDEX;
            wave_count_middle++;
        }
    } else {
        for (int i = 0; i < WS28XX_MIDDLE_LED_COUNT; i++) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
        }
        WS28XX_Update(ws);
        current_mode_pa9 = DRL_MODE;
        wave_count_middle = 0;
    }
}

void UpdateSOCIndication(WS28XX_HandleTypeDef* ws, int soc_percentage, int charging_type) {
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
    WS28XX_Update(ws);
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
