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
  * If no LICENSE file comes witstmh this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws28xx.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Enum to define different LED states
typedef enum {
    DRL_MODE,          // Daytime running lights
    TURN_SIGNAL_MODE,  // Turn signal active
    HAZARD_LIGHT_MODE, // Hazard/parking light active
    STARTUP_MODE,      // Startup blinking mode
	CHARGING_MODE,
	BREATHING_MODE,
	OFF_MODE
} LED_Mode;

typedef enum {
    HORN_OFF,
    HORN_ON
} HornState;

typedef enum {
    HEADLAMP_OFF,
    HEADLAMP_LOW_BEAM,
    HEADLAMP_HIGH_BEAM
} HeadlampState;

typedef struct {
    uint8_t stable_value;
    uint8_t last_value;
    uint32_t last_debounce_time;
    uint8_t sample_count;
} DebounceState;


enum pinstate { no_change = 0, rising_edge = 1, falling_edge = 2};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Clock_Frequency 36000 //KHz
#define WS28XX_LEFT_LED_COUNT 25
#define WS28XX_MIDDLE_LED_COUNT 152
#define WS28XX_RIGHT_LED_COUNT 25

#define WAVE_PACKET_SIZE 9
#define WAVE_SPEED 5 // Increase number to reduce speed
#define WAVE_STEP_SIZE 1
#define WAVE_PAUSE 10

#define HAZARD_BLINK_DELAY 500
#define DRL_BRIGHTNESS 200

#define WAVE_PACKET_SIZE_MIDDLE 4
#define STARTUP_WAVE_SPEED 1 // Increase number to reduce speed
#define MIDDLE_LED_MID_INDEX (WS28XX_MIDDLE_LED_COUNT / 2)
#define CHARGING_COLOR_AC 0x07FF
#define CHARGING_COLOR_DC 0x2444

#define AC_CHARGING 1
#define DC_CHARGING 2

#define BREATHING_DURATION 1

#define BAUD_RATE 9600
#define debounceDelay 10
#define debounceSamplesCount 5
#define DRL_REFRESH_TIME 1000
#define DEBOUNCE_DELAY 50 // 50ms debounce delay
#define DEBOUNCE_SAMPLES 5 // Minimum samples required for stable state
// Define Test_Mode to run in the test mode, this tests if the software is able to run the hardware in different states.
//#define TEST_MODE 1
#ifdef TEST_MODE
#define TOGGLE_INTERVAL 10000         // 10 seconds for state transitions
#define HORN_ON_DURATION 300          // Horn should be on for 300 ms
#endif

// Define TEST_MODE for testing state machines
//#define TEST_MODE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws_pa8;
WS28XX_HandleTypeDef ws_pa9;
WS28XX_HandleTypeDef ws_pa10;

LED_Mode current_mode_pa8 = DRL_MODE;
LED_Mode current_mode_pa9 = DRL_MODE;
LED_Mode current_mode_pa10 = DRL_MODE;

HornState current_horn_state = HORN_OFF;
HeadlampState current_headlamp_state = HEADLAMP_OFF;

int frame_pa8 = 0, frame_pa9 = 0, frame_pa10 = 0;
int wave_count = 0;
int drl_wave_complete_middle = 0;  // Flag to indicate when the middle strip completes DRL wave
int is_drl_displayed_pa8 = 0;      // Flag for DRL display status for left strip
int is_drl_displayed_pa10 = 0;     // Flag for DRL display status for right strip
int is_drl_displayed_pa9 = 0;      // Flag for middle strip to avoid flickering

int soc_percentage = 40;  // Example value

// Control signals
int charging_signal_received = 0, drl_signal_received = 0, hazard_signal_received = 0, startup_signal_received = 1;
int turn_signal_left_received = 0, turn_signal_right_received = 0;
int horn_signal_received = 0, headlamp_low_beam_signal_received = 0, headlamp_high_beam_signal_received = 0;
int breathing_signal_received = 0;  // Breathing mode control

//uint8_t UART_received_byte = 0;
uint16_t adc_val_pa3, adc_val_pa2, adc_val_pa4;
float voltage_pa3, voltage_pa2, voltage_pa4;
const float VREF = 3.3;

static DebounceState brake_state = {1, 1, 0, 0}; // Because this signal is active low
static DebounceState horn_state = {1, 1, 0, 0};  // Because this signal is active low
static DebounceState reverse_state = {0, 0, 0, 0};
// Global debounce state for steering flags
static DebounceState turn_signal_left_flag = {0};
static DebounceState turn_signal_right_flag = {0};
static DebounceState headlamp_low_beam_flag = {0};
static DebounceState headlamp_high_beam_flag = {0};
static DebounceState drl_flag = {0};

uint8_t turn_signal_left_raw = 0;
uint8_t turn_signal_right_raw = 0;
uint8_t headlamp_low_raw = 0;
uint8_t headlamp_high_raw = 0;
uint8_t drl_signal_received_raw = 0;

#ifdef TEST_MODE
/* Variables for Test Mode */
uint32_t last_time_middle_strip = 0;  // Timing for middle strip
uint32_t last_time_left_strip = 0;    // Timing for left side strip
uint32_t last_time_right_strip = 0;   // Timing for right side strip
uint32_t last_time_side_strip = 0;
uint32_t last_time_horn = 0;          // Timing for horn
uint32_t last_time_headlamp = 0;      // Timing for headlamp
uint32_t current_time = 0;            // Horn should be on for 300 ms
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count, int direction);
void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_drl_displayed_flag);
void UpdateHazardBlink(int led_count);
void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count);
void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws);
void UpdateSOCIndication(WS28XX_HandleTypeDef* ws, int soc_percentage, int charging_type);
void HandleMiddleStripState(void);
void HandleLeftStripState(void);
void HandleRightStripState(void);
void HandleHornState(void);
void HandleHeadlampState(void);
void SetGPIOHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void SetGPIOLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_Init_PA0_PA1_PA6(void);
void UART_Init(void);
void UART_Send(uint8_t data);
bool UART_Receive(uint8_t *data, uint32_t timeout);
void ProcessUARTMessage(uint8_t received_byte);
void UART_ReceiveAndProcess(void);
void BreathingEffect(WS28XX_HandleTypeDef* ws, int duration);
void ADC_Select_CH3 (void);
void ADC_Select_CH4 (void);
void ADC_Select_CH2 (void);
void readSteeringControls(void);
void readBrakeSwitch(void);
void readReverseGear(void);
void readHorn(void);
void Init_InputPins(void);
void InitFastOutputPins(void);
void ADC_Select_CH5(void);
void ADC_Select_CH7(void);
void readHazardSwitch(void);
void DebounceFlag(DebounceState* flag_state, uint8_t raw_value);
uint8_t DetectEdge(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t debounce_delay);
int DebounceHazardPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t debounce_delay);
void DebounceInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, DebounceState* state);
#ifdef TEST_MODE
void Handle_TestMode(void);
#endif

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
  MX_ADC1_Init();
  GPIO_Init_PA0_PA1_PA6();
  InitFastOutputPins();
  /* USER CODE BEGIN 2 */
  WS28XX_Init(&ws_pa8, &htim1, 36, TIM_CHANNEL_1, WS28XX_LEFT_LED_COUNT);   // Initialize for PA8
  WS28XX_Init(&ws_pa9, &htim1, 36, TIM_CHANNEL_2, WS28XX_MIDDLE_LED_COUNT); // Initialize for PA9 (middle strip)
  WS28XX_Init(&ws_pa10, &htim1, 36, TIM_CHANNEL_3, WS28XX_RIGHT_LED_COUNT); // Initialize for PA10
  SetGPIOHigh(GPIOA, GPIO_PIN_0);  //  Horn Off
  SetGPIOHigh(GPIOA, GPIO_PIN_1);  //  Head Lamp off
  SetGPIOHigh(GPIOA, GPIO_PIN_6);  //  in Low Beam Mode
  Init_InputPins();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	// ========================
	// Handle UART Communication
	// ========================
       // UART_ReceiveAndProcess();

	// ========================
	// Handle Inputs and Update Flags
	// ========================
	readBrakeSwitch();
	readSteeringControls();
	readHorn();
	readReverseGear();
	readHazardSwitch();

	#ifdef TEST_MODE
	    Handle_TestMode();  // Call the test mode handler
	#endif
	// ========================
	// Handle State Machines
	// ========================
	  HandleMiddleStripState();
	  HandleLeftStripState();
	  HandleRightStripState();
	  HandleHornState();
	  HandleHeadlampState();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */
void Init_InputPins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clock for GPIO ports
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // PB12 (Horn), PB13 (Hazard), PB3 (Brake fluid): Floating if inactive, pulled down when active
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB15 (Brake) (3.3V when inactive, GND when brake switch pressed)
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PA12 (Reverse): Pulled up when active
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // High-speed configuration
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void InitFastOutputPins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable the GPIOB clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure PB4 to PB9 as fast outputs with pull-down resistors
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull mode
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;       // Enable pull-down resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High-speed configuration
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Set all pins to their default inactive state (low)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);




    // Configure PB4 to PB9 as fast outputs with pull-down resistors
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High-speed configuration
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Driving relay, inactive state is GPIO pin set
}

void GPIO_Init_PA0_PA1_PA6(void)
{
    // Enable GPIOA clock (assuming it's AHB1 on your microcontroller)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable clock for GPIOA

    // Configure PA0, PA1, PA6 as output, push-pull, high-speed

    // PA0 Configuration
    GPIOA->CRL &= ~GPIO_CRL_CNF0;   // Clear CNF0[1:0] (set as push-pull)
    GPIOA->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0;  // Set MODE0 to 11 (high-speed output 50 MHz)
    GPIOA->ODR |= GPIO_ODR_ODR0;  // Set PA0 high


    // PA1 Configuration
    GPIOA->CRL &= ~GPIO_CRL_CNF1;   // Clear CNF1[1:0] (set as push-pull)
    GPIOA->CRL |= GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0;  // Set MODE1 to 11 (high-speed output 50 MHz)

    // PA6 Configuration
    GPIOA->CRL &= ~GPIO_CRL_CNF6;   // Clear CNF6[1:0] (set as push-pull)
    GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0;  // Set MODE6 to 11 (high-speed output 50 MHz)

    // Set PA0, PA1, PA6 initially to high (deactivate relays)
    GPIOA->ODR |= GPIO_ODR_ODR0;  // Set PA0 high
    GPIOA->ODR |= GPIO_ODR_ODR1;  // Set PA1 high
    GPIOA->ODR |= GPIO_ODR_ODR6;  // Set PA6 high
}

void HandleMiddleStripState(void) {
	// Adjusted priority code for setting current_mode_pa9
	if (charging_signal_received) {
	    current_mode_pa9 = CHARGING_MODE;  // Highest priority: Charging mode
	} else if (drl_signal_received) {
	    current_mode_pa9 = DRL_MODE;       // Next priority: DRL mode if charging is inactive
	} else if (breathing_signal_received) {
	    current_mode_pa9 = BREATHING_MODE; // Next: Breathing mode if no charging or DRL
	} else if (startup_signal_received){
	    current_mode_pa9 = STARTUP_MODE;   // Default to startup mode if no other signals are active
	}else
	{
		current_mode_pa9 = OFF_MODE;
	}


    switch (current_mode_pa9) {
        case CHARGING_MODE:
            UpdateSOCIndication(&ws_pa9, soc_percentage, DC_CHARGING);
            is_drl_displayed_pa9 = 0;
            break;
        case DRL_MODE:
            UpdateDRLMode(&ws_pa9, WS28XX_MIDDLE_LED_COUNT, &is_drl_displayed_pa9);
            break;
        case BREATHING_MODE:
            BreathingEffect(&ws_pa9, BREATHING_DURATION);  // Custom breathing color
            break;
        case STARTUP_MODE:
            UpdateStartupWaveForMiddle(&ws_pa9);
            is_drl_displayed_pa9 = 0;
            break;
        case OFF_MODE:
            ResetLEDStrip(&ws_pa9, WS28XX_MIDDLE_LED_COUNT);
            HAL_Delay(6);
            HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
            is_drl_displayed_pa9 = 0;
            break;
        default:
        	ResetLEDStrip(&ws_pa9, WS28XX_MIDDLE_LED_COUNT);
        	HAL_Delay(6);
        	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
        	is_drl_displayed_pa9 = 0;
    }
}

void HandleLeftStripState(void) {
    static uint32_t last_blink_time_left = 0;
    uint32_t current_time = HAL_GetTick();
    static uint8_t blink_state_left = 0;

    // Priority management for left strip
    if (charging_signal_received) {
        if (hazard_signal_received) {
            current_mode_pa8 = TURN_SIGNAL_MODE;
        } else {
            current_mode_pa8 = CHARGING_MODE;
        }
    } else if (hazard_signal_received || turn_signal_left_received) {
//        current_mode_pa8 = HAZARD_LIGHT_MODE;
    	current_mode_pa8 = TURN_SIGNAL_MODE;
    } else if (drl_signal_received && !hazard_signal_received && !turn_signal_left_received) {
        current_mode_pa8 = DRL_MODE;
    } else {
        current_mode_pa8 = OFF_MODE;
    }

    // State machine for left strip (PA8)
    switch (current_mode_pa8) {
        case CHARGING_MODE:
            ResetLEDStrip(&ws_pa8, WS28XX_LEFT_LED_COUNT);
            is_drl_displayed_pa8 = 0;
            break;

//        case HAZARD_LIGHT_MODE:
////            if (current_time - last_blink_time_left >= HAZARD_BLINK_DELAY) {
////                blink_state_left = !blink_state_left; // Toggle blink state
////                if (blink_state_left)
////                {
////                    // Set all LEDs to amber
////                    for (int i = 0; i < WS28XX_LEFT_LED_COUNT; i++) {
////                        WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_AMBER, 255);
////                    }
////                    WS28XX_Update(&ws_pa8);
////                } else
////                {
////                    // Reset all LEDs to off
////                    ResetLEDStrip(&ws_pa8, WS28XX_LEFT_LED_COUNT);
////                }
////                last_blink_time_left = current_time; // Update last blink timestamp
////            }
////            is_drl_displayed_pa8 = 0; // Reset DRL flag
//            break;

        case TURN_SIGNAL_MODE:
            if (current_time - last_blink_time_left >= HAZARD_BLINK_DELAY) {
                blink_state_left = !blink_state_left; // Toggle blink state
                if (blink_state_left)
                {
                    // Set all LEDs to amber
                    for (int i = 0; i < WS28XX_LEFT_LED_COUNT; i++) {
                        WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_AMBER, 255);
                    }
                    WS28XX_Update(&ws_pa8);
                } else
                {
                    // Reset all LEDs to off
                    ResetLEDStrip(&ws_pa8, WS28XX_LEFT_LED_COUNT);
                }
                last_blink_time_left = current_time; // Update last blink timestamp
            }
            is_drl_displayed_pa8 = 0; // Reset DRL flag
            break;


        case DRL_MODE:
            frame_pa8 = 0;
            UpdateDRLMode(&ws_pa8, WS28XX_LEFT_LED_COUNT, &is_drl_displayed_pa8);
            break;

        case OFF_MODE:
            frame_pa8 = 0;
            ResetLEDStrip(&ws_pa8, WS28XX_LEFT_LED_COUNT);
            is_drl_displayed_pa8 = 0;
            break;

        default:
            current_mode_pa8 = DRL_MODE;
            break;
    }
}

void HandleRightStripState(void) {
    static uint32_t last_blink_time_right = 0;
    uint32_t current_time = HAL_GetTick();
    static uint8_t blink_state_right = 0;

    // Priority management for right strip
    if (charging_signal_received) {
        if (hazard_signal_received) {
            current_mode_pa10 = TURN_SIGNAL_MODE;
        } else {
            current_mode_pa10 = CHARGING_MODE;
        }
    } else if (hazard_signal_received||turn_signal_right_received) {
//        current_mode_pa10 = HAZARD_LIGHT_MODE;
//    } else if (turn_signal_right_received && !hazard_signal_received) {
        current_mode_pa10 = TURN_SIGNAL_MODE;
    } else if (drl_signal_received && !hazard_signal_received && !turn_signal_right_received) {
        current_mode_pa10 = DRL_MODE;
    } else {
        current_mode_pa10 = OFF_MODE;
    }

    // State machine for right strip (PA10)
    switch (current_mode_pa10) {
        case CHARGING_MODE:
            ResetLEDStrip(&ws_pa10, WS28XX_RIGHT_LED_COUNT);
            is_drl_displayed_pa10 = 0;
            break;

//        case HAZARD_LIGHT_MODE:
//            if (current_time - last_blink_time_right >= HAZARD_BLINK_DELAY) {
//                blink_state_right = !blink_state_right; // Toggle blink state
//                if (blink_state_right)
//                {
//                    // Set all LEDs to amber
//                    for (int i = 0; i < WS28XX_RIGHT_LED_COUNT; i++) {
//                        WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_AMBER, 255);
//                    }
//                    WS28XX_Update(&ws_pa10);
//                } else
//                {
//                    // Reset all LEDs to off
//                    ResetLEDStrip(&ws_pa10, WS28XX_RIGHT_LED_COUNT);
//                }
//                last_blink_time_right = current_time; // Update last blink timestamp
//            }
//            is_drl_displayed_pa10 = 0; // Reset DRL flag
//            break;

        case TURN_SIGNAL_MODE:
            if (current_time - last_blink_time_right >= HAZARD_BLINK_DELAY) {
                blink_state_right = !blink_state_right; // Toggle blink state
                if (blink_state_right)
                {
                    // Set all LEDs to amber
                    for (int i = 0; i < WS28XX_RIGHT_LED_COUNT; i++) {
                        WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_AMBER, 255);
                    }
                    WS28XX_Update(&ws_pa10);
                } else
                {
                    // Reset all LEDs to off
                    ResetLEDStrip(&ws_pa10, WS28XX_RIGHT_LED_COUNT);
                }
                last_blink_time_right = current_time; // Update last blink timestamp
            }
            is_drl_displayed_pa10 = 0; // Reset DRL flag
            break;

        case DRL_MODE:
            frame_pa10 = 0;
            UpdateDRLMode(&ws_pa10, WS28XX_RIGHT_LED_COUNT, &is_drl_displayed_pa10);
            break;

        case OFF_MODE:
            frame_pa10 = 0;
            ResetLEDStrip(&ws_pa10, WS28XX_RIGHT_LED_COUNT);
            is_drl_displayed_pa10 = 0;
            break;

        default:
            current_mode_pa10 = DRL_MODE;
            break;
    }
}

void UpdateWaveEffect(WS28XX_HandleTypeDef* ws, int frame, int pixel_count, int direction)
{
    ResetLEDStrip(ws, pixel_count);

    for (int i = 0; i < pixel_count; i++) {
        int pixel_index = (direction == 1) ? i : (pixel_count - 1 - i); // Adjust index for direction
        if (pixel_index >= frame && pixel_index < frame + WAVE_PACKET_SIZE) {
            WS28XX_SetPixel_RGBW_565(ws, pixel_index, COLOR_RGB565_AMBER, 255);
        } else {
            WS28XX_SetPixel_RGBW_565(ws, pixel_index, COLOR_RGB565_BLACK, 0);
        }
    }
    WS28XX_Update(ws);
}

void UpdateDRLMode(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_drl_displayed)
{
    static uint32_t last_refresh_time = 0;  // Tracks the last refresh time
    uint32_t current_time = HAL_GetTick(); // Get the current tick count

    // Check if DRL mode is already displayed
    if (*is_drl_displayed) {
        // Refresh the DRL state every 100 ms
        if (current_time - last_refresh_time >= DRL_REFRESH_TIME) {
            last_refresh_time = current_time;

            // Reset and update DRL LEDs
//            ResetLEDStrip(ws, pixel_count);
            for (int i = 0; i < pixel_count; i++) {
                WS28XX_SetPixel_RGB_888(ws, i, COLOR_RGB888_WHITE);  // Set all LEDs to full white
            }

            WS28XX_Update(ws);  // Refresh the strip with the new DRL state
            HAL_Delay(6);
            HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
        }
        return;  // Exit function if DRL is already displayed
    }else
    {
        // If DRL is not displayed, initialize DRL mode
        ResetLEDStrip(ws, pixel_count);
        for (int i = 0; i < pixel_count; i++) {
            WS28XX_SetPixel_RGB_888(ws, i, COLOR_RGB888_WHITE);  // Set all LEDs to full white
        }

        WS28XX_Update(ws);  // Refresh the strip with the new DRL state
        HAL_Delay(6);
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);

        // Mark DRL as displayed
        *is_drl_displayed = 1;
        last_refresh_time = current_time;  // Reset refresh timer
    }
}

void UpdateHazardBlink(int led_count)
{
    static int blink_on = 0;  // Track blink state (on/off)

    for (int i = 0; i < led_count; i++) {
        if (blink_on) {
        	WS28XX_SetPixel_RGB_565(&ws_pa10, i, COLOR_RGB565_AMBER); // Amber on for right strip
            WS28XX_SetPixel_RGB_565(&ws_pa8, i, COLOR_RGB565_AMBER);  // Amber on for left strip
        } else {
            WS28XX_SetPixel_RGB_565(&ws_pa8, i, COLOR_RGB565_BLACK);    // Off for left strip
            WS28XX_SetPixel_RGB_565(&ws_pa10, i, COLOR_RGB565_BLACK);   // Off for right strip
        }
    }
    blink_on = !blink_on;
    WS28XX_Update(&ws_pa8);
    HAL_Delay(6);
    WS28XX_Update(&ws_pa10);
}

void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count)
{
    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_888(ws, i, COLOR_RGB888_BLACK, 0);
    }
    WS28XX_Update(ws);
}

void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws)
{
    static int wave_count_middle = 0;               // To track the number of outward-inward wave passes
    static int frame_left = MIDDLE_LED_MID_INDEX;   // Start from the middle (left side)
    static int frame_right = MIDDLE_LED_MID_INDEX;  // Start from the middle (right side)
    static uint8_t wave_direction = 0;              // 0 = outward, 1 = inward
    static int last_update_time = 0;                // To control update speed
    static int drl_wave_complete_middle = 0;        // To indicate when DRL transition is complete
    static int sequential_turn_on = 0;              // Counter for sequential turn-on after waves
    int current_time = HAL_GetTick();               // Get the current system time in milliseconds

    // Phase 1: Wave Animation (Two full passes: outward and inward)
    if (wave_count_middle < 2) {
        if (current_time - last_update_time >= STARTUP_WAVE_SPEED) {
            // Outward wave animation
            if (wave_direction == 0) {
                // Turn off the previous edge LEDs
                for (int i = 0; i < 4; i++) {
                    if (frame_left + i < MIDDLE_LED_MID_INDEX) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_left + i, COLOR_RGB888_BLACK, 0);
                    }
                    if (frame_right - i >= MIDDLE_LED_MID_INDEX) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_right - i, COLOR_RGB888_BLACK, 0);
                    }
                }

                // Turn on the new edge LEDs
                for (int i = 0; i < 4; i++) {
                    if (frame_left - i >= 0) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_left - i, COLOR_RGB888_WHITE, DRL_BRIGHTNESS);
                    }
                    if (frame_right + i < WS28XX_MIDDLE_LED_COUNT) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_right + i, COLOR_RGB888_WHITE, DRL_BRIGHTNESS);
                    }
                }

                WS28XX_Update(ws);
                HAL_Delay(6);
                HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);

                frame_left--;
                frame_right++;

                if (frame_left < 0 && frame_right >= WS28XX_MIDDLE_LED_COUNT) {
                    wave_direction = 1;  // Switch to inward direction
                }
            }
            // Inward wave animation
            else if (wave_direction == 1) {
                // Turn off the previous edge LEDs
                for (int i = 0; i < 4; i++) {
                    if (frame_left - i >= 0) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_left - i, COLOR_RGB888_BLACK, 0);
                    }
                    if (frame_right + i < WS28XX_MIDDLE_LED_COUNT) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_right + i, COLOR_RGB888_BLACK, 0);
                    }
                }

                // Turn on the new edge LEDs
                for (int i = 0; i < 4; i++) {
                    if (frame_left + i < MIDDLE_LED_MID_INDEX) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_left + i, COLOR_RGB888_WHITE, DRL_BRIGHTNESS);
                    }
                    if (frame_right - i >= MIDDLE_LED_MID_INDEX) {
                        WS28XX_SetPixel_RGBW_888(ws, frame_right - i, COLOR_RGB888_WHITE, DRL_BRIGHTNESS);
                    }
                }

                WS28XX_Update(ws);
                HAL_Delay(6);
                HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);

                frame_left++;
                frame_right--;

                if (frame_left >= MIDDLE_LED_MID_INDEX && frame_right <= MIDDLE_LED_MID_INDEX) {
                    frame_left = MIDDLE_LED_MID_INDEX;
                    frame_right = MIDDLE_LED_MID_INDEX;
                    wave_direction = 0;  // Reset to outward
                    wave_count_middle++;  // Increment pass count
                }
            }
            last_update_time = current_time;  // Update time for next wave step
        }

    // Phase 2: Sequential Turn-On after Two Passes
    } else if (!drl_wave_complete_middle) {
        if (current_time - last_update_time >= STARTUP_WAVE_SPEED * 2) {  // Slower speed for smooth sequential effect
            // Light up LEDs sequentially from center to edges
            WS28XX_SetPixel_RGBW_565(ws, MIDDLE_LED_MID_INDEX - sequential_turn_on, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
            WS28XX_SetPixel_RGBW_565(ws, MIDDLE_LED_MID_INDEX + sequential_turn_on, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);

            WS28XX_Update(ws);
            HAL_Delay(6);
            HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
            sequential_turn_on++;

            // Check if all LEDs have been turned on
            if (sequential_turn_on >= MIDDLE_LED_MID_INDEX) {
                drl_wave_complete_middle = 1;  // Mark sequential turn-on complete
            }
            last_update_time = current_time;  // Update time for next step
        }

    // Phase 3: Transition to Steady DRL Mode (All LEDs white)
    } else {
        // Set all LEDs to DRL mode (steady white)
        for (int i = 0; i < WS28XX_MIDDLE_LED_COUNT; i++) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_WHITE, DRL_BRIGHTNESS);
        }
        WS28XX_Update(ws);
        HAL_Delay(6);
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);

        drl_signal_received = 1;  // Signal that DRL mode is active
        startup_signal_received = 0; // Turn off startup mode after one run
        wave_count_middle = 0;    // Reset wave count for future sequences
        drl_wave_complete_middle = 0;  // Reset DRL completion flag
        sequential_turn_on = 0;   // Reset sequential turn-on counter
    }
}

void BreathingEffect(WS28XX_HandleTypeDef* ws, int duration)
{
    static int last_update_time = 0;
    static uint8_t brightness = 0;
    static int increasing = 1; // 1 for increasing, 0 for decreasing
    int current_time = HAL_GetTick();

    // Adjust duration for smoother transitions
    if (current_time - last_update_time >= duration) {
        last_update_time = current_time;

        // Modify brightness increment/decrement step for smoother effect
        if (increasing) {
            brightness += 1;  // Increase brightness step by 5
            if (brightness >= 255) increasing = 0; // Reverse direction
        } else {
            brightness -= 1;
            if (brightness <= 0) increasing = 1;  // Reverse direction
        }

        // Apply brightness level to each LED
        for (int i = 0; i < ws->MaxPixel; i++) {
            WS28XX_SetPixel_RGBW_888(ws, i, COLOR_RGB888_WHITE, brightness);
        }

        WS28XX_Update(ws);  // Refresh LED strip with current brightness level
        HAL_Delay(6);
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
    }
}

void UpdateSOCIndication(WS28XX_HandleTypeDef* ws, int soc_percentage, int charging_type) {
    static int last_update_time = 0;
    static uint8_t charging_blink_state = 0;
    int current_time = HAL_GetTick();
    int led_count_to_light = (WS28XX_MIDDLE_LED_COUNT * soc_percentage) / 100;
    int left_led = MIDDLE_LED_MID_INDEX;
    int right_led = MIDDLE_LED_MID_INDEX;

    if (current_time - last_update_time >= 1000) {
        last_update_time = current_time;

        // Clear blinking LEDs only
        for (int i = 0; i < 10; i++) {
            if (left_led - i >= 0) {
                WS28XX_SetPixel_RGBW_565(ws, left_led - i, COLOR_RGB565_BLACK, 0);
            }
            if (right_led + i < WS28XX_MIDDLE_LED_COUNT) {
                WS28XX_SetPixel_RGBW_565(ws, right_led + i, COLOR_RGB565_BLACK, 0);
            }
        }

        // Set steady LEDs up to SOC percentage
        for (int i = 0; i < led_count_to_light / 2; i++) {
            if (left_led - i >= 0) {
                WS28XX_SetPixel_RGBW_565(ws, left_led - i,
                    (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, DRL_BRIGHTNESS);
            }
            if (right_led + i < WS28XX_MIDDLE_LED_COUNT) {
                WS28XX_SetPixel_RGBW_565(ws, right_led + i,
                    (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, DRL_BRIGHTNESS);
            }
        }

        // Blink next 10 LEDs
        charging_blink_state = !charging_blink_state;
        uint8_t blink_brightness = charging_blink_state ? DRL_BRIGHTNESS : 0;
        for (int i = 0; i < 10; i++) {
            if (left_led - (led_count_to_light / 2) - i >= 0) {
                WS28XX_SetPixel_RGBW_565(ws, left_led - (led_count_to_light / 2) - i,
                    (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, blink_brightness);
            }
            if (right_led + (led_count_to_light / 2) + i < WS28XX_MIDDLE_LED_COUNT) {
                WS28XX_SetPixel_RGBW_565(ws, right_led + (led_count_to_light / 2) + i,
                    (charging_type == AC_CHARGING) ? CHARGING_COLOR_AC : CHARGING_COLOR_DC, blink_brightness);
            }
        }
        WS28XX_Update(ws);
        HAL_Delay(6);
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);

    }
}

void SetGPIOHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void SetGPIOLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void HandleHornState(void)
{
    if (horn_signal_received) {
        current_horn_state = HORN_ON;
    } else {
        current_horn_state = HORN_OFF;
    }

    switch (current_horn_state) {
        case HORN_ON:
            SetGPIOLow(GPIOA, GPIO_PIN_0);  // PA0 LOW to turn on the horn
            break;

        case HORN_OFF:
        default:
            SetGPIOHigh(GPIOA, GPIO_PIN_0);  // PA0 HIGH to turn off the horn
            break;
    }
}

void HandleHeadlampState(void)
{
    if (headlamp_high_beam_signal_received) {
        current_headlamp_state = HEADLAMP_HIGH_BEAM;
    } else if (headlamp_low_beam_signal_received) {
        current_headlamp_state = HEADLAMP_LOW_BEAM;
    } else {
        current_headlamp_state = HEADLAMP_OFF;
    }

    switch (current_headlamp_state) {
        case HEADLAMP_HIGH_BEAM:
            // High beam active (Both low beam and high beam relays should be on)
            SetGPIOLow(GPIOA, GPIO_PIN_1);  // PA1 LOW for headlamp power
            SetGPIOLow(GPIOA, GPIO_PIN_6);  // PA6 LOW for beam selector
            break;

        case HEADLAMP_LOW_BEAM:
            // Low beam active (Only low beam relay should be on)
            SetGPIOLow(GPIOA, GPIO_PIN_1);  // PA1 LOW for headlamp power
            SetGPIOHigh(GPIOA, GPIO_PIN_6);  // PA6 HIGH to deactivate high beam
            break;

        case HEADLAMP_OFF:
        default:
            // Headlamps off
            SetGPIOHigh(GPIOA, GPIO_PIN_1);  // PA1 HIGH to turn off the headlamp power
            SetGPIOHigh(GPIOA, GPIO_PIN_6);  // PA6 HIGH to turn off beam selector
            break;
    }
}

void readHazardSwitch(void)
{
	  uint8_t switchedge = DetectEdge(GPIOB, GPIO_PIN_13, debounceDelay);
	  uint8_t hazard_pressed = 0;

	  if (switchedge == falling_edge){
		  hazard_pressed = 1;
	  }else
	  {
		  hazard_pressed = 0;
	  }
//	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_RESET && hazard_signal_received == 0){
	  if(hazard_pressed && (hazard_signal_received == 0)){ // hazard press is active low
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // give 12 volt power through relay connected to PB1
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //sending signal to rear side
		  hazard_signal_received = 1;
//	  }else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_RESET && hazard_signal_received == 1){
	  }else if(hazard_pressed && (hazard_signal_received == 1)){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // removing 12 volt power through relay
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //sending signal to rear side
		  hazard_signal_received = 0;
//		  HAL_Delay(100);
	  }
}

void readSteeringControls(void) {
    // Read ADC for PA3
    ADC_Select_CH3();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    adc_val_pa3 = HAL_ADC_GetValue(&hadc1);
    voltage_pa3 = (adc_val_pa3 * VREF) / 4095.0; // Convert to voltage
    HAL_ADC_Stop(&hadc1);

    // Read ADC for PA4
    ADC_Select_CH4();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    adc_val_pa4 = HAL_ADC_GetValue(&hadc1);
    voltage_pa4 = (adc_val_pa4 * VREF) / 4095.0; // Convert to voltage
    HAL_ADC_Stop(&hadc1);

    // Read ADC for PA2
    ADC_Select_CH2();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    adc_val_pa2 = HAL_ADC_GetValue(&hadc1);
    voltage_pa2 = (adc_val_pa2 * VREF) / 4095.0; // Convert to voltage
    HAL_ADC_Stop(&hadc1);


    if (voltage_pa3 < 2.6 && voltage_pa3 > 2.3) {
        turn_signal_left_raw = 0;
        turn_signal_right_raw = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    } else if (voltage_pa3 < 1.7 && voltage_pa3 > 1.4) {
        turn_signal_right_raw = 0;
        turn_signal_left_raw = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    } else {
        turn_signal_right_raw = 0;
        turn_signal_left_raw = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    }

    if (voltage_pa4 < 2.6 && voltage_pa4 > 2.3) {
        headlamp_high_raw = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else if (voltage_pa4 < 1.7 && voltage_pa4 > 1.4) {
        headlamp_high_raw = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        headlamp_high_raw = 0;
    }

    if (voltage_pa2 < 2.4 && voltage_pa2 > 1.5) {
        headlamp_low_raw = 0;
        drl_signal_received_raw = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    } else if (voltage_pa2 < 3.0 && voltage_pa2 > 2.6) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        headlamp_low_raw = 1;
        drl_signal_received_raw = 1;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        headlamp_low_raw = 0;
        drl_signal_received_raw = 0;
    }

    // Debounce raw values to update stable flags
    DebounceFlag(&turn_signal_left_flag, turn_signal_left_raw);
    DebounceFlag(&turn_signal_right_flag, turn_signal_right_raw);
    DebounceFlag(&headlamp_low_beam_flag, headlamp_low_raw);
    DebounceFlag(&headlamp_high_beam_flag, headlamp_high_raw);
    DebounceFlag(&drl_flag, drl_signal_received_raw);

    // Update flags with stable values
    turn_signal_left_received = turn_signal_left_flag.stable_value;
    turn_signal_right_received = turn_signal_right_flag.stable_value;
    headlamp_low_beam_signal_received = headlamp_low_beam_flag.stable_value;
    headlamp_high_beam_signal_received = headlamp_high_beam_flag.stable_value;
    drl_signal_received = drl_flag.stable_value;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2); //We don't stop this PWM as this LED strip is too long and doesn't get update fast enough
}

void ADC_Select_CH3(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH4(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

//void ADC_Select_CH5(void)
//{
//	ADC_ChannelConfTypeDef sConfig = {0};
//	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//	  */
//	  sConfig.Channel = ADC_CHANNEL_5;
//	  sConfig.Rank = 1;
//	  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
//	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//}
//
//void ADC_Select_CH7(void)
//{
//	ADC_ChannelConfTypeDef sConfig = {0};
//	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//	  */
//	  sConfig.Channel = ADC_CHANNEL_7;
//	  sConfig.Rank = 1;
//	  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
//	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//}

void readBrakeSwitch(void) {
    DebounceInput(GPIOB, GPIO_PIN_15, &brake_state);

    if (brake_state.stable_value == GPIO_PIN_RESET) { // Active Low: Brake is pressed
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Set PB4
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // Reset PB4
    }
}

void readHorn(void) {
    DebounceInput(GPIOB, GPIO_PIN_12, &horn_state);

    if (horn_state.stable_value == GPIO_PIN_RESET) { // Active Low: Horn is pressed
        horn_signal_received = 1; // Set horn signal flag
    } else {
        horn_signal_received = 0; // Reset horn signal flag
    }
}

void readReverseGear(void) {
    DebounceInput(GPIOA, GPIO_PIN_12, &reverse_state);

    if (reverse_state.stable_value == GPIO_PIN_SET) { // Active High: Reverse is engaged
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Set PB6
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Reset PB6
    }
}

int DebounceHazardPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t debounce_delay)
{
	static int keypress_begin_time = 0;
    static int last_keypress_state = 1;
    static int debounce_loop_count = 0;
    int current_keypress_state = 0;

	current_keypress_state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	debounce_loop_count++;

	if (current_keypress_state != last_keypress_state)
	{
		keypress_begin_time = HAL_GetTick();
		debounce_loop_count = 0;
	}

	if (((HAL_GetTick() - keypress_begin_time) >= debounce_delay) &&( debounce_loop_count > debounceSamplesCount))
	{
		last_keypress_state = current_keypress_state;
		return current_keypress_state;
	}
	return 0;

}

uint8_t DetectEdge(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t debounce_delay)
{
	static int prev_pin_state = 0;
	int curr_pin_state = 0;

	curr_pin_state = DebounceHazardPin(GPIOx,GPIO_Pin,debounce_delay);

	if (prev_pin_state == 0 && curr_pin_state == 1){
		prev_pin_state = curr_pin_state;
		return rising_edge;
	}
	else if(prev_pin_state == 1 && curr_pin_state == 0) {
		prev_pin_state = curr_pin_state;
		return falling_edge;
	}
	else{
		prev_pin_state = curr_pin_state;
	return no_change;
	}
}

void DebounceInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, DebounceState* state)
{
    uint8_t current_value = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    uint32_t current_time = HAL_GetTick();

    if (current_value != state->last_value) {
        state->last_debounce_time = current_time; // Reset debounce timer
        state->sample_count = 0;                 // Reset sample count
    }

    if ((current_time - state->last_debounce_time) >= DEBOUNCE_DELAY) {
        if (state->sample_count >= DEBOUNCE_SAMPLES) {
            state->stable_value = current_value; // Update stable value
        } else {
            state->sample_count++;
        }
    }

    state->last_value = current_value; // Update last value
}

void DebounceFlag(DebounceState* flag_state, uint8_t raw_value)
{
    uint32_t current_time = HAL_GetTick();

    if (raw_value != flag_state->last_value) {
        flag_state->last_debounce_time = current_time; // Reset debounce timer
        flag_state->sample_count = 0;                 // Reset sample count
    }

    if ((current_time - flag_state->last_debounce_time) >= 50) { // 50ms debounce delay
        if (flag_state->sample_count >= 5) { // 5 consistent samples required
            flag_state->stable_value = raw_value; // Update stable value
        } else {
            flag_state->sample_count++;
        }
    }

    flag_state->last_value = raw_value; // Update last observed value
}

#ifdef TEST_MODE
void Handle_TestMode(void)
{
    static int test_state = 0;
    static uint32_t last_state_change_time = 0;
    uint32_t current_time = HAL_GetTick();

    // Transition to the next state every 14 seconds
    if (current_time - last_state_change_time >= 14000) {
        // Reset all flags to 0
        charging_signal_received = 0;
        drl_signal_received = 0;
        hazard_signal_received = 0;
        turn_signal_left_received = 0;
        turn_signal_right_received = 0;
        horn_signal_received = 0;
        headlamp_low_beam_signal_received = 0;
        headlamp_high_beam_signal_received = 0;
        breathing_signal_received = 0;

        // Test mode states
        switch (test_state) {
            case 0:
                // Startup mode for all
                breathing_signal_received = 1; // Enable startup mode initially
                break;
            case 1:
                // Turn signals for side strips, DRL for middle strip
                turn_signal_left_received = 1;
                turn_signal_right_received = 1;
                drl_signal_received = 1;
                break;
            case 2:
                // SOC charging for middle, hazard for side strips
                charging_signal_received = 1;
                hazard_signal_received = 1;
                break;
            case 3:
                // All in DRL mode
                drl_signal_received = 1;
                break;
            default:
                // Reset the test state after the last one
                test_state = -1;
                break;
        }

        // Move to the next test state
        test_state++;
        last_state_change_time = current_time;
    }
}
#endif

//void UART_Init(void)
//{
//    // Enable clock for GPIOB and USART3
//    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // Enable GPIOB clock
//    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock
//
//    // Configure PB10 (TX) and PB11 (RX)
//    GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11); // Reset the bits first
//    GPIOB->CRH |= (GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1);  // PB10: Alternate function push-pull for TX
//    GPIOB->CRH |= GPIO_CRH_CNF11_0;                        // PB11: Input floating for RX
//
//    // Set the baud rate
//    USART3->BRR = (Clock_Frequency * 1000) / BAUD_RATE;
//
//    // Enable USART3, TX, and RX
//    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
//}
//
//void UART_Send(uint8_t data)
//{
//    while (!(USART3->SR & USART_SR_TXE));  // Wait until TX buffer is empty
//    USART3->DR = data;                     // Send the data
//}
//
//bool UART_Receive(uint8_t *data, uint32_t timeout)
//{
//    uint32_t startTime = HAL_GetTick();
//    while (!(USART3->SR & USART_SR_RXNE))  // Wait until RX buffer is not empty
//    {
//        if ((HAL_GetTick() - startTime) >= timeout)
//        {
//            return false;  // Timeout reached
//        }
//    }
//    *data = (uint8_t)USART3->DR;  // Read received data
//    return true;
//}
//
//void ProcessUARTMessage(uint8_t received_byte)
//{
//    // Side LED Strips Control (Bits 4 and 5)
//    switch ((received_byte >> 4) & 0b11) {
//        case 0b00:
//            drl_signal_received = 1;         // DRL Mode
//            turn_signal_left_received = 0;
//            turn_signal_right_received = 0;
//            hazard_signal_received = 0;
//            break;
//        case 0b01:
//            drl_signal_received = 0;
//            turn_signal_left_received = 1;   // Left Turn Signal
//            turn_signal_right_received = 0;
//            hazard_signal_received = 0;
//            break;
//        case 0b10:
//            drl_signal_received = 0;
//            turn_signal_left_received = 0;
//            turn_signal_right_received = 1;  // Right Turn Signal
//            hazard_signal_received = 0;
//            break;
//        case 0b11:
//            drl_signal_received = 0;
//            turn_signal_left_received = 0;
//            turn_signal_right_received = 0;
//            hazard_signal_received = 0;      // Hazard Mode
//            break;
//    }
//
//    // Headlamp Control (Bits 1 and 2)
//    switch ((received_byte >> 1) & 0b11) {
//        case 0b00:
//            headlamp_low_beam_signal_received = 0;  // Headlamp off
//            headlamp_high_beam_signal_received = 0;
//            break;
//        case 0b01:
//            headlamp_low_beam_signal_received = 0;
//            headlamp_high_beam_signal_received = 1;  // High Beam
//            break;
//        case 0b10:
//            headlamp_low_beam_signal_received = 1;  // Low Beam
//            headlamp_high_beam_signal_received = 0;
//            break;
//        case 0b11:
//            // Don't care (keep the current headlamp state)
//            break;
//    }
//
//    // Horn Control (Bit 0)
//    horn_signal_received = (received_byte & 0b00000001);  // Horn on/off
//}
//
//void UART_ReceiveAndProcess(void)
//{
//    if (UART_Receive(&UART_received_byte, 1000)) {  // Timeout of 1000 ms
////    	UART_Send(UART_received_byte);  // Echo the received byte back to the terminal
//        ProcessUARTMessage(UART_received_byte);  // Process the byte to set flags
//    }
//}
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
