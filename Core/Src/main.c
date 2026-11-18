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
#include "../../Libraries/CAN26/CAN26.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_INTERVAL_MS 500

#define STEP_TIM      htim3
#define STEP_CHANNEL  TIM_CHANNEL_1
#define STEPPER_WAKE_DELAY_MS 2     // DRV8425 wakes in 1.2 ms max
#define SLEEP_TIMEOUT_MS      1000

/* Need to verify the timeout ms amount */
#define CAN_TIMEOUT_MS        1000
#define ACTUATOR_TIMEOUT_MS   500
#define LIMIT_SWITCH_DEBOUNCE_MS 50

#define PIN_DIR       STEPPER_DIR_Pin
#define DIR_OPEN      GPIO_PIN_SET
#define DIR_CLOSE     GPIO_PIN_RESET
#define STEPPER_PORT  GPIOA
#define STEPPER_NEABLE_Pin STEPPER_SLEEP_Pin  // placeholder, change if the enable pin is different
#define STEPS_PER_REV 200  // placeholder: full steps per rev x microstep setting

#define MOTOR_ENABLE_LEVEL  GPIO_PIN_SET
#define MOTOR_DISABLE_LEVEL GPIO_PIN_RESET

#define LIMIT_SWITCH_PRESSED GPIO_PIN_SET

#define LASERS_PERIPH_ID  0x01
#define LASERS_PIN        GPIO_PIN_9
#define LASERS_PORT       GPIOA         

#define ACTUATOR1_PERIPH_ID 0x02
#define ACTUATOR1_NSLEEP    GPIO_PIN_0
#define ACTUATOR1_DISABLE   GPIO_PIN_1
#define ACTUATOR1_PH        GPIO_PIN_2
#define ACTUATOR1_EN        GPIO_PIN_3
#define ACTUATOR1_NFAULT    GPIO_PIN_4
#define ACTUATOR1_PORT      GPIOA

#define ACTUATOR2_PERIPH_ID 0x03
#define ACTUATOR2_NSLEEP    GPIO_PIN_13
#define ACTUATOR2_DISABLE   GPIO_PIN_0
#define ACTUATOR2_PH        GPIO_PIN_1
#define ACTUATOR2_EN        GPIO_PIN_2
#define ACTUATOR2_NFAULT    GPIO_PIN_3
#define ACTUATOR2_PORT      GPIOC

#define FLASLIGHT_PERIPH_ID 0x04
#define FLASLIGHT_PIN       GPIO_PIN_8
#define FLASLIGHT_PORT      GPIOA

#define LIMIT_SWITCH_PIN  GPIO_PIN_1
#define LIMIT_SWITCH_PORT GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
CANHandle_t CANHandle = &hfdcan2;
CANDevice_t thisCANDevice = {
		.peripheralDomain = 1,
		.powerDomain = 0,
		.motorDomain = 1,
		.deviceUUID = CAN_UUID_HAND
};

CANDevice_t jetson = {
		.peripheralDomain = 0,
		.powerDomain = 0,
		.motorDomain = 0,
		.deviceUUID = CAN_UUID_JETSON
};

const char *fwName = "hand";
uint16_t revision = 4;
CANPacket_t rxPacket;
CANPacket_t txPacket;

uint32_t last_heartbeat_time = 0;
uint32_t can_led_off_time = 0;

uint32_t last_can_rx_time = 0;
uint8_t can_timeout_active = 0;

uint32_t motor_enable_time   = 0;
uint8_t  motor_enabled       = 0;

uint32_t actuator1_enable_time = 0;
uint8_t  actuator1_active      = 0;

uint32_t actuator2_enable_time = 0;
uint8_t actuator2_active       = 0;

static uint8_t limit_triggered = 0;
static uint32_t limit_debounce_time = 0;
static volatile uint32_t steps_remaining = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void Send_Heartbeat(void);
void Send_LimitSwitch_Alert(void);

void Stepper_On(int32_t steps);
void Stepper_Brake(void);
void Stepper_Sleep(void);

void Lasers_On(void);
void Lasers_Off(void);

void Flaslight_On(void);
void Flaslight_Off(void);

void Actuator1_Forward(void);
void Actuator1_Reverse(void);
void Actuator1_On();
void Actuator1_Brake(void);
void Actuator1_Sleep(void);

void Actuator2_Forward(void);
void Actuator2_Reverse(void);
void Actuator2_On();
void Actuator2_Brake(void);
void Actuator2_Sleep(void);

void Actuator1_Off(void);
void Actuator1_Brake(void);

void Actuator2_Off(void);
void Actuator2_Off(void);
void Actuator2_Brake(void);

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
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  CANInit(CANHandle, &thisCANDevice);
  HAL_GPIO_WritePin(STEPPER_PORT, STEPPER_SLEEP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, CAN_LED_Pin, GPIO_PIN_SET);
  Lasers_Off();
  Flaslight_Off();
  Actuator1_Off();
  Actuator2_Off();

  last_heartbeat_time = HAL_GetTick();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t current_tick = HAL_GetTick();

	  if (can_led_off_time != 0) {
		  if (current_tick >= can_led_off_time) {
			  HAL_GPIO_WritePin(GPIOB, CAN_LED_Pin, GPIO_PIN_SET);
			  can_led_off_time = 0;
		  }
	  }

	  if ((current_tick - last_heartbeat_time) >= HEARTBEAT_INTERVAL_MS) {
		  Send_Heartbeat();
		  last_heartbeat_time = current_tick;
	  }

    if ((current_tick - last_can_rx_time) >= CAN_TIMEOUT_MS && !can_timeout_active) {
      can_timeout_active = 1;
      Stepper_Sleep();
      Lasers_Off();
      Flaslight_Off();
      Actuator_Brake();
      actuator_active = 0;
      actuator_enable_time = 0;
    }

	  if (actuator1_active && (current_tick - actuator1_enable_time) > ACTUATOR_TIMEOUT_MS) {
		  Actuator1_Brake();
		  actuator1_active = 0;
		  actuator1_enable_time = 0;
	  }

    if (actuator2_active && (current_tick - actuator2_enable_time) > ACTUATOR_TIMEOUT_MS) {
		  Actuator2_Brake();
		  actuator2_active = 0;
		  actuator2_enable_time = 0;
	  }

	  if (HAL_GPIO_ReadPin(LIMIT_SWITCH_PORT, LIMIT_SWITCH_PIN) == LIMIT_SWITCH_PRESSED) {
		  if (!limit_triggered) {
			  if (limit_debounce_time == 0) {
				  limit_debounce_time = current_tick;
			  } else if ((current_tick - limit_debounce_time) >= LIMIT_SWITCH_DEBOUNCE_MS) {
				  limit_triggered = 1;
				  Stepper_Brake();
				  Send_LimitSwitch_Alert();
			  }
		  }
	  } else {
		  limit_debounce_time = 0;
		  limit_triggered = 0;
	  }

	  if(CANPollAndReceive(CANHandle, &rxPacket)) {
		  can_led_off_time = HAL_GetTick() + 20;
		  HAL_GPIO_WritePin(GPIOB, CAN_LED_Pin, GPIO_PIN_RESET);

      last_can_rx_time = HAL_GetTick();
      can_timeout_active = 0;
      
		  uint8_t ackReq = rxPacket.command & 0x80;
		  bool failure = 1;
		  CANDevice_t sender = {
					.peripheralDomain = 0,
					.powerDomain = 0,
					.motorDomain = 0,
					.deviceUUID = rxPacket.senderUUID
		  };

		  switch(rxPacket.command & 0x7F) {
		  	  case(CAN_COMMAND_ID__VERSION_GET):
					txPacket = CANUniversalPacket_FirmwareVersion(thisCANDevice, sender, fwName, revision);
					failure = CANSend(CANHandle, &txPacket);
					break;


		  	  case(CAN_COMMAND_ID__PWM_DUTY_CYCLE):
					CANPeripheralPacket_SetPWMDutyCycle_Decoded_t pwmPacket = CANPeripheralPacket_SetPWMDutyCycle_Decode(&rxPacket);
					if(pwmPacket.peripheralID == LASERS_PERIPH_ID) {
						if (pwmPacket.dutyCycle > 0.0f) {
              Lasers_On();
						}
						else {
							Lasers_Off();
						}
						failure = 0;
					} else if(pwmPacket.peripheralID == FLASLIGHT_PERIPH_ID) {
            if (pwmPacket.dutyCycle > 0.0f) {
              Flaslight_On();
						}
						else {
							Flaslight_Off();
						}
						failure = 0;
          }
					break;

		  	  case(CAN_COMMAND_ID__E_STOP):
		  			Stepper_Sleep();
            Lasers_Off();
            Actuator1_Brake();
            Actuator2_Brake();
            actuator1_active = 0;
            actuator1_enable_time = 0;
            actuator2_active = 0;
            actuator2_enable_time = 0;
            failure = 0;
            break;

		  	  case(CAN_COMMAND_ID__STEPPER_DRIVE_REVS):
					CANMotorPacket_Stepper_DriveRevolutions_Decoded_t revs_decoded = CANMotorPacket_Stepper_DriveRevolutions_Decode(&rxPacket);
					if (revs_decoded.numRevolutions > 0.0f) {
						Stepper_On((int32_t)(revs_decoded.numRevolutions * STEPS_PER_REV));
						failure = 0;
					} else if (revs_decoded.numRevolutions < 0.0f) {
						if(limit_triggered) {
							Send_LimitSwitch_Alert();
							failure = 1;
						} else {
							Stepper_On((int32_t)(revs_decoded.numRevolutions * STEPS_PER_REV));
							failure = 0;
						}
					} else {
						Stepper_Sleep();
						failure = 0;
					}
					break;

		  	  case(CAN_COMMAND_ID__LINEAR_ACTUATOR_CONTROL):
					CANPeripheralPacket_SetLinearActuator_Decoded_t decoded =
		                CANPeripheralPacket_SetLinearActuator_Decode(&rxPacket);

		            if (decoded.peripheralID == ACTUATOR1_PERIPH_ID) {
		                failure = 0;
		            	if (decoded.drive > 0) {
		                    Actuator1_Forward();
		                    actuator1_active = 1;
		                    actuator1_enable_time = HAL_GetTick();
		                } else if (decoded.drive < 0) {
		                    Actuator1_Reverse();
		                    actuator1_active = 1;
		                    actuator1_enable_time = HAL_GetTick();
		                } else {
		                    Actuator1_Brake();
		                    actuator1_active = 0;
		                    actuator1_enable_time = 0;
		                }
                } else if (decoded.peripheralID == ACTUATOR2_PERIPH_ID) {
                    failure = 0;
                  if (decoded.drive > 0) {
                        Actuator2_Forward();
                        actuator2_active = 1;
                        actuator2_enable_time = HAL_GetTick();
                  } else if (decoded.drive < 0) {
                        Actuator2_Reverse();
                        actuator2_active = 1;
                        actuator2_enable_time = HAL_GetTick();
                  } else {
                        Actuator2_Brake();
                        actuator2_active = 0;
                        actuator2_enable_time = 0;
		              }
                }
		            break;
		  }

			if(ackReq) {
				txPacket = CANUniversalPacket_Acknowledge(thisCANDevice, sender, failure, rxPacket.command);
				CANSend(CANHandle, &txPacket);
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (steps_remaining > 0) {
      motor_enable_time = HAL_GetTick();  // still stepping, keep the driver awake
    } else if (motor_enabled && (HAL_GetTick() - motor_enable_time) > SLEEP_TIMEOUT_MS) {
      Stepper_Sleep();
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 4;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 27;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 4;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LASERS_PIN|STEPPER_DIR_Pin|STEPPER_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|DBG_LED_Pin|ERR_LED_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LASERS_PIN
                           STEPPER_DIR_Pin STEPPER_SLEEP_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LASERS_PIN
                          |STEPPER_DIR_Pin|STEPPER_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 CAN_LED_Pin DBG_LED_Pin
                           ERR_LED_Pin PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|CAN_LED_Pin|DBG_LED_Pin
                          |ERR_LED_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Stepper_On(int32_t steps) {
  if (steps == 0) return;
  GPIO_PinState direction = (steps > 0) ? DIR_OPEN : DIR_CLOSE;

  if (limit_triggered && direction == DIR_CLOSE) return;

  HAL_GPIO_WritePin(STEPPER_PORT, PIN_DIR, direction);

  if (!motor_enabled) {               // wake the driver up if it is asleep
    HAL_GPIO_WritePin(STEPPER_PORT, STEPPER_NEABLE_Pin, MOTOR_ENABLE_LEVEL);
    HAL_GPIO_WritePin(GPIOB, DBG_LED_Pin, GPIO_PIN_RESET);
    motor_enabled = 1;
    HAL_Delay(STEPPER_WAKE_DELAY_MS);  // allow the driver to wake up
  }
  motor_enable_time = HAL_GetTick();        // reset the sleep timer

  HAL_TIM_PWM_Stop(&STEP_TIM, STEP_CHANNEL);  // stop any previous pulses
  steps_remaining = (steps > 0) ? steps : -steps;  // number of pulses to send
  __HAL_TIM_SET_COUNTER(&STEP_TIM, 0);
  __HAL_TIM_CLEAR_FLAG(&STEP_TIM, TIM_FLAG_UPDATE);  // clear before enabling the interrupt
  __HAL_TIM_ENABLE_IT(&STEP_TIM, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start(&STEP_TIM, STEP_CHANNEL);
}

void Stepper_Brake(void) {
  HAL_TIM_PWM_Stop(&STEP_TIM, STEP_CHANNEL);  // stop pulses, driver stays on and holds
  __HAL_TIM_DISABLE_IT(&STEP_TIM, TIM_IT_UPDATE);
  steps_remaining = 0;
}

void Stepper_Sleep(void) {
  Stepper_Brake();
  HAL_GPIO_WritePin(STEPPER_PORT, STEPPER_NEABLE_Pin, MOTOR_DISABLE_LEVEL);  // put driver to sleep
  HAL_GPIO_WritePin(GPIOB, DBG_LED_Pin, GPIO_PIN_SET);
  motor_enabled = 0;
  motor_enable_time = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &STEP_TIM && steps_remaining > 0) {
    steps_remaining--;
    if (steps_remaining == 0) {
      Stepper_Brake();  // all steps sent
    }
  }
}

void Lasers_On(void) {
  HAL_GPIO_WritePin(LASERS_PORT, LASERS_PIN, GPIO_PIN_SET);
}

void Lasers_Off(void) {
    HAL_GPIO_WritePin(LASERS_PORT, LASERS_PIN, GPIO_PIN_RESET);
}

void Flaslight_On(void) {
    HAL_GPIO_WritePin(FLASLIGHT_PORT, FLASLIGHT_PIN, GPIO_PIN_SET);
}

void Flaslight_Off(void) {
    HAL_GPIO_WritePin(FLASLIGHT_PORT, FLASLIGHT_PIN, GPIO_PIN_RESET);
}

void Actuator1_Forward(void) {
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_B, GPIO_PIN_RESET);
}

void Actuator1_Reverse(void) {
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_B, GPIO_PIN_SET);
}

void Actuator2_Forward(void) {
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_B, GPIO_PIN_RESET);
}

void Actuator2_Reverse(void) {
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_B, GPIO_PIN_SET);
}

void Actuator1_Brake(void) {
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_B, GPIO_PIN_SET);
}

void Actuator1_Off(void) {
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR1_PORT, ACTUATOR1_PIN_B, GPIO_PIN_RESET);
}

void Actuator2_Brake(void) {
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_B, GPIO_PIN_SET);
}

void Actuator2_Off(void) {
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_B, GPIO_PIN_RESET);
}

void Send_Heartbeat(void) {
    CANPacket_t heartbeat;
    heartbeat.device         = jetson;
    heartbeat.priority       = CAN_PRIORITY_LOW;
    heartbeat.command        = CAN_COMMAND_ID__HEARTBEAT;
    heartbeat.senderUUID     = ((CANDeviceUUID_t)thisCANDevice.deviceUUID);
    heartbeat.contentsLength = 5;
    uint32_t error = 0;
    uint8_t  state = motor_enabled ? 1 : 0;
    CANStoreUInt32(heartbeat.contents + 0, error);
    heartbeat.contents[4] = state;
    CANSend(CANHandle, &heartbeat);
}

void Send_LimitSwitch_Alert(void) {
    CANPacket_t alert = CANMotorPacket_LimitSwitchAlert(thisCANDevice, jetson, 0x01, true);
    CANSend(CANHandle, &alert);
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
