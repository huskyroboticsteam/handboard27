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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_INTERVAL_MS 500

/* Need to verify the timeout ms amount */
#define CAN_TIMEOUT_MS        1000
#define MOTOR_TIMEOUT_MS      500
#define ACTUATOR_TIMEOUT_MS   500
#define LIMIT_SWITCH_DEBOUNCE_MS 50

#define PIN_DIR       GPIO_PIN_5
#define DIR_OPEN      GPIO_PIN_SET
#define DIR_CLOSE     GPIO_PIN_RESET
#define STEPPER_PORT  GPIOA

#define MOTOR_ENABLE_LEVEL  GPIO_PIN_SET
#define MOTOR_DISABLE_LEVEL GPIO_PIN_RESET

#define LIMIT_SWITCH_PRESSED GPIO_PIN_SET

#define LASER_PERIPH_ID 0x01
#define LASER_RED1_PIN  GPIO_PIN_3    /* Placeholder Pin Out */
#define LASER_RED2_PIN  GPIO_PIN_4    /* Placeholder Pin Out */
#define LASER_GREEN_PIN GPIO_PIN_5    /* Placeholder Pin Out */ 
#define LASER_PORT      GPIOA         /* Placeholder Port */

#define ACTUATOR1_PERIPH_ID 0x02
#define ACTUATOR1_PIN_A     GPIO_PIN_6
#define ACTUATOR1_PIN_B     GPIO_PIN_7
#define ACTUATOR1_PORT      GPIOB

#define ACTUATOR2_PERIPH_ID 0x03
#define ACTUATOR2_PIN_A     GPIO_PIN_8
#define ACTUATOR2_PIN_B     GPIO_PIN_9
#define ACTUATOR2_PORT      GPIOB

#define LIMIT_SWITCH_PIN  GPIO_PIN_1
#define LIMIT_SWITCH_PORT GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

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

uint32_t actuator_enable_time = 0;
uint8_t  actuator_active      = 0;

static uint8_t limit_triggered = 0;
static uint32_t limit_debounce_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_FDCAN2_Init(void);
/* USER CODE BEGIN PFP */
void Send_Heartbeat(void);
void Send_LimitSwitch_Alert(void);
void Motor_Disable(void);
uint8_t Motor_Enable(void);
void Laser_SetBrightness(uint16_t brightness);
void Laser_On(void);
void Laser_Off(void);
void Actuator1_Forward(void);
void Actuator1_Reverse(void);
void Actuator2_Forward(void);
void Actuator2_Reverse(void);
void Actuator_Off(void);
void Actuator_Brake(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */

  CANInit(CANHandle, &thisCANDevice);
  HAL_GPIO_WritePin(STEPPER_PORT, STEPPER_NSLEEP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, CAN_LED_Pin, GPIO_PIN_SET);
  Laser_Off();
  Actuator_Off();

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
      Motor_Disable();
      Laser_Off();
      Actuator_Brake();
      actuator_active = 0;
      actuator_enable_time = 0;
    }

	  if (motor_enabled && (current_tick - motor_enable_time) > MOTOR_TIMEOUT_MS) {
		  Motor_Disable();
	  }

	  if (actuator_active && (current_tick - actuator_enable_time) > ACTUATOR_TIMEOUT_MS) {
		  Actuator_Brake();
		  actuator_active = 0;
		  actuator_enable_time = 0;
	  }

	  if (HAL_GPIO_ReadPin(LIMIT_SWITCH_PORT, LIMIT_SWITCH_PIN) == LIMIT_SWITCH_PRESSED) {
		  if (!limit_triggered) {
			  if (limit_debounce_time == 0) {
				  limit_debounce_time = current_tick;
			  } else if ((current_tick - limit_debounce_time) >= LIMIT_SWITCH_DEBOUNCE_MS) {
				  limit_triggered = 1;
				  Motor_Disable();
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
					if(pwmPacket.peripheralID == LASER_PERIPH_ID) {
						if (pwmPacket.dutyCycle > 0.0f) {
							// uint16_t brightness = (uint16_t)((pwmPacket.dutyCycle * LASER_MAX_BRIGHTNESS) / 100.0f);
						  //   Laser_SetBrightness(brightness);
              Laser_On();
						}
						else {
							Laser_Off();
						}
						failure = 0;
					}
					break;
		  	  case(CAN_COMMAND_ID__E_STOP):
		  			Motor_Disable();
		  	  	  	Laser_Off();
		  	  	  	Actuator_Brake();
		  	  	  	actuator_active = 0;
		  	  	  	actuator_enable_time = 0;
		  	  	  	failure = 0;
		  	  	  	break;

		  	  case(CAN_COMMAND_ID__STEPPER_DRIVE_REVS):
					CANMotorPacket_Stepper_DriveRevolutions_Decoded_t revs_decoded = CANMotorPacket_Stepper_DriveRevolutions_Decode(&rxPacket);
					if (revs_decoded.numRevolutions > 0.0f) {
						HAL_GPIO_WritePin(STEPPER_PORT, PIN_DIR, DIR_OPEN);
						failure = Motor_Enable();
					} else if (revs_decoded.numRevolutions < 0.0f) {
						if(limit_triggered) {
							Send_LimitSwitch_Alert();
						}

						HAL_GPIO_WritePin(STEPPER_PORT, PIN_DIR, DIR_CLOSE);
						failure = Motor_Enable();
					} else {
						Motor_Disable();
						failure = 0;
					}
					break;

		  	  case(CAN_COMMAND_ID__LINEAR_ACTUATOR_CONTROL):
					CANPeripheralPacket_SetLinearActuator_Decoded_t decoded =
		                CANPeripheralPacket_SetLinearActuator_Decode(&rxPacket);

		            if (decoded.peripheralID == ACTUATOR_PERIPH_ID) {
		                failure = 0;
		            	if (decoded.drive > 0) {
		                    Actuator_Forward();
		                    actuator_active = 1;
		                    actuator_enable_time = HAL_GetTick();
		                } else if (decoded.drive < 0) {
		                    Actuator_Reverse();
		                    actuator_active = 1;
		                    actuator_enable_time = HAL_GetTick();
		                } else {
		                    Actuator_Brake();
		                    actuator_active = 0;
		                    actuator_enable_time = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|STEPPER_NSLEEP_Pin|STEPPER_NEABLE_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|DBG_LED_Pin|ERR_LED_Pin
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 STEPPER_NSLEEP_Pin STEPPER_NEABLE_Pin PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|STEPPER_NSLEEP_Pin|STEPPER_NEABLE_Pin|GPIO_PIN_5;
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
                           ERR_LED_Pin PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|CAN_LED_Pin|DBG_LED_Pin
                          |ERR_LED_Pin|GPIO_PIN_6|GPIO_PIN_7;
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
uint8_t Motor_Enable(void) {
	uint8_t direction = (GPIOA->ODR & PIN_DIR) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	if(!limit_triggered || (direction == DIR_OPEN)) { // Let the motor open hand even if limit switch is pressed
    	HAL_GPIO_WritePin(STEPPER_PORT, STEPPER_NEABLE_Pin, MOTOR_ENABLE_LEVEL);
    	HAL_GPIO_WritePin(GPIOB, DBG_LED_Pin, GPIO_PIN_RESET);
        motor_enabled     = 1;
        motor_enable_time = HAL_GetTick();
    	return HAL_OK;
    } else {
    	return HAL_ERROR;
    }
}

void Motor_Disable(void) {
    HAL_GPIO_WritePin(STEPPER_PORT, STEPPER_NEABLE_Pin, MOTOR_DISABLE_LEVEL);
    HAL_GPIO_WritePin(GPIOB, DBG_LED_Pin, GPIO_PIN_SET);
    motor_enabled     = 0;
    motor_enable_time = 0;
}

void Laser_On(void) {
  HAL_GPIO_WritePin(LASER_PORT, LASER_RED1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LASER_PORT, LASER_RED2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LASER_PORT, LASER_GREEN_PIN, GPIO_PIN_SET);
}

void Laser_Off(void) {
    HAL_GPIO_WritePin(LASER_PORT, LASER_RED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LASER_PORT, LASER_RED2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LASER_PORT, LASER_GREEN_PIN, GPIO_PIN_RESET);
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

void Actuator_Brake(void) {
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACTUATOR2_PORT, ACTUATOR2_PIN_B, GPIO_PIN_SET);
}

void Actuator_Off(void) {
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
