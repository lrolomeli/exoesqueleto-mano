/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	empty,
	full
}enum_buffer_status;

typedef enum{
	emergency_rst,
	conn_success,
	moving_forward,
	tx_succeed,
	moving_backward,
	halt_state,
	reference_routine,
	disconn_success
}enum_action;

typedef enum{
	active,
	passive
}enum_action_cat;

typedef enum{
	lost,
	home,
	referenced
}enum_motor_stat;

typedef struct{
	int32_t absolute_pos;
	uint16_t step;
}st_step_position;

typedef enum{
	full_step,
	half_step,
	quarter_step,
	eigth_step,
	sixteen_step
}enum_stepping;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define YES 1
#define NO 0
#define FINISH 1
#define dfl_steps 2000
#define HOME_STEPS 1
#define COMPLETE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t buffer = 0;
static volatile uint8_t one_msec_flag = 0;
static volatile enum_motor_stat home_routine = lost;
static uint32_t steps_to_go = 0;
static volatile enum_buffer_status buffer_status = empty;
const enum_stepping stepping = full_step;
const uint8_t ack[] = "\rReference Routine Complete\n";
static uint8_t op_has_started = NO;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static enum_action read_action(void);
static void set_pwm(uint16_t pwm);
static uint8_t send_step_pulses(st_step_position * position, enum_action mov_act);
static void one_second_pwm(void);
void start_action(enum_action act);
void clean_buffer(void);
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	static enum_action action = halt_state;
	static st_step_position position = {0};
	//uint16_t steps = 0;

	HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(full == buffer_status)
		{
			action = read_action();
			if(action == emergency_rst)
			{
				op_has_started = NO;
			}
			else if(action % 2 == active)
			{
				start_action(action);
				home_routine = 0;
			}
			else if(action % 2 == passive)
			{
			}
			else
			{
			}
			clean_buffer();
			HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));

		}

		if(home_routine == home)
		{
			op_has_started = NO;
			position.absolute_pos = 0;
			home_routine = referenced;
		}

		if(one_msec_flag)
		{
			if(op_has_started)
			{
				if(send_step_pulses(&position, action) == FINISH)
				{
					op_has_started = NO;
				}
			}
			one_second_pwm();
			one_msec_flag = 0;
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

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin)
	{
	case GPIO_PIN_5:
		home_routine = home;
		HAL_UART_Transmit(&huart3, (const uint8_t *) ack, sizeof(ack), 100);
		break;
	case GPIO_PIN_6:
		break;
	case GPIO_PIN_7:
		break;
	case GPIO_PIN_8:
		break;
	case GPIO_PIN_9:
		break;
	default:
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//waits until buffer is empty again
	if(empty == buffer_status){
		buffer_status = full;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if(htim->Instance == TIM3)
	{
		if(~one_msec_flag)
		{
			one_msec_flag = 1;
		}
	}
}

static enum_action read_action(void)
{
	uint8_t rx_data = 0;

	rx_data = buffer;

	//compare byte
	switch(rx_data){
	case '\e':
		return emergency_rst;
	case 'f':
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
		return moving_forward;
	case 'b':
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		return moving_backward;
	case 'h':
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		return reference_routine;
	case 0:
		return tx_succeed;
	case '+':
			return halt_state;
	default:
		return halt_state;
	}
}

static void set_pwm(uint16_t pwm){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);
}

static uint8_t send_step_pulses(st_step_position * position, enum_action act)
{

	if(steps_to_go > 0)
	{
		switch(act)
		{
		case reference_routine:
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			return 0;
		case moving_backward:
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			position->absolute_pos--;
			steps_to_go--;
			return 0;
		case moving_forward:
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			position->absolute_pos++;
			steps_to_go--;
			return 0;
		default:
			steps_to_go = 0;
			return FINISH;
		}

	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		return FINISH;
	}
}

static void one_second_pwm(void)
{
	static uint16_t var = 999;

	set_pwm(var << 6);
	if(var > 0)
	{
		var--;
	}
	else{
		var = 999;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

void start_action(enum_action act){

	switch(act)
	{
	case reference_routine:
		home_routine = lost;
		steps_to_go = HOME_STEPS;
		op_has_started = YES;
		break;
	case moving_backward:
		steps_to_go = dfl_steps << stepping;
		op_has_started = YES;
		break;
	case moving_forward:
		steps_to_go = dfl_steps << stepping;
		op_has_started = YES;
		break;
	default:
		steps_to_go = 0;
		op_has_started = NO;
		break;
	}

}

void clean_buffer()
{
	buffer = 0;
	buffer_status = empty;
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
