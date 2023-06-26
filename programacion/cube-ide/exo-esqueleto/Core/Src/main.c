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
	empty = 0,
	full
}enum_buffer_status;

typedef enum{
	emergency_rst = 0,
	conn_success,
	moving_forward,
	tx_succeed,
	moving_backward,
	halt_state,
	reference_routine,
	disconn_success
}enum_action;

typedef enum{
	active = 0,
	passive
}enum_action_cat;

typedef enum{
	lost = 0,
	home,
	referenced
}enum_motor_stat;

typedef enum{
	full_step = 0,
	half_step,
	quarter_step,
	eigth_step,
	sixteen_step
}enum_stepping;

typedef enum{
	thumb = 0,
	index,
	middle,
	ring,
	little,
	flength
}enum_fingers;

typedef enum{
	portA = 0,
	portB,
}enum_ports;

typedef struct{
	int32_t absolute_pos;
	uint32_t steps_to_go;
	uint8_t op_has_started;
}st_fposition;

typedef struct{
	const uint8_t port;
	const uint16_t pin;
}st_gpio_config_t;

typedef struct {
	const st_gpio_config_t step;
	const st_gpio_config_t direction;
	const st_gpio_config_t sleep;
	const st_gpio_config_t home;
}st_fconfig;

typedef struct {
	enum_action action;
	enum_fingers finger;
}st_action;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define YES 1
#define NO 0
#define DOWN 1
#define UP 0
#define FINISH 1
#define dfl_steps 500
#define HOME_STEPS 1
#define COMPLETE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const st_fconfig fconfig[flength] = {
	/*       steps               direction                sleep                 home          */
	{ {portA, GPIO_PIN_1}, 	{portA, GPIO_PIN_0},   {portB, GPIO_PIN_4},	 {portB, GPIO_PIN_9} },//thumb
	{ {portB, GPIO_PIN_14},	{portB, GPIO_PIN_13},  {portB, GPIO_PIN_15}, {portB, GPIO_PIN_5} },//index
	{ {portB, GPIO_PIN_1}, 	{portB, GPIO_PIN_0},   {portA, GPIO_PIN_8},	 {portB, GPIO_PIN_6} },//middle
	{ {portA, GPIO_PIN_6}, 	{portA, GPIO_PIN_5},   {portA, GPIO_PIN_7},	 {portB, GPIO_PIN_7} },//ring
	{ {portA, GPIO_PIN_3}, 	{portA, GPIO_PIN_2},   {portB, GPIO_PIN_3},	 {portB, GPIO_PIN_8} }//little
};

static uint8_t buffer = 0;
static volatile uint8_t timeout_flg = 0;
static volatile enum_motor_stat home_routine[flength] = {lost};
static volatile enum_buffer_status buffer_status = empty;
static const enum_stepping stepping = sixteen_step;
static const uint8_t ackt[] = "\rThumb Reference Routine Complete\n";
static const uint8_t acki[] = "\rIndex Reference Routine Complete\n";
static const uint8_t ackm[] = "\rMiddleReference Routine Complete\n";
static const uint8_t ackr[] = "\rRing Reference Routine Complete\n";
static const uint8_t ackl[] = "\rLittle Reference Routine Complete\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void read_action(st_action * action);
static uint8_t send_step_pulses(st_fposition * fposition, st_action * act);
static void alive_fn(void);
void start_action(st_fposition * fposition, st_action * act);
void clean_buffer(void);
void sleep_motor(enum_ports port, uint16_t pin);
void motor_wakeup(enum_ports port, uint16_t pin);
uint8_t is_finger_up(uint16_t finger_home_sensor);
void send_pulse(enum_ports port, uint16_t pin);
void set_direction(enum_ports port, uint16_t pin, GPIO_PinState dir);

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
	static st_fposition fposition[flength] = {{0, 0, NO}};
	st_action action = {0};
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

	HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));
	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(full == buffer_status)
		{
			read_action(&action);
			if(action.action == emergency_rst)
			{
				fposition[action.finger].op_has_started = NO;
				home_routine[action.finger] = lost;
			}
			else if(action.action % 2 == active)
			{
				start_action(&fposition[action.finger], &action);
			}
			else if(action.action % 2 == passive)
			{

			}
			else
			{
				fposition[action.finger].op_has_started = NO;
				home_routine[action.finger] = lost;
			}
			clean_buffer();
			HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));

		}

		if(home_routine[action.finger] == home)
		{
			sleep_motor(fconfig[action.finger].sleep.port, fconfig[action.finger].sleep.pin);
			fposition[action.finger].op_has_started = NO;
			fposition[action.finger].absolute_pos = 0;
			home_routine[action.finger] = referenced;
		}

		if(timeout_flg)
		{
			if(fposition[action.finger].op_has_started)
			{
				if(send_step_pulses(fposition, &action) == FINISH)
				{
					sleep_motor(fconfig[action.finger].sleep.port, fconfig[action.finger].sleep.pin);
					fposition[action.finger].op_has_started = NO;
				}
			}
			alive_fn();
			timeout_flg = 0;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(fconfig[thumb].home.pin == GPIO_Pin)
	{
		if(home_routine[thumb] == lost)
		{
			home_routine[thumb] = home;
			HAL_UART_Transmit(&huart3, (const uint8_t *) ackt, sizeof(acki), 100);
		}
	}

	if(fconfig[index].home.pin == GPIO_Pin)
	{
		if(home_routine[index] == lost)
		{
			home_routine[index] = home;
			HAL_UART_Transmit(&huart3, (const uint8_t *) acki, sizeof(acki), 100);
		}
	}

	if(fconfig[middle].home.pin == GPIO_Pin)
	{
		if(home_routine[middle] == lost)
		{
			home_routine[middle] = home;
			HAL_UART_Transmit(&huart3, (const uint8_t *) ackm, sizeof(acki), 100);
		}
	}

	if(fconfig[ring].home.pin == GPIO_Pin)
	{
		if(home_routine[ring] == lost)
		{
			home_routine[ring] = home;
			HAL_UART_Transmit(&huart3, (const uint8_t *) ackr, sizeof(acki), 100);
		}
	}

	if(fconfig[little].home.pin == GPIO_Pin)
	{
		if(home_routine[little] == lost)
		{
			home_routine[little] = home;
			HAL_UART_Transmit(&huart3, (const uint8_t *) ackl, sizeof(acki), 100);
		}
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
		if(~timeout_flg)
		{
			timeout_flg = 1;
		}
	}
}

static void read_action(st_action * action)
{
	//compare byte
	switch(buffer){
	case '\e':
		action->action = emergency_rst;
		action->finger = flength;
		return;
	case 'f':

		action->action = moving_forward;
		action->finger = index;
		return;
	case 'b':
		action->action = moving_backward;
		action->finger = index;
		return;
	case 'h':
		action->action = reference_routine;
		action->finger = index;
		return;
	case 0:
		action->action = halt_state;
		return;
	case '+':
		action->action = halt_state;
		return;
	default:
		action->action = halt_state;
		return;
	}
}

static uint8_t send_step_pulses(st_fposition * fposition, st_action * act)
{

	if(fposition[act->finger].steps_to_go > 0)
	{
		switch(act->action)
		{
		case reference_routine:
			if(is_finger_up(fconfig[act->finger].home.pin)){
				return FINISH;
			}
			else
			{
				send_pulse(fconfig[act->finger].step.port, fconfig[act->finger].step.pin);
				return 0;
			}
		case moving_backward:
			if(is_finger_up(fconfig[act->finger].home.pin)){
				return FINISH;
			}
			else
			{
				send_pulse(fconfig[act->finger].step.port, fconfig[act->finger].step.pin);
				fposition[act->finger].absolute_pos--;
				fposition[act->finger].steps_to_go--;
				return 0;
			}

		case moving_forward:
			send_pulse(fconfig[act->finger].step.port, fconfig[act->finger].step.pin);
			fposition[act->finger].absolute_pos++;
			fposition[act->finger].steps_to_go--;
			return 0;
		default:
			fposition[act->finger].steps_to_go = 0;
			return FINISH;
		}

	}
	else{
		return FINISH;
	}
}

static void alive_fn(void)
{
	static uint16_t var = 16000;

	if(var > 0)
	{
		var--;
	}
	else{
		var = 16000;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

void start_action(st_fposition * fposition, st_action * act)
{
	switch(act->action)
	{
	case reference_routine:
		set_direction(fconfig[act->finger].direction.port, fconfig[act->finger].direction.pin, UP);
		home_routine[act->finger] = lost;
		motor_wakeup(fconfig[act->finger].sleep.port, fconfig[act->finger].sleep.pin);
		fposition->steps_to_go = HOME_STEPS;
		fposition->op_has_started = YES;
		break;
	case moving_backward:
		set_direction(fconfig[act->finger].direction.port, fconfig[act->finger].direction.pin, UP);
		motor_wakeup(fconfig[act->finger].sleep.port, fconfig[act->finger].sleep.pin);
		fposition->steps_to_go = dfl_steps << stepping;
		fposition->op_has_started = YES;
		break;
	case moving_forward:
		set_direction(fconfig[act->finger].direction.port, fconfig[act->finger].direction.pin, DOWN);
		motor_wakeup(fconfig[act->finger].sleep.port, fconfig[act->finger].sleep.pin);
		fposition->steps_to_go = dfl_steps << stepping;
		fposition->op_has_started = YES;
		break;
	default:
		fposition->steps_to_go = 0;
		fposition->op_has_started = NO;
		break;
	}

}

void clean_buffer()
{
	buffer = 0;
	buffer_status = empty;
}

void sleep_motor(enum_ports port, uint16_t pin)
{
	if(port == portA)
		HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);
}

void motor_wakeup(enum_ports port, uint16_t pin)
{
	if(port == portA)
		HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);
}


void send_pulse(enum_ports port, uint16_t pin)
{
	// if finger is index or middle or ring = portA
	// else if finger is thumb or little = portB
	//
	if(port == portA)
		HAL_GPIO_TogglePin(GPIOA, pin);
	else
		HAL_GPIO_TogglePin(GPIOB, pin);
}


uint8_t is_finger_up(uint16_t finger_home_sensor){
	return HAL_GPIO_ReadPin(GPIOB, finger_home_sensor);
}

void set_direction(enum_ports port, uint16_t pin, GPIO_PinState dir)
{
	if(port == portA)
		HAL_GPIO_WritePin(GPIOA, pin, dir);
	else
		HAL_GPIO_WritePin(GPIOB, pin, dir);
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