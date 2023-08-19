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
	Up = 0,
	Down,
	None
}enum_motion;

typedef enum{
	No = 0,
	Yes
}enum_in_operation;

typedef enum{
	Unreferenced = 0,
	Referenced
}enum_referenced;


typedef enum{
	not_used = 0x00,
	reference_routine = 0x01,
	all_way_down  = 0x02,
	all_way_up  = 0x03,
	sinewave  = 0x04,
	gotoposition  = 0x05,
	reserved_plus=0x2B,
	reserved_minus=0x2D,
	reserved_major_C = 0x43,
	reserved_major_D = 0x44,
	nothing=0x74,
	deselect_thumb = 0x75,
	deselect_index = 0x76,
	deselect_middle = 0x77,
	deselect_ring = 0x78,
	deselect_little = 0x79,
	select_thumb = 0x7A,
	select_index = 0x7B,
	select_middle = 0x7C,
	select_ring = 0x7D,
	select_little = 0x7E,
	stopped = 0x7F,
	speed_1 = 0xFB,
	speed_2 = 0xFC,
	speed_3 = 0xFD,
	speed_4 = 0xFE,
	speed_5 = 0xFF
}enum_action;

typedef enum{
	lost = 0,
	home,
	referenced
}enum_motor_status;

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

typedef enum{
	normal = 0,
	fast = 1,
	as_hell

}enum_speed;

typedef struct{
	uint16_t absolute_pos[flength];
	uint16_t go_to[flength];
	uint8_t fingers_in_op[flength];
	enum_in_operation in_operation;
}st_exoesk;

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

typedef void(*func_ptr_t)(st_exoesk * exoesk);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define test

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static const st_fconfig exoconfig[flength] = {
	/*       steps               direction                sleep                 home          */
	{ {portA, GPIO_PIN_1}, 	{portA, GPIO_PIN_0},   {portB, GPIO_PIN_4},	 {portB, GPIO_PIN_9} },//thumb
	{ {portB, GPIO_PIN_14},	{portB, GPIO_PIN_13},  {portB, GPIO_PIN_15}, {portB, GPIO_PIN_5} },//index
	{ {portB, GPIO_PIN_1}, 	{portB, GPIO_PIN_0},   {portA, GPIO_PIN_8},	 {portB, GPIO_PIN_6} },//middle
	{ {portA, GPIO_PIN_6}, 	{portA, GPIO_PIN_5},   {portA, GPIO_PIN_7},	 {portB, GPIO_PIN_7} },//ring
	{ {portA, GPIO_PIN_3}, 	{portA, GPIO_PIN_2},   {portB, GPIO_PIN_3},	 {portB, GPIO_PIN_8} }//little
};

#ifdef test
static enum_fingers finger_under_test = ring;
#endif

static volatile uint8_t timeout_flg = 0;
static volatile enum_motor_status home_routine[flength] = {lost};
static volatile enum_buffer_status buffer_status = empty;
//static const enum_stepping stepping = sixteen_step;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void set_direction(enum_ports port, uint16_t pin, GPIO_PinState dir);
//static uint8_t is_finger_up(uint16_t finger_home_sensor);
static void send_pulse(enum_ports port, uint16_t pin);
static void motor_wakeup(enum_ports port, uint16_t pin);
static void sleep_motor(enum_ports port, uint16_t pin);
static void clean_buffer(uint8_t * buf);
static void sinewave_fn(st_exoesk * exoesk);
static void go_up_fn(st_exoesk * exoesk);
static void go_down_fn(st_exoesk * exoesk);
static void ref_routine_fn(st_exoesk * exoesk);
static void gotopos_fn(st_exoesk * exoesk, uint16_t position);
static void exo_prepare(st_exoesk * exoesk, uint16_t position);
static void alive_fn(void);
static void send_step_pulses(st_exoesk * exoesk);
static void finger_motion(st_exoesk * exoesk);
static void toggle_finger(st_exoesk * exoesk, enum_action act);
static void go_first_down(st_exoesk * exoesk);
static uint8_t is_system_referenced(void);
static void send_home(st_exoesk * exoesk);
static void prepare_action(st_exoesk * exoesk, uint8_t * pos_flag, enum_action act);
static void dynamics(st_exoesk * exoesk, uint8_t * exo_rdy_to_op, uint8_t * home_send);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
func_ptr_t exo_preactions[4] = {ref_routine_fn, go_down_fn, go_up_fn, sinewave_fn};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

#ifdef test
	if(exoconfig[finger_under_test].home.pin == GPIO_Pin)
	{
		if(home_routine[finger_under_test] == home)
		{
			sleep_motor(exoconfig[finger_under_test].sleep.port, exoconfig[finger_under_test].sleep.pin);
			home_routine[finger_under_test] = referenced;
		}
	}

#else

	if(exoconfig[index].home.pin == GPIO_Pin)
	{
		if(home_routine[index] == home)
		{
			sleep_motor(exoconfig[index].sleep.port, exoconfig[index].sleep.pin);
			home_routine[index] = referenced;
		}
	}

	if(exoconfig[thumb].home.pin == GPIO_Pin)
	{
		if(home_routine[thumb] == home)
		{
			sleep_motor(exoconfig[thumb].sleep.port, exoconfig[thumb].sleep.pin);
			home_routine[thumb] = referenced;
		}
	}



	if(exoconfig[middle].home.pin == GPIO_Pin)
	{
		if(home_routine[middle] == home)
		{
			sleep_motor(exoconfig[middle].sleep.port, exoconfig[middle].sleep.pin);
			home_routine[middle] = referenced;
		}
	}

	if(exoconfig[ring].home.pin == GPIO_Pin)
	{
		if(home_routine[ring] == home)
		{
			sleep_motor(exoconfig[ring].sleep.port, exoconfig[ring].sleep.pin);
			home_routine[ring] = referenced;
		}
	}

	if(exoconfig[little].home.pin == GPIO_Pin)
	{
		if(home_routine[little] == home)
		{
			sleep_motor(exoconfig[little].sleep.port, exoconfig[little].sleep.pin);
			home_routine[little] = referenced;
		}
	}

#endif

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//waits until buffer is empty again
	if(empty == buffer_status)
	{
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	st_exoesk exoesk = {{0}, {0}, {0}, No};
	uint8_t exo_rdy_to_op = No;
	uint8_t buffer = 0;
	uint8_t position_flag = 0;
	uint8_t home_send = No;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));
	htim3.Init.Period = htim3.Init.Period >> (normal);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	go_first_down(&exoesk);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(full == buffer_status && Yes == exo_rdy_to_op)
		{
			prepare_action(&exoesk, &position_flag, buffer);
			clean_buffer(&buffer);
			HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));
		}

		if (No == exo_rdy_to_op)
		{
			exo_rdy_to_op = is_system_referenced();
		}

		if(timeout_flg)
		{
			dynamics(&exoesk, &exo_rdy_to_op, &home_send);
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

void setSpeed(uint8_t speed){
	htim3.Init.Period = 5*speed;
}

static void dynamics(st_exoesk * exoesk, uint8_t * exo_rdy_to_op, uint8_t * home_send)
{
	if(exoesk->in_operation)
	{
		/* Move fingers */
		finger_motion(exoesk);
	}
	/* It is not intuitive but this means home routine can start
	 *
	 * Program begins with exoesk in operation which means
	 * fingers are initially moving downwards and only when they stop
	 * fingers will be down and home motion can begin
	 */
	else if(No == *exo_rdy_to_op && No == *home_send)
	{
		/* No key button is pressed at this point */
		send_home(exoesk);
		*home_send = Yes;
	}
	else
	{
		exoesk->in_operation = No;
	}
}

static void prepare_action(st_exoesk * exoesk, uint8_t * pos_flag, enum_action act)
{
	uint8_t speed = 1;
	uint16_t steps = 0;

	//al inicio la bandera de posicion es 0
	if(*pos_flag>0)
	{
		steps = act * 25;
		gotopos_fn(exoesk, steps);
		*pos_flag = 0;
	}
	else
	{
		if(act > not_used && act < gotoposition)
		{
			exo_preactions[act-1](exoesk);
		}
		else if(act == gotoposition)
		{
			// set flag for entering position
			*pos_flag = 1;
		}
		else if(act > nothing && act < stopped)
		{
			toggle_finger(exoesk, act);
		}
		else if(act == reserved_major_C)
		{
			uint8_t finger=0;
			for(finger=0; finger<flength; finger++)
			{
				exoesk->fingers_in_op[finger] = No;
			}
		}
		else if(act >= speed_1 && act <= speed_5)
		{
			switch(act)
			{
			case speed_1:
				speed = 1;
				break;
			case speed_2:
				speed = 2;
				break;
			case speed_3:
				speed = 3;
				break;
			case speed_4:
				speed = 4;
				break;
			case speed_5:
				speed = 5;
				break;
			default:
				break;
			}
			setSpeed(speed);
		}
		else
		{

		}
	}
}

static void send_home(st_exoesk * exoesk)
{
#ifdef test
	exoesk->fingers_in_op[finger_under_test] = Yes;
	set_direction(exoconfig[finger_under_test].direction.port, exoconfig[finger_under_test].direction.pin, Up);
	motor_wakeup(exoconfig[finger_under_test].sleep.port, exoconfig[finger_under_test].sleep.pin);
	exoesk->absolute_pos[finger_under_test] = UNKNOWN;
	exoesk->go_to[finger_under_test] = HOME_POSITION;
	home_routine[finger_under_test] = home;
	exoesk->in_operation = Yes;
#else
	uint8_t finger=0;
	for(finger=0; finger<flength; finger++)
	{
		exoesk->fingers_in_op[finger] = Yes;
		set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, Up);
		motor_wakeup(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
		exoesk->absolute_pos[finger] = UNKNOWN;
		exoesk->go_to[finger] = HOME_POSITION;
		home_routine[finger] = home;
	}
	exoesk->in_operation = Yes;
#endif

}

static void toggle_finger(st_exoesk * exoesk, enum_action act)
{
	uint8_t finger = (act - deselect_thumb);

	if(finger < flength){
		exoesk->fingers_in_op[finger % flength] = No;
	}
	else
	{
		exoesk->fingers_in_op[finger % flength] = Yes;
	}

}

static uint8_t is_system_referenced(void)
{
#ifdef test
		if(home_routine[finger_under_test] == referenced)
		{
			return Yes;
		}
#else
	if(home_routine[thumb] == referenced &&
		home_routine[index] == referenced &&
		home_routine[middle] == referenced &&
		home_routine[ring] == referenced &&
		home_routine[little] == referenced)
	{
		return Yes;
	}
#endif
	else
	{
		return No;
	}
}

static void finger_motion(st_exoesk * exoesk)
{
	if(exoesk->in_operation)
	{
		send_step_pulses(exoesk);
	}
}

static void send_step_pulses(st_exoesk * exoesk)
{
	uint8_t fingers_ready = 0;
#ifndef test
	uint8_t fingers_in_op = 0;
#endif
#ifdef test



	// case finger is active
	if(exoesk->fingers_in_op[finger_under_test])
	{
		if(exoesk->go_to[finger_under_test] == exoesk->absolute_pos[finger_under_test])
		{
			sleep_motor(exoconfig[finger_under_test].sleep.port, exoconfig[finger_under_test].sleep.pin);
			fingers_ready++;
		}
		else
		{
			send_pulse(exoconfig[finger_under_test].step.port, exoconfig[finger_under_test].step.pin);
			if(exoesk->go_to[finger_under_test] > exoesk->absolute_pos[finger_under_test])
			{
				exoesk->absolute_pos[finger_under_test]++;
			}
			else
			{
				exoesk->absolute_pos[finger_under_test]--;
			}

		}

	}
#else
	uint8_t finger = 0;
	for(finger=0; finger<flength; finger++)
	{
		// case finger is active
		if(exoesk->fingers_in_op[finger])
		{
			fingers_in_op++;
			if(exoesk->go_to[finger] == exoesk->absolute_pos[finger])
			{
				sleep_motor(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
				fingers_ready++;
			}
			else
			{
				send_pulse(exoconfig[finger].step.port, exoconfig[finger].step.pin);
				if(exoesk->go_to[finger] > exoesk->absolute_pos[finger])
				{
					exoesk->absolute_pos[finger]++;
				}
				else
				{
					exoesk->absolute_pos[finger]--;
				}

			}

		}

	}
#endif

#ifdef test
	if(fingers_ready < 1)
#else
	if(fingers_ready < fingers_in_op)
#endif
	{
		exoesk->in_operation = Yes;
	}
	else
	{
		exoesk->in_operation = No;
	}

}

static void alive_fn(void)
{
	static uint16_t var = 2000;

	if(var > 0)
	{
		var--;
	}
	else
	{
		var = 2000;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

static void ref_routine_fn(st_exoesk * exoesk)
{

}
static void go_down_fn(st_exoesk * exoesk)
{
	exo_prepare(exoesk, MAX_POSITION);
}
static void go_up_fn(st_exoesk * exoesk)
{
	exo_prepare(exoesk, HOME_POSITION);
}

static void gotopos_fn(st_exoesk * exoesk, uint16_t position)
{
	exo_prepare(exoesk, position);
}

static void sinewave_fn(st_exoesk * exoesk)
{

}

static void exo_prepare(st_exoesk * exoesk, uint16_t position)
{
#ifdef test

	if(exoesk->fingers_in_op[finger_under_test])
	{
		exoesk->go_to[finger_under_test] = position;
		if(exoesk->absolute_pos[finger_under_test] != exoesk->go_to[finger_under_test] && exoesk->go_to[finger_under_test] <= MAX_POSITION && exoesk->go_to[finger_under_test] >= HOME_POSITION)
		{
			if(exoesk->go_to[finger_under_test]>exoesk->absolute_pos[finger_under_test])
				set_direction(exoconfig[finger_under_test].direction.port, exoconfig[finger_under_test].direction.pin, Down);
			else
				set_direction(exoconfig[finger_under_test].direction.port, exoconfig[finger_under_test].direction.pin, Up);
			motor_wakeup(exoconfig[finger_under_test].sleep.port, exoconfig[finger_under_test].sleep.pin);
			exoesk->in_operation = Yes;
		}
	}

#else
	uint8_t fingers_prep = 0;
	uint8_t finger = thumb;
	for(finger=thumb; finger<flength; finger++)
	{
		if(exoesk->fingers_in_op[finger])
		{
			exoesk->go_to[finger] = position;
			// things to prevent
			// 1. that current position is not the same as to go position
			// 2. that to go position is not greather than maximum position
			// 3. that to go position is not smaller than minimum position
			if(exoesk->absolute_pos[finger] != exoesk->go_to[finger] && exoesk->go_to[finger] <= MAX_POSITION && exoesk->go_to[finger] >= HOME_POSITION)
			{
				if(exoesk->go_to[finger]>exoesk->absolute_pos[finger])
					set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, Down);
				else
					set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, Up);
				motor_wakeup(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
				exoesk->go_to[finger] = position;
				fingers_prep++;
			}
		}
	}
	if(fingers_prep > 0)
	{
		exoesk->in_operation = Yes;
	}
#endif

}

static void go_first_down(st_exoesk * exoesk)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		exoesk->fingers_in_op[finger] = Yes;
		exoesk->go_to[finger] = Home_Steps;
		exoesk->absolute_pos[finger] = HOME_POSITION;
		set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, Down);
		motor_wakeup(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
	}

	exoesk->in_operation = Yes;
}

static void clean_buffer(uint8_t * buf)
{
	*buf = 0;
	buffer_status = empty;
}

static void sleep_motor(enum_ports port, uint16_t pin)
{
	if(port == portA)
		HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);
}

static void motor_wakeup(enum_ports port, uint16_t pin)
{
	if(port == portA)
		HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);
}


static void send_pulse(enum_ports port, uint16_t pin)
{
	if(port == portA)
		HAL_GPIO_TogglePin(GPIOA, pin);
	else
		HAL_GPIO_TogglePin(GPIOB, pin);
}


//static uint8_t is_finger_up(uint16_t finger_home_sensor)
//{
//	return HAL_GPIO_ReadPin(GPIOB, finger_home_sensor);
//}

static void set_direction(enum_ports port, uint16_t pin, GPIO_PinState dir)
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
