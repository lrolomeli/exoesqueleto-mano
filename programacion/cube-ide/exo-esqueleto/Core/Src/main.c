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
	Down
}enum_motion;

typedef enum{
	No = 0,
	Yes
}enum_in_operation;

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
	stopped = 0x7F
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
#define dfl_steps (500U)
#define UNKNOWN (65535U)
#define MAX_POSITION (24000U)
#define HOME_POSITION (0U)

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

static volatile uint8_t timeout_flg = 0;
static volatile enum_motor_stat home_routine[flength] = {lost};
static volatile enum_buffer_status buffer_status = empty;
static const enum_stepping stepping = sixteen_step;
//static const uint8_t ackt[] = "\rThumb Reference Routine Complete\n";
//static const uint8_t acki[] = "\rIndex Reference Routine Complete\n";
//static const uint8_t ackm[] = "\rMiddleReference Routine Complete\n";
//static const uint8_t ackr[] = "\rRing Reference Routine Complete\n";
//static const uint8_t ackl[] = "\rLittle Reference Routine Complete\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void set_direction(enum_ports port, uint16_t pin, GPIO_PinState dir);
static uint8_t is_finger_up(uint16_t finger_home_sensor);
static void send_pulse(enum_ports port, uint16_t pin);
static void motor_wakeup(enum_ports port, uint16_t pin);
static void sleep_motor(enum_ports port, uint16_t pin);
static void clean_buffer(uint8_t * buf);
static void sinewave_fn(st_exoesk * exoesk);
static void go_up_fn(st_exoesk * exoesk);
static void go_down_fn(st_exoesk * exoesk);
static void ref_routine_fn(st_exoesk * exoesk);
static void gotopos_fn(st_exoesk * exoesk, uint16_t position);
static void exo_prepare(st_exoesk * exoesk, enum_motion direction, uint16_t position);
static void alive_fn(void);
static void send_step_pulses(st_exoesk * exoesk);
static void finger_motion(st_exoesk * exoesk);
static void toggle_finger(st_exoesk * exoesk, enum_action act);
static void exo_init(st_exoesk * exoesk);
static void home_routine_fn(st_exoesk * exoesk);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
func_ptr_t exo_preactions[4] = {ref_routine_fn, go_down_fn, go_up_fn, sinewave_fn};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(exoconfig[thumb].home.pin == GPIO_Pin)
	{
		if(home_routine[thumb] == lost)
		{
			sleep_motor(exoconfig[thumb].sleep.port, exoconfig[thumb].sleep.pin);
			home_routine[thumb] = home;
		}
	}

	else if(exoconfig[index].home.pin == GPIO_Pin)
	{
		if(home_routine[index] == lost)
		{
			sleep_motor(exoconfig[index].sleep.port, exoconfig[index].sleep.pin);
			home_routine[index] = home;
		}
	}

	else if(exoconfig[middle].home.pin == GPIO_Pin)
	{
		if(home_routine[middle] == lost)
		{
			sleep_motor(exoconfig[middle].sleep.port, exoconfig[middle].sleep.pin);
			home_routine[middle] = home;
		}
	}

	else if(exoconfig[ring].home.pin == GPIO_Pin)
	{
		if(home_routine[ring] == lost)
		{
			sleep_motor(exoconfig[ring].sleep.port, exoconfig[ring].sleep.pin);
			home_routine[ring] = home;
		}
	}

	else if(exoconfig[little].home.pin == GPIO_Pin)
	{
		if(home_routine[little] == lost)
		{
			sleep_motor(exoconfig[little].sleep.port, exoconfig[little].sleep.pin);
			home_routine[little] = home;
		}
	}

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
	enum_action act = reference_routine;
	st_exoesk exoesk = {{UNKNOWN,UNKNOWN,UNKNOWN,UNKNOWN,UNKNOWN}, {0}, {0}, No};

	uint8_t buffer = 0;
	uint8_t position_flag = 0;
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
	exo_init(&exoesk);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(full == buffer_status)
		{
			act = buffer;
			if(position_flag)
			{
				gotopos_fn(&exoesk, (act-1)<<sixteen_step);
				position_flag = 0;
			}
			else
			{
				if(act > not_used && act < gotoposition)
				{
					exo_preactions[act-1](&exoesk);
				}
				else if(act == gotoposition)
				{
					// set flag for entering position
					position_flag = 1;
				}
				else if(act > nothing && act < stopped)
				{
					toggle_finger(&exoesk, act);
				}
				else if(act == reserved_major_C)
				{
					uint8_t finger=0;
					for(finger=0; finger<flength; finger++)
					{
						exoesk.fingers_in_op[finger] = No;
					}
				}
				else
				{

				}
			}

			clean_buffer(&buffer);
			HAL_UART_Receive_IT(&huart3, &buffer, sizeof(buffer));

		}

		// In case any finger touches home button
		// that finger has to stop moving
		uint8_t finger=0;
		for(finger=0; finger<flength; finger++)
		{
			if(home_routine[finger] == home)
			{
				sleep_motor(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
				exoesk.absolute_pos[finger] = HOME_POSITION;
				home_routine[finger] = referenced;
			}

		}

		if(timeout_flg)
		{
			finger_motion(&exoesk);
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

static void finger_motion(st_exoesk * exoesk)
{

	if(exoesk->in_operation)
	{
		send_step_pulses(exoesk);
	}

}

static void send_step_pulses(st_exoesk * exoesk)
{

	uint8_t finger = 0;
	uint8_t counter = 0;

	for(finger=0; finger<flength; finger++)
	{
		// if the motor finger has reached the go_to position it sleeps
		// as well as if the motor is moving back and the finger has reached the home position
		// or in other words if the go_to position is smaller than absolute position and the finger has reached the home position
		// then it has to stop, not if the motor is going on the other way.
		if((exoesk->absolute_pos[finger] == exoesk->go_to[finger]) || (is_finger_up(exoconfig[finger].home.pin) && exoesk->go_to[finger] < exoesk->absolute_pos[finger]))
		{
			sleep_motor(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
			counter++;
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
	if(counter < flength)
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
	static uint16_t var = 16000;

	if(var > 0)
	{
		var--;
	}
	else
	{
		var = 16000;
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

static void ref_routine_fn(st_exoesk * exoesk)
{
	home_routine_fn(exoesk);
}
static void go_down_fn(st_exoesk * exoesk)
{
	exo_prepare(exoesk, Down, dfl_steps << stepping);
}
static void go_up_fn(st_exoesk * exoesk)
{
	exo_prepare(exoesk, Up, dfl_steps << stepping);
}
static void sinewave_fn(st_exoesk * exoesk)
{

}

static void home_routine_fn(st_exoesk * exoesk)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		if(exoesk->fingers_in_op[finger])
		{
			if(is_finger_up(home_routine[finger]))
			{
				exoesk->absolute_pos[finger] = HOME_POSITION;
				home_routine[finger] = referenced;
			}
			else
			{
				set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, Up);
				exoesk->absolute_pos[finger] = UNKNOWN;
				motor_wakeup(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);
				exoesk->go_to[finger] = HOME_POSITION;
				home_routine[finger] = lost;
			}
		}
	}
	exoesk->in_operation = Yes;
}

static void exo_prepare(st_exoesk * exoesk, enum_motion direction, uint16_t steps_to_go)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		if(exoesk->fingers_in_op[finger])
		{
			set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, direction);
			motor_wakeup(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);

			if(direction == Down && exoesk->absolute_pos[finger] <= (MAX_POSITION-steps_to_go))
			{
				exoesk->go_to[finger] += steps_to_go;
			}
			// Si vas para arriba solo checa que lo que vas a avanzar no sea mas de lo permitido
			// es decir que la posicion en la que estas + lo que vas a avanzar
			else if(direction == Up && exoesk->absolute_pos[finger] >= steps_to_go )
			{
				exoesk->go_to[finger] -= steps_to_go;
			}
			else{

			}


		}
	}
	exoesk->in_operation = Yes;
}

static void exo_init(st_exoesk * exoesk)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
#ifdef test
		exoesk->fingers_in_op[index] = Yes;
#else
		exoesk->fingers_in_op[finger] = Yes;
#endif
	}

	home_routine_fn(exoesk);

	for(uint8_t finger=thumb; finger<flength; finger++)
	{
#ifdef test
		exoesk->fingers_in_op[index] = No;
#else
		exoesk->fingers_in_op[finger] = No;
#endif
	}

}

static void gotopos_fn(st_exoesk * exoesk, uint16_t position)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		enum_motion direction;
		if(position > exoesk->absolute_pos[finger])
		{
			direction = Down;
			exoesk->go_to[finger] = position - exoesk->absolute_pos[finger];
		}
		else
		{
			direction = Up;
			exoesk->go_to[finger] = exoesk->absolute_pos[finger] - position;
		}

		if((*(exoesk->fingers_in_op) & (1<<finger)))
		{
			set_direction(exoconfig[finger].direction.port, exoconfig[finger].direction.pin, direction);
			motor_wakeup(exoconfig[finger].sleep.port, exoconfig[finger].sleep.pin);

		}
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


static uint8_t is_finger_up(uint16_t finger_home_sensor)
{
	return HAL_GPIO_ReadPin(GPIOB, finger_home_sensor);
}

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
