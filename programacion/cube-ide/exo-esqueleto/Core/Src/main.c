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
}enum_bit_state;

typedef enum{
	Unreferenced = 0,
	Referenced
}enum_referenced;

typedef enum{
	ReleaseHomeButtons = 0,
	WaitReleaseStage,
	GoTowardsHomeButtons,
	GoInitialPosition,
	HomeRoutineComplete,
	ErrorHoming
}enum_home_stages;


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
	stop = 0x7F,
	speed_0 = 0xFA,
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

typedef enum{
	home_fs = 0,
	idle_fs,

}enum_fsm_state;

typedef struct{
	enum_bit_state in_operation;
	enum_fsm_state fsm_state;
	uint8_t bluetooth_command;
}st_exoesk;

typedef struct{
	uint16_t current_pos[flength];
	uint16_t go_to[flength];
	enum_bit_state fingers_in_op[flength];
	enum_bit_state fingers_in_pos[flength];
}st_gfinger_params;

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

static uint8_t times = 0;
static volatile uint8_t timeout_flg = 0;
static volatile enum_motor_status home_routine[flength] = {lost};
static volatile enum_buffer_status buffer_status = empty;
static volatile st_gfinger_params gfinger_params = { {0},{0},{0},{0} };
static const uint8_t hfingers[9] = {index,middle,ring,0,little,0,0,0,thumb};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void set_direction(uint8_t finger, GPIO_PinState dir);
static void sleep_motor(uint8_t finger);
static void motor_wakeup(uint8_t finger);
static void send_pulse(uint8_t finger);
static void clean_buffer(uint8_t * buf);
static void sinewave_fn(st_exoesk * exoesk);
static void preset_fingers_target(st_exoesk * exoesk, uint16_t position);
static void alive_fn(void);
static void send_step_pulses(st_exoesk * exoesk);
static void toggle_finger(uint8_t btcmd);
static uint8_t is_system_referenced(void);
static void send_home(st_exoesk * exoesk);
static void select_all_fingers(st_exoesk * exoesk);
static void deselect_all_fingers(st_exoesk * exoesk);
static void prepare_action(st_exoesk * exoesk);
static void dynamics(st_exoesk * exoesk);
static void home_f(st_exoesk * exoesk);
static void idle_f(st_exoesk * exoesk);
static void finger_default_conditions();
static uint8_t process_cmd(st_exoesk * exoesk);
static uint8_t fingers_in_position(void);
static void emergency_stop(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const func_ptr_t fsm_state[2] = {home_f, idle_f};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t finger = 0;

	GPIO_Pin = GPIO_Pin >> 6;
	finger = hfingers[GPIO_Pin];

	if(home_routine[finger] == home)
	{
		// Si el dedo se dirige hacia home actuamos
		sleep_motor(finger);
		gfinger_params.fingers_in_pos[finger] = Yes;
		gfinger_params.current_pos[finger] = HOME_POSITION;
		home_routine[finger] = referenced;
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
		if(timeout_flg==0)
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
	uint8_t halfms_cnt=0;
	st_exoesk exoesk = {No, home_fs, 0};
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

	HAL_UART_Receive_IT(&huart3, &exoesk.bluetooth_command, sizeof(exoesk.bluetooth_command));
	htim3.Init.Period = htim3.Init.Period >> (normal);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		fsm_state[exoesk.fsm_state](&exoesk);

		if(timeout_flg)
		{

			if (halfms_cnt < times)
			{

				halfms_cnt++;
			}
			else
			{
				dynamics(&exoesk);
				halfms_cnt = 0;
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

static void home_f(st_exoesk * exoesk)
{
	static enum_home_stages home_stage = ReleaseHomeButtons;
	static uint16_t failure_blinking_delay = 10000;

	switch(home_stage)
	{
	case ReleaseHomeButtons:
		// Seleccionamos los dedos que queremos mover en este caso todos.
		select_all_fingers(exoesk);
		// Definimos el objetivo al que hay que movernos. Los pasos necesarios para despejar el boton.
		preset_fingers_target(exoesk, Clear_Button_Distance);
		// Esperar a que se despejen los motores
		home_stage = WaitReleaseStage;
		break;
	case WaitReleaseStage:
		if(No == exoesk->in_operation)
		{
			// Definimos el objetivo al que hay que movernos. Los pasos necesarios para volver al boton
			send_home(exoesk);
			home_stage = GoTowardsHomeButtons;
		}
		break;
	case GoTowardsHomeButtons:
		if(is_system_referenced())
		{
			// Definimos el objetivo al que hay que movernos. Los pasos necesarios para despejar el boton.
			preset_fingers_target(exoesk, Initial_Position);
			home_stage = GoInitialPosition;
		}
		else if (exoesk->in_operation)
		{
			home_stage = GoTowardsHomeButtons;
		}
		else
		{
			emergency_stop();
			finger_default_conditions();
			home_stage = ErrorHoming;
		}
		break;
	case GoInitialPosition:
		if(No == exoesk->in_operation)
		{
			deselect_all_fingers(exoesk);
			home_stage = HomeRoutineComplete;
			exoesk->fsm_state = idle_fs;
		}
		break;
	case HomeRoutineComplete:
		// Comenzamos con condiciones iniciales
		finger_default_conditions();
		home_stage = ReleaseHomeButtons;
		break;
	case ErrorHoming:
		if(failure_blinking_delay > 0)
		{
			failure_blinking_delay--;
		}
		else
		{
			failure_blinking_delay = 10000;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		break;
	default:
		break;
	}

}

static void idle_f(st_exoesk * exoesk)
{

	if(full == buffer_status)
	{
		prepare_action(exoesk);
		clean_buffer(&exoesk->bluetooth_command);
		HAL_UART_Receive_IT(&huart3, &exoesk->bluetooth_command, sizeof(exoesk->bluetooth_command));
	}

}

static void dynamics(st_exoesk * exoesk)
{
	if(exoesk->in_operation)
	{
		/* Move fingers */
		send_step_pulses(exoesk);
	}

}

static void prepare_action(st_exoesk * exoesk)
{
	static uint8_t cmd_complete = 1;
	uint16_t steps = 0;

	if(cmd_complete == 0)
	{
		steps = exoesk->bluetooth_command * 25;
		preset_fingers_target(exoesk, steps);
		cmd_complete = 1;
	}
	else
	{
		cmd_complete = process_cmd(exoesk);
	}
}

static uint8_t process_cmd(st_exoesk * exoesk)
{
	switch(exoesk->bluetooth_command)
	{
	case not_used:
		break;
	case reference_routine:
		exoesk->fsm_state = home_fs;
		break;
	case all_way_down:
		preset_fingers_target(exoesk, MAX_POSITION);
		break;
	case all_way_up:
		preset_fingers_target(exoesk, HOME_POSITION);
		break;
	case sinewave:
		sinewave_fn(exoesk);
		break;
	case gotoposition:
		return 0;
		break;
	case reserved_major_C:
		break;
	case deselect_thumb:
	case deselect_index:
	case deselect_middle:
	case deselect_ring:
	case deselect_little:
	case select_thumb:
	case select_index:
	case select_middle:
	case select_ring:
	case select_little:
		toggle_finger(exoesk->bluetooth_command);
		break;
	case stop:
		exoesk->in_operation = No;

		break;
	case speed_1:
	case speed_2:
	case speed_3:
	case speed_4:
	case speed_5:
		times = exoesk->bluetooth_command - speed_0;
		break;
	default:
		break;
	}
	return 1;

}

static void select_all_fingers(st_exoesk * exoesk)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		gfinger_params.fingers_in_op[finger] = Yes;
	}
}

static void deselect_all_fingers(st_exoesk * exoesk)
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		gfinger_params.fingers_in_op[finger] = Yes;
	}
}

static void send_home(st_exoesk * exoesk)
{
	uint8_t finger=thumb;
	for(finger=thumb; finger<flength; finger++)
	{
		gfinger_params.current_pos[finger] = UNDEFINED;
		gfinger_params.go_to[finger] = HOME_POSITION;

		home_routine[finger] = home;
		gfinger_params.fingers_in_pos[finger] = No;

		set_direction(finger, Up);
		motor_wakeup(finger);
	}
	exoesk->in_operation = Yes;
}

static void toggle_finger(uint8_t btcmd)
{

	uint8_t finger = (btcmd - deselect_thumb);

	if(finger < flength){
		gfinger_params.fingers_in_op[finger % flength] = Yes;
	}
	else
	{
		gfinger_params.fingers_in_op[finger % flength] = No;
	}

}

static uint8_t is_system_referenced(void)
{
	if(home_routine[thumb] == referenced &&
		home_routine[index] == referenced &&
		home_routine[middle] == referenced &&
		home_routine[ring] == referenced &&
		home_routine[little] == referenced)
	{
		return Yes;
	}
	else
	{
		return No;
	}
}

static void send_step_pulses(st_exoesk * exoesk)
{
	uint8_t finger = thumb;
	for(finger = thumb; finger < flength; finger++)
	{
		if(gfinger_params.fingers_in_op[finger])
		{
			/* when finger is selected to be moved */
			if(gfinger_params.go_to[finger] == gfinger_params.current_pos[finger])
			{
				/* when finger has reached expected position */
				sleep_motor(finger);
				gfinger_params.fingers_in_pos[finger] = Yes;
			}
			else
			{
				/* finger has not reached expected position */
				send_pulse(finger);
				if(gfinger_params.go_to[finger] > gfinger_params.current_pos[finger])
				{
					gfinger_params.current_pos[finger]++;
				}
				else
				{
					gfinger_params.current_pos[finger]--;
				}
			}
		}
	}

	/* if all fingers are in position we should sleep the system */
	if(fingers_in_position())
	{
		exoesk->in_operation = No;
	}
	else
	{
		exoesk->in_operation = Yes;
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

static void finger_default_conditions()
{
	for(uint8_t finger=thumb; finger<flength; finger++)
	{
		/* el dedo no esta en operacion */
		gfinger_params.fingers_in_op[finger] = No;
		/* la rutina de home no se ha completado */
		home_routine[finger] = lost;
		/* no se conoce la posicion del motor */
		gfinger_params.current_pos[finger] = NOT_KNOWN_POSITION;
		/* no se ha determinado la posicion a la que vamos a movernos */
		gfinger_params.go_to[finger] = NOT_KNOWN_POSITION;
		/* el dedo esta en cierta posicion y por lo tanto no se esta moviendo */
		gfinger_params.fingers_in_pos[finger] = Yes;
	}
}

static void sinewave_fn(st_exoesk * exoesk)
{

}

/**
 * preset_fingers_target function
 *
 * 1) Revisa cual dedo se encuentra habilitado para moverse.
 * 2) Establece el objetivo al que hay que mover cada dedo
 * 3)
 *
 */
static void preset_fingers_target(st_exoesk * exoesk, uint16_t position)
{
	uint8_t finger = thumb;
	for(finger=thumb; finger<flength; finger++)
	{
		if(gfinger_params.fingers_in_op[finger])
		{
			gfinger_params.fingers_in_pos[finger] = No;
			gfinger_params.go_to[finger] = position;
			// things to prevent
			if (gfinger_params.current_pos[finger] < position)
			{
				// goto position is not greater than maximum position
				if(MAX_POSITION < position)
				{
					gfinger_params.go_to[finger] = MAX_POSITION;
				}
				set_direction(finger, Down);
				motor_wakeup(finger);
			}
			else if(gfinger_params.current_pos[finger] > position)
			{
				set_direction(finger, Up);
				motor_wakeup(finger);
			}
			else
			{
				// current position is not the same as goto position
				gfinger_params.go_to[finger] = gfinger_params.current_pos[finger];
				gfinger_params.fingers_in_pos[finger] = Yes;
			}
		}
	}

	/* if all fingers are in position we should do nothing */
	if(fingers_in_position())
	{
		exoesk->in_operation = No;
	}
	else
	{
		exoesk->in_operation = Yes;
	}

}

static uint8_t fingers_in_position()
{
	return (gfinger_params.fingers_in_pos[thumb] &&
			gfinger_params.fingers_in_pos[index] &&
			gfinger_params.fingers_in_pos[middle] &&
			gfinger_params.fingers_in_pos[ring] &&
			gfinger_params.fingers_in_pos[little]);
}

static void clean_buffer(uint8_t * buf)
{
	*buf = 0;
	buffer_status = empty;
}

static void emergency_stop(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

static void sleep_motor(uint8_t finger)
{
	if(exoconfig[finger].sleep.port == portA)
		HAL_GPIO_WritePin(GPIOA, exoconfig[finger].sleep.pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOB, exoconfig[finger].sleep.pin, GPIO_PIN_RESET);
}

static void motor_wakeup(uint8_t finger)
{
	if(exoconfig[finger].sleep.port == portA)
		HAL_GPIO_WritePin(GPIOA, exoconfig[finger].sleep.pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, exoconfig[finger].sleep.pin, GPIO_PIN_SET);
}

static void send_pulse(uint8_t finger)
{
	if(exoconfig[finger].step.port == portA)
		HAL_GPIO_TogglePin(GPIOA, exoconfig[finger].step.pin);
	else
		HAL_GPIO_TogglePin(GPIOB, exoconfig[finger].step.pin);
}

static void set_direction(uint8_t finger, GPIO_PinState dir)
{

	if(exoconfig[finger].direction.port == portA)
		HAL_GPIO_WritePin(GPIOA, exoconfig[finger].direction.pin, dir);
	else
		HAL_GPIO_WritePin(GPIOB, exoconfig[finger].direction.pin, dir);
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
