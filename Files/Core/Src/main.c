/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#include <stdio.h>
#include <stdlib.h>
#include "shell.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define STACK_SIZE 1000
#define TAB_SIZE 10
/*
 * Variables defined to allow various controls
 */
uint32_t actual_alpha = 512;
uint32_t target_alpha= 512;
uint32_t aimed_speed;
/*
 * Defining buffers and variables to realize the calculus of speed values
 */
float error_speed[3];
float send_speed[3];
float ppt=1;
float pos[3];
float speed_buf[3];
float t_alpha;
/*
 * Defining PID control Gains and constants
 */
float KP = 0.06;
float KI = 0.05;
float KD = 0.01;
float Te=0.000556;
/*
 * Interruptions variables required to identification
 */
int Button_pressed;
int it_count=0;
int select_control;
static char prompt[] = "Guerin-Perret@Nucleo-G431>> ";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Pinout description accesible from shell command line
 */
int pinout(int argc, char ** argv){

	printf("PIN PA2 : USART2_TX --> USART connection to the computer\r\n");
	printf("PIN PA3 : USART2_RX --> USART connection to the computer\r\n");
	printf("PIN PC13 : GPIO_EXTI13 --> User button to perform the ignition sequence \r\n");
	printf("PIN PC1 : TIM1_CH2 --> Command_R_TOP / chopper PIN 13 \r\n");
	printf("PIN PB0 : TIM1_CH2N --> Command_R_BOT / chopper PIN 31\r\n");
	printf("PIN PC0 : TIM1_CH1 --> Command_Y_TOP / chopper PIN 12 \r\n");
	printf("PIN PA7 : TIM1_CH1N --> Command_Y_BOT / chopper PIN 30 \r\n");
	printf("PIN PC2 : GPIO_OUTPUT --> Reset_Driver / chopper PIN 33 \r\n");
	printf("PIN PA4 : GPIO_EXTI4 --> encoder Z output \r\n");
	printf("PIN PA0 : TIM2_CH1 --> Encoder A output \r\n");
	printf("PIN PA1 : TIM2_CH2 --> Encoder B output \r\n");
	return 0;
}
/*
 * Timer description to justify their use
 */
int timer(int argc, char ** argv){
	printf("TIM1 : PWM \r\n");
	printf("TIM2: Encoder mode \r\n");
	printf("TIM4 : asserv courant basique\r\n");
	printf("TIM6 : count in second \r\n");
	return 0;
}

int start(int argc, char ** argv){
	//Initiate Reset on driver Sequence
	printf("Initiate Reset pin \r\n");
	HAL_GPIO_WritePin(Reset_driver_GPIO_Port, Reset_driver_Pin, 0);
	HAL_GPIO_WritePin(Reset_driver_GPIO_Port, Reset_driver_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Reset_driver_GPIO_Port, Reset_driver_Pin, 0);
	//Starts PWM generation for full H bridge
	printf("Initiate PWM command \r\n");
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	//Initiate encoder mode with two channels available
	printf("Initialize encoder mode \r\n");
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	//Begin buffer init
	pos[0]=0;
	pos[1]=0;
	pos[2]=0;
	printf("Initialized position values \r\n");
	speed_buf[0]=0;
	speed_buf[1]=0;
	speed_buf[2]=0;
	error_speed[0]=0;
	error_speed[1]=0;
	error_speed[2]=0;
	send_speed[0]=0;
	send_speed[1]=0;
	send_speed[2]=0;
	return 0;
}
/*
 * This function aims at stopping gently the moto without provoking HALL Overcurrent situations
 */
int stop(int argc, char ** argv){
	printf("Shutting down motor\r\n");
	aimed_speed=0;
	if( (float)aimed_speed!=send_speed[2]){
		HAL_TIM_Base_Start_IT(&htim4);
	}
	TIM1->CCR1=512;
	TIM1->CCR2=512;
	return 0;
}
/*
 *This function initiate regularly interruptions calling upon another function to realise current control.
 */
int speed(int argc, char ** argv){
	select_control=0;
	target_alpha = atoi(argv[3]);
	printf("La vitesse demandée est %lu \r\n", target_alpha);
	printf("Target speed %lu \r\n", target_alpha);
	printf("Actual speed %lu \r\n", actual_alpha);
	if(actual_alpha!=target_alpha){
		TIM1->CCR1=actual_alpha;
		TIM1->CCR2=1024-actual_alpha;
		HAL_TIM_Base_Start_IT(&htim4);
	}
	return 0;
}
/*
 * This function by setting up the speed higher and higher each iteration allows the user to realize a current control
 * This current control is made by hand and do know use a PID method
 */
int auto_speedup(){
	if(actual_alpha<target_alpha){
		actual_alpha++;
		TIM1->CCR1=actual_alpha;
		TIM1->CCR2=1024-actual_alpha;
		printf("Actual speed = %lu \r\n", actual_alpha);
	}
	else if(actual_alpha>target_alpha){
		actual_alpha--;
		TIM1->CCR1=actual_alpha;
		TIM1->CCR2=1024-actual_alpha;
		printf("Actual speed = %lu \r\n", actual_alpha);
	}
	else{
		HAL_TIM_Base_Start_IT(&htim4);
	}
	return 0;
}
/*
 * This pid_auto method was not tested on the real motor but on much smaller ones, the steps are similars but it would be
 * strange (and lucky) that the pid works as expect on the first attempt. If the pid is not well configured, parameters can
 * be changed above.
 */
int pid_auto(int argc, char ** argv){
	select_control=1;
	aimed_speed = atoi(argv[1]);
	printf("target speed = %lu \r\n", aimed_speed );
	if(aimed_speed!=speed_buf[1])	{
		HAL_TIM_Base_Start_IT(&htim4);

	}else{
		printf("Speed already achieved \r\n");
	}
	return 0;

}
/*
 * Function called by pid_auto(), the strategy is to memorize in a buffer old values and to follow the numerical equation given
 * by the automation course. Once the speed which needs to be sent to the machine is calculated, a conversion to cycle is made.
 * This conversion is based on the fact that when the cycle equals 1024, mean voltage equals 48V and speed 3000 rpm. Here from
 * the speed, we want to calculate the value of the cycle. Negative speeds are not recommanded, since we did not have the time
 * to adapt.
 */
int pid(){
	printf("Target value is %lu \r\n", aimed_speed);
	pos[0]=pos[1];
	pos[1]=pos[2];
	pos[2]=(TIM2->CNT)>>2;
	speed_buf[0]=speed_buf[1];
	speed_buf[1]=speed_buf[2];
	speed_buf[2]=(pos[2]-pos[1])*100/(100000*ppt);
	error_speed[0]=error_speed[1];
	error_speed[1]=error_speed[2];
	error_speed[2]=aimed_speed-speed_buf[1];
	send_speed[0]=send_speed[1];
	send_speed[1]=send_speed[2];
	send_speed[2]=KP*error_speed[2]-KP*error_speed[0]+KI*(Te/2)*error_speed[2]-KI*Te*error_speed[1]+error_speed[0]+KD*(Te/2)*error_speed[2]-2*KD*(Te/2)*error_speed[1]+KD*error_speed[0]+send_speed[0];
	t_alpha=abs(512+(uint32_t)(send_speed[2]/1024*3000));
	if(t_alpha>810){
		TIM1->CCR1=810;
		HAL_TIM_Base_Start_IT(&htim4);
	}if(t_alpha<810){
		TIM1->CCR1=t_alpha;
		HAL_TIM_Base_Start_IT(&htim4);
	}if(send_speed[2]==(float)aimed_speed){
		printf("Target speed reached, motor is running at %lu \r\n",aimed_speed);
		HAL_TIM_Base_Stop_IT(&htim4);
	}

	return 0;
}



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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//Creating shell command line
	shell_init();
	shell_add('P', pinout, "*Pinout : Display of used pins* \r\n -->Command : <P> \r\n -->Output : lists all used pins and their uses. \r\n");
	shell_add('T', timer, "*Timer : Display of used pins* \r\n -->Command : <T> \r\n -->Output : lists all used timers and their uses. \r\n");
	shell_add('G', start, "*Go - Start* \r\n -->Command : <G> \r\n -->Output : turns on the motor power stage.\r\n");
	shell_add('S', stop, "*Stop* \r\n -->Command : <S>\r\n -->Output : turns off the motor power stage.\r\n");
	shell_add('s', speed, "*Speed* \r\n -->Command : <s> <cycle> <=> <XXXX>\r\n -->Output : controls the motor at the desired cycle (value/1024)\r\n");
	shell_add('p', pid_auto, "*Electronic speed control \r\n --> Command <p> <speed> <=> <XXX> \r\n --> Output: PID control of the motor, speed is in rpm \r\n");

	shell_run();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if ( GPIO_Pin == USR_BUTTON_Pin ) {
		GPIO_PinState PinState;
		PinState = HAL_GPIO_ReadPin( USR_BUTTON_GPIO_Port, USR_BUTTON_Pin);
		if ( PinState == GPIO_PIN_RESET ){
			Button_pressed = 1;
			char g = 'G';
			char * buf = 'G';
			printf("User Button\r\n");
			shell_exec(g, buf);
			uart_write(prompt, 28);
		}
		else {
			Button_pressed = 0;
		}
	}	if(GPIO_Pin==Enc_inter_Pin){
		uint32_t z = HAL_GPIO_ReadPin(Enc_inter_GPIO_Port, Enc_inter_Pin);
		uint32_t ppt_cal;
		if(z==1){
			if(it_count==0){
				it_count++;
				ppt_cal=TIM2->CNT;
			}if(it_count==1){
				ppt=TIM2->CNT-ppt_cal;
			}
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim ==&htim4 && select_control==0){
		auto_speedup();
	}if(htim==&htim4 && select_control == 0){
		pid();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
