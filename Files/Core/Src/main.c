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

FILE* fichier;
int i;

//#define STACK_SIZE 1000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int pinout(int argc, char ** argv){

	printf("PIN : fonction\r\n");

	return 0;
}

int start(int argc, char ** argv){
	//Initiate Reset on driver Sequence
	printf("Initiate Reset pin \r\n");
	HAL_GPIO_WritePin(Reset_driver_GPIO_Port, Reset_driver_Pin, 0);
	HAL_GPIO_WritePin(Reset_driver_GPIO_Port, Reset_driver_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Reset_driver_GPIO_Port, Reset_driver_Pin, 0);
	printf("Initiate PWM command \r\n");
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	return 0;
}

int stop(int argc, char ** argv){
	printf("Power OFF\r\n");
	TIM1->CCR1=512;
	TIM1->CCR2=512;
	return 0;
}


int fonction(int argc, char ** argv) {
	//Usage : f <arg 1> <arg2> ...
	//Affiche le nombre d'argument donné et la liste des arguments

	printf("argc = %d\r\n", argc);

	/*Begin question 2 */
	for (int i = 0 ; i < argc ; i++) {
		printf("arg numero %d = %s\r\n", i, argv[i]);
	}
	/*End question 2 */

	return 0;
}

int speed(int argc, char ** argv){
	int vitesse;
	vitesse = atoi(argv[3]);
	printf("La vitesse demandée est %i \r\n", vitesse);
	if(vitesse<512){
		printf("Vitesse négative interdite\r\n");
		return 0;
	}else{
		TIM1->CCR1=vitesse;
		TIM1->CCR2=1024-vitesse;
	}
	return 0;
}


//void vTaskShell(void * p) {
//	shell_init();
//	shell_add('P', pinout, "*Pinout : Affichage des broches connect\351es* \r\n -->Commande : <P> \r\n -->Sortie : toutes les broches connect\351es et leur foncion. \r\n");
//		shell_add('G', start, "*Go - Start* \r\n -->Commande : <G> \r\n -->Sortie : allume l'\351tage de puissane du moteur.\r\n");
//		shell_add('S', stop, "*Stop* \r\n -->Commande : <S>\r\n -->Sortie : \351teind l'\351tage de puissance du moteur.\r\n");
//	shell_run();
//

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//	BaseType_t xReturned;
	//	TaskHandle_t xHandle = NULL;

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
  /* USER CODE BEGIN 2 */
	//Création du shell
	shell_init();
	shell_add('f', fonction, "Une fonction inutile \r\n Commande attendue : <f> <argument1> <argument2> etc. \r\n Sortie : le nombre d'argument et les arguments un par un \r\n");
	shell_add('P', pinout, "*Pinout : Affichage des broches connectées* \r\n -->Commande : <P> \r\n -->Sortie : toutes les broches connectées et leur foncion. \r\n");
	shell_add('G', start, "*Go - Start* \r\n -->Commande : <G> \r\n -->Sortie : allume l'étage de puissane du moteur.\r\n");
	shell_add('S', stop, "*Stop* \r\n -->Commande : <S>\r\n -->Sortie : éteind l'étage de puissance du moteur.\r\n");
	shell_add('s', speed, "*Speed* \r\n -->Commande : <s> <speed> <=> <XXXX>\r\n -->Sortie : commande le moteur à la vitesse souhaitée.\r\n");

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
