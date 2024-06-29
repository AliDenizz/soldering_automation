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
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUFFER_SIZE 50
char rxBuffer[BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
int x = 0;
int y = 0;
int currentX = 0;
int currentY = 0;
volatile uint8_t flag1=0;
int step_delay = 800;
double x_coff=1.578;
double y_coff=1.239;
int step_x, step_y;
int tester=0;
volatile uint8_t x_stop_value=0;
volatile uint8_t y_stop_value=0;
volatile uint8_t counter_x=0;
volatile uint8_t counter_y=0;
volatile uint8_t shotFlag = 0;
volatile uint8_t doneFlag = 0;
int Z_half = 980;



void micro_delay(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1)<delay);
}

void processBuffer(void) {
    if (strncmp(rxBuffer, "DONE", 4) == 0) {
        doneFlag = 1;
    }else if (strncmp(rxBuffer, "SHOT", 4) == 0) {
		shotFlag = 1;

    }else {

        char *token;
        token = strtok(rxBuffer, ":,");
        if (token != NULL && strcmp(token, "x") == 0) {
            token = strtok(NULL, ",");
            if (token != NULL) {
                x = atoi(token); // x koordinatı
            }
            token = strtok(NULL, ":,");
            if (token != NULL && strcmp(token, "y") == 0) {
                token = strtok(NULL, ",");
                if (token != NULL) {
                    y = atoi(token); // y koordinatı
                }
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rxBuffer[rxIndex] == '\n') { // Yeni satır karakteri
            rxBuffer[rxIndex] = '\0';
            processBuffer();
            rxIndex = 0; // Buffer index sıfırlanır
            flag1=1;
        } else {
            rxIndex++;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxBuffer[rxIndex], 1); // Yeni veri alımı
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(x_stop_GPIO_Port, x_stop_Pin)){
		x_stop_value=1;
	}else if(HAL_GPIO_ReadPin(y_stop_GPIO_Port, y_stop_Pin)){
		y_stop_value=1;
	}
}
void zero_point(void){

	while(1){

		if(x_stop_value==0){
			HAL_GPIO_WritePin(x_dir_GPIO_Port, x_dir_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_SET);
			micro_delay(step_delay);
			HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_RESET);
			micro_delay(step_delay);

		}else if(x_stop_value==1){
			HAL_GPIO_WritePin(x_dir_GPIO_Port, x_dir_Pin, GPIO_PIN_SET);
			for(int i=0;i<14;i++){
				HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_SET);
				micro_delay(step_delay);
				HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_RESET);
				micro_delay(step_delay);
			}
			x_stop_value=0;
			break;

		}
	}
	while(1){

		if(y_stop_value==0){
			HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_SET);
			micro_delay(step_delay);
			HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);
			micro_delay(step_delay);

		}else if(y_stop_value==1){
			HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_SET);
			for(int i=0;i<14;i++){
				HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_SET);
				micro_delay(step_delay);
				HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);
				micro_delay(step_delay);
			}
			y_stop_value=0;
			break;

		}
	}while(!shotFlag){

	}
	if(shotFlag==1){
		HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_SET);
		for(int i=0;i<640;i++){
			HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_SET);
			micro_delay(step_delay);
			HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);
			micro_delay(step_delay);
		}
		HAL_GPIO_WritePin(x_dir_GPIO_Port, x_dir_Pin, GPIO_PIN_SET);
		for(int i=0;i<180;i++){
			HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_SET);
			micro_delay(step_delay);
			HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_RESET);
			micro_delay(step_delay);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxBuffer[rxIndex], 1);
  HAL_TIM_Base_Start(&htim1);
  zero_point();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (!doneFlag)
  {
	  while(flag1==1){
		  if(currentX - x < 0){
			  HAL_GPIO_WritePin(x_dir_GPIO_Port, x_dir_Pin, GPIO_PIN_SET);
			  step_x=(int)round((x-currentX)*x_coff);
		  }else{
			  HAL_GPIO_WritePin(x_dir_GPIO_Port, x_dir_Pin, GPIO_PIN_RESET);
			  step_x=(int)round((currentX-x)*x_coff);
		  }
		  if(currentY - y < 0){
			  HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_SET);
			  step_y=(int)round((y-currentY)*y_coff);
		  }else{
			  HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_RESET);
			  step_y=(int)round((currentY-y)*y_coff);
		  }
		  for(int i=0;i<step_x;i++){
			  if(x_stop_value==1){
				  HAL_GPIO_WritePin(x_dir_GPIO_Port, x_dir_Pin, GPIO_PIN_SET);
				  for(i=0;i<5;i++){
					  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_SET);
					  micro_delay(step_delay);
					  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);
					  micro_delay(step_delay);
				  }
				  currentX=0;
				  x_stop_value=0;
				  break;
			  }
			  HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_SET);
			  micro_delay(step_delay);
			  HAL_GPIO_WritePin(x_step_GPIO_Port, x_step_Pin, GPIO_PIN_RESET);
			  micro_delay(step_delay);
		  }
		  for(int i=0;i<step_y;i++){
			  if(y_stop_value==1){
				  HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_SET);
				  for(i=0;i<5;i++){
					  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_SET);
					  micro_delay(step_delay);
					  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);
					  micro_delay(step_delay);
				  }
				  currentY=0;
				  y_stop_value=0;
				  break;
			  }
			  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_SET);
			  micro_delay(step_delay);
			  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);
			  micro_delay(step_delay);
		  }
		  HAL_GPIO_WritePin(z_dir_GPIO_Port, z_dir_Pin, GPIO_PIN_SET);
		  for(int i = 0; i<Z_half; i++){
			  HAL_GPIO_WritePin(z_step_GPIO_Port, z_step_Pin, GPIO_PIN_SET);
			  micro_delay(step_delay);
			  HAL_GPIO_WritePin(z_step_GPIO_Port, z_step_Pin, GPIO_PIN_RESET);
			  micro_delay(step_delay);
		  }

		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  HAL_Delay(4000);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		  HAL_GPIO_WritePin(z_dir_GPIO_Port, z_dir_Pin, GPIO_PIN_RESET);
		  for(int i = 0; i<Z_half; i++){
			  HAL_GPIO_WritePin(z_step_GPIO_Port, z_step_Pin, GPIO_PIN_SET);
			  micro_delay(step_delay);
			  HAL_GPIO_WritePin(z_step_GPIO_Port, z_step_Pin, GPIO_PIN_RESET);
			  micro_delay(step_delay);
		  }




		  currentX=x;
		  currentY=y;
		  flag1=0;
		  HAL_UART_Transmit_IT(&huart1, (uint8_t *)"OK", sizeof("OK")-1);
		  break;

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  zero_point();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|z_dir_Pin|z_step_Pin|x_step_Pin
                          |x_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(y_dir_GPIO_Port, y_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(y_step_GPIO_Port, y_step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : x_stop_Pin y_stop_Pin */
  GPIO_InitStruct.Pin = x_stop_Pin|y_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin z_dir_Pin z_step_Pin x_step_Pin
                           x_dir_Pin */
  GPIO_InitStruct.Pin = LED_Pin|z_dir_Pin|z_step_Pin|x_step_Pin
                          |x_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : y_dir_Pin */
  GPIO_InitStruct.Pin = y_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(y_dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : y_step_Pin */
  GPIO_InitStruct.Pin = y_step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(y_step_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
