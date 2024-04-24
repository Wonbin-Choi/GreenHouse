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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define CMD_SIZE 200
#define ARR_CNT 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned char Font[19] = {0x3F, 0X06, 0X5B, 0X4F,
                                         0X66, 0X6D, 0X7C, 0X07,
                                         0X7F, 0X67, 0X77, 0X7C,
                                         0X39, 0X5E, 0X79, 0X71,
                                         0X08, 0X80, 0x40};
volatile unsigned char arrayNum[4] = {0};
volatile int fndFlag = 1;
volatile unsigned int m_cnt, m_cntFlag = 0;
volatile unsigned char rx2Flag = 0;
char rx2Data[50];
volatile unsigned char rx3Flag = 0;
char rx3Data[50];
uint8_t cdata1;
uint8_t cdata2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void FND_Init();
void display_digit(int pos, int num);
void display_fnd(int N);
void display_onoff(int flag);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void bluetooth_Event();
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
	//uint16_t value = 0;
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
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  FND_Init();
  HAL_UART_Receive_IT(&huart2, &cdata1,1);
  HAL_UART_Receive_IT(&huart3, &cdata2,1);
  printf("%s\r\n",__FILE__);
  printf("main() Start!!\r\n");
  display_fnd(0);
  //HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); // green
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); // blue
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); // red

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,255);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(rx2Flag)
	  {
	  	printf("recv2 : %s\r\n",rx2Data);
	  	rx2Flag =0;
	  }
#if 0
	  if(m_cntFlag){
		  display_fnd(m_cnt);
		  m_cntFlag = 0;
	  }
#endif
#if 0 /////////////// LED test //////////////////////
	  //htim10.Instance->CCR1 = value;
	  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,value);
	  HAL_Delay(10);
	  value += 50;
	  if(value > 17699)
		  value = 0;
#endif
#if 0 /////////////// SERVO test ////////////////
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,125);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,125);
	  HAL_Delay(5000);

	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,25);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25);
	  HAL_Delay(5000);
#endif


	  if(rx3Flag){
		  rx3Flag = 0;
		  bluetooth_Event();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 254;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  htim3.Init.Prescaler = 1680-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_ON_OFF_Pin */
  GPIO_InitStruct.Pin = RELAY_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RELAY_ON_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM7){
		static unsigned int t_cnt = 0;
		static int digit = 0;
		t_cnt++;
		if(t_cnt >= 1000){ // 1sec
			t_cnt = 0;
			m_cnt++;
			m_cntFlag = 1;
		}
		display_digit(digit,arrayNum[digit]);
		digit++;
		if(digit == 4)
			digit = 0;
	}
}

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART3 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
	    	static int i=0;
	    	rx2Data[i] = cdata1;
	    	if(rx2Data[i] == '\r')
	    	{
	    		rx2Data[i] = '\0';
	    		rx2Flag = 1;
	    		i = 0;
	    	}
	    	else
	    	{
	    		i++;
	    	}
	    	HAL_UART_Receive_IT(&huart2, &cdata1,1);
	    }
	if(huart->Instance == USART3)
	    {
	    	static int i=0;
	    	rx3Data[i] = cdata2;
	    	if(rx3Data[i] == '\n')
	    	{
	    		rx3Data[i] = '\0';
	    		rx3Flag = 1;
	    		i = 0;
	    	}
	    	else
	    	{
	    		i++;
	    	}
	    	HAL_UART_Receive_IT(&huart3, &cdata2,1);
	    }
	}

void FND_Init(){
	MX_GPIO_Init();
	MX_TIM7_Init();
	HAL_TIM_Base_Start_IT(&htim7);
}

void display_digit(int pos, int num) {

	if (fndFlag) {
		GPIOC->ODR = GPIOC->ODR | 0xf00; // fnd all off
		GPIOC->ODR = GPIOC->ODR & ~(GPIO_PIN_8 << pos);
		GPIOC->ODR = (GPIOC->ODR & 0xff00) | Font[num];
	}

}
void display_fnd(int N)
{
	int Buff;

	if (N < 0) {
		N = -N;
		arrayNum[0] = 18;
	} else
		arrayNum[0] = N / 1000;

	Buff = N % 1000;
	arrayNum[1] = Buff / 100;
	Buff = Buff % 100;
	arrayNum[2] = Buff / 10;
	arrayNum[3] = Buff % 10;

}

void display_onoff(int flag) {

	fndFlag = flag;

	if (!fndFlag) {
		GPIOC->ODR = GPIOC->IDR | 0xf00; //fnd all off
	}
}

void bluetooth_Event()
{

  int i=0;
  int num1 = 0;
  int num2 = 0;
  int num3 = 0;
  // orange //
  int red = 255;
  int green = 0;
  int blue = 255;
  int bt_sec = m_cnt;
  char * pToken;
  char * pArray[ARR_CNT]={0};
  char recvBuf[CMD_SIZE]={0};
  strcpy(recvBuf,rx3Data);

  printf("rx3Data : %s\r\n",recvBuf);

  pToken = strtok(recvBuf,"[@]");

  while(pToken != NULL)
  {
    pArray[i] =  pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }

  if(!strcmp(pArray[1],"GMO"))
  {
	  num1 = atoi(pArray[2]); // temp
	  num2 = atoi(pArray[3]); // humid
	  num3 = atoi(pArray[4]); // light
	  int brightness = map(100 - num3, 0, 100, 0, 255);
	  display_fnd(num1);
	  if(num1 < 23){
		  HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, 1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	  }
	  else if(num1 >= 23 && num1 <= 27){
		  HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);;
	  }
	  else if(num1 > 27){
		  HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	  }
	  if(num2 < 45){
		  for(int k = 0; k < 2 ; k++){ // default = 10
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,125); // degree 180
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,125);
			  while(m_cnt - 2 < bt_sec){}
			  bt_sec = m_cnt;

			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,25); // degree 0
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25);
			  while(m_cnt - 2 < bt_sec){}
			  bt_sec = m_cnt;
		  }
	  }

	  red = (red * brightness) / 255;
	  green = (green * brightness) / 255;
	  blue = (blue * brightness) / 255;
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,255 - red); // arduino LED input
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,255 - green);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,255 - blue);
  }

  else if(!strcmp(pArray[1],"DRY"))
  {
	  if(!strcmp(pArray[2],"ON")){
		  HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, 1);
	  }
	  else if(!strcmp(pArray[2],"OFF")){
		  HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, 0);
	  }
  }
  else if(!strcmp(pArray[1],"COOL"))
    {
  	  if(!strcmp(pArray[2],"ON")){
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
  	  }
  	  else if(!strcmp(pArray[2],"OFF")){
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
  	  }
    }
  else if(!strcmp(pArray[1],"WATER"))
    {
	  if(pArray[2] != NULL){
		  num2 = atoi(pArray[2]);
		  for(int k = 0; k < num2 ; k++){ // user input water count
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,125); // degree 180
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,125);
			  while(m_cnt - 2 < bt_sec){}
			  bt_sec = m_cnt;

			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,25); // degree 0
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25);
			  while(m_cnt - 2 < bt_sec){}
			  bt_sec = m_cnt;
		  }
	  }
	  else
		  for(int k = 0; k < 10 ; k++){ // default = 10
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,125); // degree 180
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,125);
			  while(m_cnt - 2 < bt_sec){}
			  bt_sec = m_cnt;

			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,25); // degree 0
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,25);
			  while(m_cnt - 2 < bt_sec){}
			  bt_sec = m_cnt;
		  }
    }

  else if(!strcmp(pArray[1],"LED"))
    {
	  if(!strcmp(pArray[2],"ON")){
		  if(pArray[3] != NULL)
			  red = 255 - atoi(pArray[3]);
		  else red = 0;
		  if(pArray[3] != NULL)
			  green = 255 - atoi(pArray[4]);
		  else green = 0;
		  if(pArray[3] != NULL)
			  blue = 255 - atoi(pArray[5]);
		  else blue = 0;
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,red);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,green);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,blue);
	  }
	  else if(!strcmp(pArray[2],"OFF")){
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,254);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,254);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,254);
	  }
    }

  else if(!strncmp(pArray[1]," New conn",sizeof(" New conn")))
  {
      return;
  }
  else if(!strncmp(pArray[1]," Already log",sizeof(" Already log")))
  {
      return;
  }
  else
      return;
  //HAL_UART_Transmit(&huart3, (uint8_t*)sendBuf, strlen(sendBuf), HAL_MAX_DELAY);
  //sprintf(sendBuf,"[%s]%s@%s@%s@%s\n",pArray[0],pArray[1],pArray[2],pArray[3],pArray[4]);
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
