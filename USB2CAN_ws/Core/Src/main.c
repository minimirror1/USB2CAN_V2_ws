/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* Component Includes*/
#include "cp_delay.h"
#include "cp_can_driver.h"
#include "cp_can_datalink.h"
#include "cp_can_network.h"
#include "cp_crc_ccitt.h"

#include "cp_uart_driver.h"
#include "cp_uart_datalink.h"

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void CM_CAN1_Driver_Init(void);
static void CM_UART2_Driver_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern CP_CAN_ManagerHandleTypeDef cpCanManager;//test
uint8_t data[64] = {0,};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t t_run;

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CM_CAN1_Driver_Init();
  CM_UART2_Driver_Init();

  uint32_t ExtId = 0;
  NET_MOTIONIST_HeaderParserTypeDef *pNetHeader = (NET_MOTIONIST_HeaderParserTypeDef *)&ExtId;
  pNetHeader->b.tarid_base = 0;


  for(int i = 0; i < 64 ; i ++)
  {
	  data[i] = i;
  }
  //NET_MOTIONIST_DataHeaderParserTypeDef *pNetDHeader = (NET_MOTIONIST_DataHeaderParserTypeDef *)&data[0];

  //pNetDHeader->b.tarid_sub = 30;

  NET_MOTIONIST_TX_PacketInfoTypeDef txPacket;
  txPacket.tarid_base = 5;
  txPacket.tarid_sub = 1;
  txPacket.ackEn = 1;
  txPacket.cmd = 50;
  txPacket.len = sizeof(data);
  txPacket.pData = data;

  CP_NET_TX_AddPacket(&hcan1, &txPacket);

  uint32_t t_txT = 0;


  //crc test
  //uint16_t crc = CP_CRC_CCITT_Calc(data, 60);
  //volatile uint16_t res = CP_CRC_CCITT_Check(data, 60, crc);

  uint8_t str[] = "hello world!";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(CP_Delay_NonStop(&t_run, 500))
	  {
		  HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);

		  //CP_CAN_SendAddQueue_ExtData(&cpCanManager.list[0].cpCan, ExtId, data, 8);
		  //HAL_UART_Transmit_IT(&huart2, str, sizeof(str));
		  //CP_UART_SendString(USART2, (char *)str);
	  }

	  CP_CAN_Process();
	  CP_UART_Protocol_Process();
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  huart2.Init.BaudRate = 921600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_CAN1_RX_Pin|LED_CAN1_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_UART1_RX_GPIO_Port, LED_UART1_RX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_UART1_TX_GPIO_Port, LED_UART1_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_ERR_Pin */
  GPIO_InitStruct.Pin = LED_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ERR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RUN_Pin LED_CAN1_RX_Pin LED_CAN1_TX_Pin */
  GPIO_InitStruct.Pin = LED_RUN_Pin|LED_CAN1_RX_Pin|LED_CAN1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_UART1_RX_Pin */
  GPIO_InitStruct.Pin = LED_UART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_UART1_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_UART1_TX_Pin */
  GPIO_InitStruct.Pin = LED_UART1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_UART1_TX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void CM_NET_Motionist_Init(void)
{
	NET_MOTIONIST_InitTypeDef NET_InitStruct = {0,};

	/* Configure HAL CAN Handle */
	NET_InitStruct.hcan = &hcan1;

	/* Configure ID */
	NET_InitStruct.myID_base = 1;
	NET_InitStruct.myID_sub = 1;

	if (CP_NET_MOTIONIST_Init(&NET_InitStruct) != HAL_OK)
	{
		Error_Handler();
	}

}
static void CM_CAN1_Driver_Init(void)
{
	CP_CAN_InitTypeDef CP_CAN_InitStruct = {0,};

	/*Configure HAL CAN Handle */
	CP_CAN_InitStruct.hcan = &hcan1;

	/*Configure Tx LED GPIO pin */
	CP_CAN_InitStruct.txLed.f_active = CP_CAN_LED_ENABLE;
	CP_CAN_InitStruct.txLed.GPIO_Port = LED_CAN1_TX_GPIO_Port;
	CP_CAN_InitStruct.txLed.Pin = LED_CAN1_TX_Pin;
	CP_CAN_InitStruct.txLed.onStatus = GPIO_PIN_RESET;

	/*Configure Rx LED GPIO pin */
	CP_CAN_InitStruct.rxLed.f_active = CP_CAN_LED_ENABLE;
	CP_CAN_InitStruct.rxLed.GPIO_Port = LED_CAN1_RX_GPIO_Port;
	CP_CAN_InitStruct.rxLed.Pin = LED_CAN1_RX_Pin;
	CP_CAN_InitStruct.rxLed.onStatus = GPIO_PIN_RESET;

	CP_CAN_Driver_Init(&CP_CAN_InitStruct);

	CM_NET_Motionist_Init();
}

void CM_UART2_Driver_Init(void)
{
	CP_UART_InitTypeDef cpUartInitStruct = {0,};

	cpUartInitStruct.huart = &huart2;

	cpUartInitStruct.UART_IRQn = USART2_IRQn;

	/* tx led gpio */
	cpUartInitStruct.txLed.active = CP_UART_ALT_GPIO_ENABLE;
	cpUartInitStruct.txLed.GPIO_Port = LED_UART1_TX_GPIO_Port;
	cpUartInitStruct.txLed.Pin = LED_UART1_TX_Pin;
	cpUartInitStruct.txLed.onStatus = GPIO_PIN_RESET;

	/* rx led gpio */
	cpUartInitStruct.rxLed.active = CP_UART_ALT_GPIO_ENABLE;
	cpUartInitStruct.rxLed.GPIO_Port = LED_UART1_RX_GPIO_Port;
	cpUartInitStruct.rxLed.Pin = LED_UART1_RX_Pin;
	cpUartInitStruct.rxLed.onStatus = GPIO_PIN_RESET;

	CP_UART_Init(&cpUartInitStruct);

	/* uart packet protocol */
	UART_Protocol_InitTypeDef uartProtocolInit;
	uartProtocolInit.huart = &huart2;
	CP_UART_Protocol_Init(&uartProtocolInit);

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
