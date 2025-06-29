/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    LEFT_LED_CTRL = 0,
	RIGHT_LED_CTRL
}LED_Ctrl_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_PinState Prev_SW1_State = GPIO_PIN_RESET;
GPIO_PinState Prev_SW2_State = GPIO_PIN_RESET;
GPIO_PinState Prev_SW3_State = GPIO_PIN_RESET;
GPIO_PinState Prev_SW4_State = GPIO_PIN_RESET;

GPIO_PinState Curr_SW1_State = GPIO_PIN_RESET;
GPIO_PinState Curr_SW2_State = GPIO_PIN_RESET;
GPIO_PinState Curr_SW3_State = GPIO_PIN_RESET;
GPIO_PinState Curr_SW4_State = GPIO_PIN_RESET;

GPIO_PinState Pin_State = GPIO_PIN_RESET;;

LED_Ctrl_Mode Mode = LEFT_LED_CTRL;

uint8_t uart3_rx_data;
uint8_t uart3_rx_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Curr_SW1_State =  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
	  if ((Prev_SW1_State == GPIO_PIN_RESET && Curr_SW1_State == GPIO_PIN_SET) || uart3_rx_data == '1')
	  {
		  uart3_rx_data = 0;
		  GPIO_TypeDef *targetPort = (Mode == LEFT_LED_CTRL) ? GPIOD : GPIOC;
		  uint16_t targetPin = (Mode == LEFT_LED_CTRL) ? GPIO_PIN_12 : GPIO_PIN_6;

		  Pin_State = HAL_GPIO_ReadPin(targetPort, targetPin);
		  HAL_GPIO_WritePin(targetPort, targetPin, (GPIO_PinState)(!Pin_State));
	  }
	  Prev_SW1_State = Curr_SW1_State;

	  Curr_SW2_State =  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
	  if ((Prev_SW2_State == GPIO_PIN_RESET && Curr_SW2_State == GPIO_PIN_SET) || uart3_rx_data == '2')
	  {
		  uart3_rx_data = 0;
		  GPIO_TypeDef *targetPort = (Mode == LEFT_LED_CTRL) ? GPIOD : GPIOB;
		  uint16_t targetPin = (Mode == LEFT_LED_CTRL) ? GPIO_PIN_13 : GPIO_PIN_5;

		  Pin_State = HAL_GPIO_ReadPin(targetPort, targetPin);
		  HAL_GPIO_WritePin(targetPort, targetPin, (GPIO_PinState)(!Pin_State));
	  }
	  Prev_SW2_State = Curr_SW2_State;

	  Curr_SW3_State =  HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
	  if ((Prev_SW3_State == GPIO_PIN_RESET && Curr_SW3_State == GPIO_PIN_SET) || uart3_rx_data == '3')
	  {
		  uart3_rx_data = 0;
		  GPIO_TypeDef *targetPort = (Mode == LEFT_LED_CTRL) ? GPIOD : GPIOB;
		  uint16_t targetPin = (Mode == LEFT_LED_CTRL) ? GPIO_PIN_14 : GPIO_PIN_0;

		  Pin_State = HAL_GPIO_ReadPin(targetPort, targetPin);
		  HAL_GPIO_WritePin(targetPort, targetPin, (GPIO_PinState)(!Pin_State));
	  }
	  Prev_SW3_State = Curr_SW3_State;

	  Curr_SW4_State =  HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
	  if((Prev_SW4_State == GPIO_PIN_RESET && Curr_SW4_State == GPIO_PIN_SET) || uart3_rx_data == '4')
	  {
		  uart3_rx_data = 0;
		  Mode = (LED_Ctrl_Mode)(!Mode);
	  }
	  Prev_SW4_State = Curr_SW4_State;
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));
		uart3_rx_flag = 1;
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
