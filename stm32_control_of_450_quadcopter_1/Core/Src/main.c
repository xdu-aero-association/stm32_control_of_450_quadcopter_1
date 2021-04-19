/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//手飞模式控制投放框状态机
typedef enum
{
	no_cmd = 0,
	first_block,
	first_block_done,
	second_block,
	second_block_done,
	third_block,
	third_block_done,
}Free_block;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cmd[4]={0};
uint8_t block_state = no_cmd;

uint8_t flag[3]={0};






//设置舵机当前的控制指令
void set_block_control()
{
	

	//不打开投放框
	 if(block_state == no_cmd)
	{
		TIM1->CCR1 = 1500;
		TIM1->CCR2 = 1500;
		TIM1->CCR3 = 1300;
	}
	//打开第一个投放框
	else if(block_state == first_block)
	{
		TIM1->CCR1 = 600;
		TIM1->CCR2 = 1500;
		TIM1->CCR3 = 1300;
		HAL_Delay(800);
		flag[0] = 1;
	}
	else if(block_state == first_block_done)
	{
		TIM1->CCR1 = 1500;
		TIM1->CCR2 = 1500;
		TIM1->CCR3 = 1300;
		//HAL_Delay(400);
	}
	//打开第二个投放框
	else if(block_state == second_block)
	{
		TIM1->CCR1 = 1500;
		TIM1->CCR2 = 600;
		TIM1->CCR3 = 1300;
		HAL_Delay(800);
		flag[1] = 1;
	}
	else if(block_state == second_block_done)
	{
		TIM1->CCR1 = 1500;
		TIM1->CCR2 = 1500;
		TIM1->CCR3 = 1300;
		//HAL_Delay(400);
	}
	//打开第三个投放框
	else if(block_state == third_block)
	{
		TIM1->CCR1 = 1500;
		TIM1->CCR2 = 1500;
		TIM1->CCR3 = 600;
		HAL_Delay(800);
		flag[2] = 1;
	}
	
	
	
	
	//读取红外检测模块判断物块是否掉落
	//if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == SET) && (block_state == first_block))
	 if((block_state == first_block)&&(flag[0] == 1))
	{
		block_state = first_block_done;
		flag[0] = 0;
	}
	//else if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == SET) && (block_state == second_block))
	else if((block_state == second_block)&&(flag[1] == 1))
	{
		block_state = second_block_done;	
		flag[1] = 0;
	}
	//else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == SET) && (block_state == third_block))
	else if((block_state == third_block)&&(flag[2] == 1))
	{
		block_state = no_cmd;
		flag[2] = 0;
	}

	
	
	

}


unsigned char ucRxBuffer[250];
unsigned char ucRxCnt = 0;
void find_cmd()
{
	ucRxBuffer[ucRxCnt++]=(unsigned char)USART1->DR;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[ucRxCnt]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<4) {return;}//数据不满11个，则返回
	else
	{
		cmd[0] =  ucRxBuffer[0];
		cmd[1] =  ucRxBuffer[1];
		cmd[2] =  ucRxBuffer[2];
		cmd[3] =  ucRxBuffer[3];
		ucRxCnt=0;//清空缓存区
	}
	
}



void USART1_IRQHandler(void)
{
  
  
  if(USART1->CR1 & 20)
  {
  
	find_cmd();
  
  }
  HAL_UART_IRQHandler(&huart1);
  HAL_UART_Receive_IT(&huart1,cmd,1);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

HAL_TIM_Base_Start(&htim1);
HAL_Delay(800);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//TIM1->CCR3 = 1000;
	set_block_control();		 //设置投放物投放指令
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int i;
    for(i=0;i<1000;i++);
	while(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6));
	if(block_state == third_block)
		block_state = no_cmd;
	else if(block_state == second_block_done)
		block_state = third_block;
	else if(block_state == first_block_done)
		block_state = second_block;
	else if(block_state == no_cmd)
		block_state = first_block;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
