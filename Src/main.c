/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "wifi_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//UART_HandleTypeDef huart2;
uint8_t sum = 0;
uint8_t canbuf[8];
uint16_t *m;
char *c;
//UART_HandleTypeDef huart2;
uint8_t txdataaaa[8] = {0X00,0X01,0X02,0X03,0X04,0X05,0X06,0X07};//待机控制器指令
uint8_t txdataa[3] = {0X01,0X01,0X00};//
uint8_t txdata[8] = {0X00,0X04,0X00,0X00,0XFF,0XFF,0XFF,0XFF};//超时错误告警
uint8_t txdata0[8] = {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};//请求握手
uint8_t txdata1[8] = {0XAA,0X00,0X00,0X00,0X00,0X00,0X00,0X00};//握手成功
uint8_t txdata2[8] = {0X14,0X13,0X20,0X26,0X08,0X12,0X20,0XFF};//发送时间同步
uint8_t txdata3[8] = {0X70,0X17,0XA0,0X0F,0X7D,0X0F,0XFF,0XFF};//最大传输能力600V,3.5A
uint8_t txdata4[1] = {0XAA};//充电机准备就绪
uint8_t txdata5[8] = {00,00,22,00,00,00,00,00};//充电状态电压、电流、温度
uint8_t txdata6[8] = {0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF};//过温终止
uint8_t txdata7[8] = {0X00,0X40,0X00,0X00,0XFF,0XFF,0XFF,0XFF};//过流终止
uint8_t txdata8[8] = {0X00,0X00,0X00,0X10,0XFF,0XFF,0XFF,0XFF};//电压异常终止
uint8_t txdata9[8] = {0X40,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF};//充满正常终止
uint8_t txdata10[8] = {0X7D,0X00,0X17,0X00,0X01,0XFF,0XFF,0XFF};//充电结束
uint8_t tx_startup_first_data[8] = {0X01,0X05,0X00,0X00,0xFF,0X00,0X8C,0X3A}; // 初级继电器启动
uint8_t tx_close_first_data[8] = {0X01,0X05,0X00,0X00,0x00,0X00,0XCD,0XCA}; // 初级继电器启动

uint8_t aaa;
uint8_t brm;
uint8_t bcp;
uint8_t bro;
uint8_t bcs;
uint8_t bcl = 0;
uint8_t bst = 0;
uint8_t bsd;
uint8_t aaa = 1;

uint8_t x1[6];
uint8_t x2[8];



int r;
int s;
int y;
int z;

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
	MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	Configure_Filter();//过滤器配置  STM32CubeMX自动生成的代码里没有，需要自己配置
  HAL_CAN_Start(&hcan);
	bool isPrimaryRelayStart = false; // 初级继电器启动状态
	bool isBMSReady = false; // BMS是否准备好充电状态
	bool isChargerReady = false; // 充电机是否准备好充电状态
	bool isWifiConnected = false; // Wifi是否连通
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    // 判断wifi是否连通
		isWifiConnected = ProcessWifiConnected();
			
	}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/* USER CODE BEGIN 4 */
void Error_Warn(void)
{
  
  __disable_irq();
  while (1)
  {
		CAN_Send_Test(0x101FF456,txdata,8);//发送失败错误报文
  }

}

uint16_t checksum(uint8_t  *buf,uint8_t len)
{
   uint8_t i;
   uint16_t cs = 0;
   for(i=0;i<len;i++)
   {
      cs +=buf[i];
   }

   return (uint16_t)cs;
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
