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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t sum = 0;
uint8_t canbuf[8];
uint16_t *m;
char *c;
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
  /* USER CODE BEGIN 2 */
	Configure_Filter();//过滤器配置  STM32CubeMX自动生成的代码里没有，需要自己配置
  HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//上位机发出充电指令
/*
		while(1)
		{
			HAL_UART_Receive(&huart1,(uint8_t *)&x1,4,HAL_MAX_DELAY);
			if(x1[0] == 'h' && x1[1] == 'o' && x1[2] == 'm' && x1[3] == 'e' )
			{
				printf("right!\r\n");
				break;
			}
			else
			{
				printf("error!\r\n");
			}	
		}

		//当收到上位机发出的home的指令时，则充电机开始与待机控制器交互
		x1[0] = '0';
		x1[1] = '0';
		x1[2] = '0';
		x1[3] = '0';
*/	

    printf("测试开始\r\n");
		do
		{
			CAN_Send_Test(0x0123F156,txdataaaa,8);//发送控制器交互报文
			HAL_Delay(2000);
			aaa = can_receive_msg(0x012356F1, canbuf);
		}
	  while( aaa == 0);//收到待机控制器反馈信息
		printf("待机控制器收到充电指令！\r\n");
    //待机控制器返回信息，则交互成功
		
//	CAN通讯握手和配置准备阶段
	  printf("CAN通讯握手请求！\r\n");
		
		do
		{
			CAN_Send_Test(0x1826F456,txdataa,3);//发送不能辨识报文CRM
			HAL_Delay(2000);
			sum++;
//			printf("%d\r\n",sum);
			brm = can_receive_msg(0x182756F4, canbuf);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }
		}
	  while( brm == 0);//收到BMS和充电机辨识报文
		printf("充电机与BMS握手成功！\r\n");
		sum = 0;
		
		
		do
		{
			CAN_Send_Test(0x1801F456,txdata0,8);//发送不能辨识报文CRM
			
			sum++;
//			printf("%d\r\n",sum);
			brm = can_receive_msg(0x1CEC56F4, canbuf);
			HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }
		}
	  while( brm == 0);//收到BMS和充电机辨识报文
		printf("充电机与BMS握手成功！\r\n");
		sum = 0;

		
		do
		{
			
			CAN_Send_Test(0x1801F456,txdata1,8);//发送能辨识报文CRM
			
			sum++;
//			printf("%d\r\n",sum);
			bcp = can_receive_msg(0x1CEC56F4, canbuf);
			HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }
		}
	  while( bcp == 0);//收到电池充电参数报文
		printf("收到bms充电参数进行充电机参数配置\r\n");
		sum = 0;
		do
		{
			
			CAN_Send_Test(0x1807F456,txdata2,8);//发送时间同步
			CAN_Send_Test(0x1808F456,txdata3,8);//发送充电机最大输出能力参数
			HAL_Delay(2000);
			sum++;
//			printf("%d\r\n",sum);
			bro = can_receive_msg(0x100956F4, canbuf);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }

		}	
	  while( bro == 0);//BMS准备就绪
		printf("bms准备就绪!\r\n");
	  sum = 0;
     do
		{
			
			CAN_Send_Test(0x100AF456,txdata4,1);//充电机准备就绪
			
			sum++;
//			printf("%d\r\n",sum);
			bcl = can_receive_msg(0x181056F4, canbuf);
			HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }
		}
	  while( bcl == 0);//BMS总状态发送
		printf("充电机准备就绪!\r\n");
		printf("开始充电!\r\n");
		while(1)
		{
			sum = 0;
			bcl = 0;
			do
		{
			printf("等待bms充电需求\r\n");
			
			
//			sum++;
//			printf("%d\r\n",sum);
			bcl = can_receive_msg(0x181056F4,canbuf);
//		  aaa	= can_receive_msg(0x101956F4, canbuf);//bms发送中止请求
//			HAL_Delay(20);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }
		}
		 while( bcl == 0);//BMS充电需求参数
		 printf("收到bms充电需求\r\n");
		
		
		// 这是需要校验的8位数据
		x1[0] = 0X01;
		x1[5] = 0XFF;
		
		for(int i=0;i<4;i++)
		{
		    x1[i+1] = canbuf[i];
		}

    uint8_t len = sizeof(x1) / sizeof(x1[0]);
 
    // 计算校验和
    uint16_t checksum_value = checksum(x1, len);

    // 获取校验和的低位和高位
    unsigned char low_byte = (unsigned char)(checksum_value & 0xFF);
    unsigned char high_byte = (unsigned char)((checksum_value >> 8) & 0xFF);

    // 将低位和高位添加到数组后面
    x1[len] = low_byte;
    x1[len + 1] = high_byte;

		//将收到的bms信息发送给DC-DC充电在线调节电流
		HAL_UART_Transmit(&huart1,x1,8,HAL_MAX_DELAY);
		
		//收到采集的电压电流值转发给bms
		if(HAL_UART_Receive(&huart1,x2,8,HAL_MAX_DELAY) == HAL_OK)
		{
			CAN_Send_Test(0x1812F456,x2,8);//采样信息
		
		}
		 
//		sum++;
//		printf("%d\r\n",sum);
//		HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				CAN_Send_Test(0x101AF456,txdata8,8);
//				break;
//       }
//		
		bst = can_receive_msg(0x101956F4, canbuf);
		 if(bst==1)
			 break;
		
		}
		printf("充电终止！\r\n");
		sum = 0;
//		do
//		{
//			
//			if(*(m+2) > 90)
//			{
//				CAN_Send_Test(0x101AF456,txdata6,8);//充电机温度过高终止
//				printf("充电机温度过高终止！\r\n");
//			}
//			else if(*(m+1) < (400-14))
//			{
//				CAN_Send_Test(0x101AF456,txdata7,8);//充电机电流异常终止
//				printf("充电机电流异常终止！\r\n");
//			}
//			else if(*m > 5000 && *m < 6000)
//			{
//				CAN_Send_Test(0x101AF456,txdata8,8);//充电机电压异常终止
//				printf("充电机电压异常终止！\r\n");
//			}
//			else
//			{
//				CAN_Send_Test(0x101AF456,txdata9,8);//充电机充满电终止
//			  printf("充电机充满电终止！\r\n");
//			}
//			HAL_Delay(2000);
//			sum++;
//			printf("%d\r\n",sum);
//			bst = can_receive_msg(0x101956F4, canbuf);
//			if(sum > 20)
//			{
//				printf("超时错误告警!\r\n");
//				Error_Warn();
//       }
//		}
//	   while( bst == 0);//BMS终止充电
//		 printf("进入充电结束阶段！\r\n");
//		sum = 0;
		do
		{
			
			CAN_Send_Test(0x101AF456,txdata9,8);//充电机终止充电请求
			HAL_Delay(2000);
			sum++;
//			printf("%d\r\n",sum);
			bsd = can_receive_msg(0x181C56F4, canbuf);
			if(sum > 20)
			{
				printf("超时错误告警!\r\n");
				Error_Warn();
       }
		}
	  while( bsd == 0);//BMS统计数据
		
		
		CAN_Send_Test(0x181DF456,txdata10,8);//充电机统计数据
		printf("充电结束！\r\n");//结束充电
			

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
