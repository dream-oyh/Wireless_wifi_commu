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
uint8_t txdataaaa[8] = {0X00,0X01,0X02,0X03,0X04,0X05,0X06,0X07};//����������ָ��
uint8_t txdataa[3] = {0X01,0X01,0X00};//
uint8_t txdata[8] = {0X00,0X04,0X00,0X00,0XFF,0XFF,0XFF,0XFF};//��ʱ����澯
uint8_t txdata0[8] = {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};//��������
uint8_t txdata1[8] = {0XAA,0X00,0X00,0X00,0X00,0X00,0X00,0X00};//���ֳɹ�
uint8_t txdata2[8] = {0X14,0X13,0X20,0X26,0X08,0X12,0X20,0XFF};//����ʱ��ͬ��
uint8_t txdata3[8] = {0X70,0X17,0XA0,0X0F,0X7D,0X0F,0XFF,0XFF};//���������600V,3.5A
uint8_t txdata4[1] = {0XAA};//����׼������
uint8_t txdata5[8] = {00,00,22,00,00,00,00,00};//���״̬��ѹ���������¶�
uint8_t txdata6[8] = {0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF};//������ֹ
uint8_t txdata7[8] = {0X00,0X40,0X00,0X00,0XFF,0XFF,0XFF,0XFF};//������ֹ
uint8_t txdata8[8] = {0X00,0X00,0X00,0X10,0XFF,0XFF,0XFF,0XFF};//��ѹ�쳣��ֹ
uint8_t txdata9[8] = {0X40,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF};//����������ֹ
uint8_t txdata10[8] = {0X7D,0X00,0X17,0X00,0X01,0XFF,0XFF,0XFF};//������
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
	Configure_Filter();//����������  STM32CubeMX�Զ����ɵĴ�����û�У���Ҫ�Լ�����
  HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//��λ���������ָ��
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

		//���յ���λ��������home��ָ��ʱ���������ʼ���������������
		x1[0] = '0';
		x1[1] = '0';
		x1[2] = '0';
		x1[3] = '0';
*/	

    printf("���Կ�ʼ\r\n");
		do
		{
			CAN_Send_Test(0x0123F156,txdataaaa,8);//���Ϳ�������������
			HAL_Delay(2000);
			aaa = can_receive_msg(0x012356F1, canbuf);
		}
	  while( aaa == 0);//�յ�����������������Ϣ
		printf("�����������յ����ָ�\r\n");
    //����������������Ϣ���򽻻��ɹ�
		
//	CANͨѶ���ֺ�����׼���׶�
	  printf("CANͨѶ��������\r\n");
		
		do
		{
			CAN_Send_Test(0x1826F456,txdataa,3);//���Ͳ��ܱ�ʶ����CRM
			HAL_Delay(2000);
			sum++;
//			printf("%d\r\n",sum);
			brm = can_receive_msg(0x182756F4, canbuf);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }
		}
	  while( brm == 0);//�յ�BMS�ͳ�����ʶ����
		printf("������BMS���ֳɹ���\r\n");
		sum = 0;
		
		
		do
		{
			CAN_Send_Test(0x1801F456,txdata0,8);//���Ͳ��ܱ�ʶ����CRM
			
			sum++;
//			printf("%d\r\n",sum);
			brm = can_receive_msg(0x1CEC56F4, canbuf);
			HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }
		}
	  while( brm == 0);//�յ�BMS�ͳ�����ʶ����
		printf("������BMS���ֳɹ���\r\n");
		sum = 0;

		
		do
		{
			
			CAN_Send_Test(0x1801F456,txdata1,8);//�����ܱ�ʶ����CRM
			
			sum++;
//			printf("%d\r\n",sum);
			bcp = can_receive_msg(0x1CEC56F4, canbuf);
			HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }
		}
	  while( bcp == 0);//�յ���س���������
		printf("�յ�bms���������г�����������\r\n");
		sum = 0;
		do
		{
			
			CAN_Send_Test(0x1807F456,txdata2,8);//����ʱ��ͬ��
			CAN_Send_Test(0x1808F456,txdata3,8);//���ͳ�����������������
			HAL_Delay(2000);
			sum++;
//			printf("%d\r\n",sum);
			bro = can_receive_msg(0x100956F4, canbuf);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }

		}	
	  while( bro == 0);//BMS׼������
		printf("bms׼������!\r\n");
	  sum = 0;
     do
		{
			
			CAN_Send_Test(0x100AF456,txdata4,1);//����׼������
			
			sum++;
//			printf("%d\r\n",sum);
			bcl = can_receive_msg(0x181056F4, canbuf);
			HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }
		}
	  while( bcl == 0);//BMS��״̬����
		printf("����׼������!\r\n");
		printf("��ʼ���!\r\n");
		while(1)
		{
			sum = 0;
			bcl = 0;
			do
		{
			printf("�ȴ�bms�������\r\n");
			
			
//			sum++;
//			printf("%d\r\n",sum);
			bcl = can_receive_msg(0x181056F4,canbuf);
//		  aaa	= can_receive_msg(0x101956F4, canbuf);//bms������ֹ����
//			HAL_Delay(20);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }
		}
		 while( bcl == 0);//BMS����������
		 printf("�յ�bms�������\r\n");
		
		
		// ������ҪУ���8λ����
		x1[0] = 0X01;
		x1[5] = 0XFF;
		
		for(int i=0;i<4;i++)
		{
		    x1[i+1] = canbuf[i];
		}

    uint8_t len = sizeof(x1) / sizeof(x1[0]);
 
    // ����У���
    uint16_t checksum_value = checksum(x1, len);

    // ��ȡУ��͵ĵ�λ�͸�λ
    unsigned char low_byte = (unsigned char)(checksum_value & 0xFF);
    unsigned char high_byte = (unsigned char)((checksum_value >> 8) & 0xFF);

    // ����λ�͸�λ��ӵ��������
    x1[len] = low_byte;
    x1[len + 1] = high_byte;

		//���յ���bms��Ϣ���͸�DC-DC������ߵ��ڵ���
		HAL_UART_Transmit(&huart1,x1,8,HAL_MAX_DELAY);
		
		//�յ��ɼ��ĵ�ѹ����ֵת����bms
		if(HAL_UART_Receive(&huart1,x2,8,HAL_MAX_DELAY) == HAL_OK)
		{
			CAN_Send_Test(0x1812F456,x2,8);//������Ϣ
		
		}
		 
//		sum++;
//		printf("%d\r\n",sum);
//		HAL_Delay(500);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				CAN_Send_Test(0x101AF456,txdata8,8);
//				break;
//       }
//		
		bst = can_receive_msg(0x101956F4, canbuf);
		 if(bst==1)
			 break;
		
		}
		printf("�����ֹ��\r\n");
		sum = 0;
//		do
//		{
//			
//			if(*(m+2) > 90)
//			{
//				CAN_Send_Test(0x101AF456,txdata6,8);//�����¶ȹ�����ֹ
//				printf("�����¶ȹ�����ֹ��\r\n");
//			}
//			else if(*(m+1) < (400-14))
//			{
//				CAN_Send_Test(0x101AF456,txdata7,8);//���������쳣��ֹ
//				printf("���������쳣��ֹ��\r\n");
//			}
//			else if(*m > 5000 && *m < 6000)
//			{
//				CAN_Send_Test(0x101AF456,txdata8,8);//������ѹ�쳣��ֹ
//				printf("������ѹ�쳣��ֹ��\r\n");
//			}
//			else
//			{
//				CAN_Send_Test(0x101AF456,txdata9,8);//������������ֹ
//			  printf("������������ֹ��\r\n");
//			}
//			HAL_Delay(2000);
//			sum++;
//			printf("%d\r\n",sum);
//			bst = can_receive_msg(0x101956F4, canbuf);
//			if(sum > 20)
//			{
//				printf("��ʱ����澯!\r\n");
//				Error_Warn();
//       }
//		}
//	   while( bst == 0);//BMS��ֹ���
//		 printf("����������׶Σ�\r\n");
//		sum = 0;
		do
		{
			
			CAN_Send_Test(0x101AF456,txdata9,8);//������ֹ�������
			HAL_Delay(2000);
			sum++;
//			printf("%d\r\n",sum);
			bsd = can_receive_msg(0x181C56F4, canbuf);
			if(sum > 20)
			{
				printf("��ʱ����澯!\r\n");
				Error_Warn();
       }
		}
	  while( bsd == 0);//BMSͳ������
		
		
		CAN_Send_Test(0x181DF456,txdata10,8);//����ͳ������
		printf("��������\r\n");//�������
			

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
		CAN_Send_Test(0x101FF456,txdata,8);//����ʧ�ܴ�����
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
