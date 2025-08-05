/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
CAN_TxHeaderTypeDef TxMessage;
CAN_RxHeaderTypeDef RxHeader;
uint8_t rxbuf[8];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Configure_Filter(void)
{
    CAN_FilterTypeDef sFilterConfig;
    
    sFilterConfig.FilterIdHigh=0X0000;     //32λID
    sFilterConfig.FilterIdLow=0X0000;
    sFilterConfig.FilterMaskIdHigh=0X0000; //32λMASK
    sFilterConfig.FilterMaskIdLow=0X0000;  
    sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;//������0������FIFO0
    sFilterConfig.FilterBank=1;          //������0
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation=ENABLE; //�����˲���0
    sFilterConfig.SlaveStartFilterBank=14;
    
    if(HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK)
    {
        Error_Handler();
    }
}

/*�������ݺ���*/
 
void CAN_Send_Test(uint32_t ID, uint8_t txdata[],uint16_t len)
{
    
	  uint32_t pTxMailbox = 0;
    TxMessage.IDE = CAN_ID_EXT;     //����ID����
//	  TxMessage.StdId = ID;
	  TxMessage.ExtId = ID;       //����ID��
    TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	  TxMessage.DLC = len;              //�������ݳ���
  	HAL_CAN_AddTxMessage(&hcan,&TxMessage,txdata,&pTxMailbox);
}


///CAN�����жϻص�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, rxbuf) != HAL_OK)
  {
    /* Reception Error */
    printf("CAN receive test data fail!\r\n");
  }
	else
	{
		printf("CAN receive test data success!\r\n");
		printf("���� ID = 0x%04x ����ϢΪ��\r\n",RxHeader.ExtId);
		for (uint8_t i = 0; i < 8; i++)
    {
			printf("%x ",rxbuf[i]);
    }
    printf("\r\n");
		printf("\r\n");
		printf("\r\n");
	}
	
}

/**
 * @brief       CAN �������ݲ�ѯ
 *   @note      �������ݸ�ʽ�̶�Ϊ: ��׼ID, ����֡
 * @param       id      : Ҫ��ѯ�� ��׼ID(11λ)
 * @param       buf     : ���ݻ�����
 * @retval      ���ս��
 *   @arg       0   , �����ݱ����յ�;
 *   @arg       ����, ���յ����ݳ���
 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0)     /* û�н��յ����� */
    {
        return 0;
    }

    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)  /* ��ȡ���� */
    {
        return 0;
    }

    if (RxHeader.ExtId!= id || RxHeader.IDE != CAN_ID_EXT || RxHeader.RTR != CAN_RTR_DATA)       /* ���յ���ID���� / ������չ֡ / ��������֡ */
    {
        return 0;    
    }

    return 1;
		
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
