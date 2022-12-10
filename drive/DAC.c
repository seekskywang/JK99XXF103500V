/******************** (C) COPYRIGHT 2015 AVER ********************
 * �ļ���  ��DAC.C
 * ����    ������
 * ����    ���⺯����
 * ����    ������16BIT DAC
 * Ӳ�����ӣ�
 * �޸����ڣ�2015-8-18
*********************************************************************/
#include "my_register.h" 
#include "dac.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/**
  * @brief  ʹ��DAC��ʱ�ӣ���ʼ��GPIO
  * @param  ��
  * @retval ��
  */
/********************************************************************/
#define DAC_SYNC_LOW   GPIO_ResetBits(GPIOB, GPIO_Pin_1);
#define DAC_SYNC_HIGH  GPIO_SetBits(GPIOB, GPIO_Pin_1);   
      
#define DAC_SCK_LOW   GPIO_ResetBits(GPIOB, GPIO_Pin_0);
#define DAC_SCK_HIGH  GPIO_SetBits(GPIOB, GPIO_Pin_0); 

#define DAC_SDA_LOW   GPIO_ResetBits(GPIOC, GPIO_Pin_5);
#define DAC_SDA_HIGH  GPIO_SetBits(GPIOC, GPIO_Pin_5); 

#define DAC_LDAC_LOW   GPIO_ResetBits(GPIOC, GPIO_Pin_4);
#define DAC_LDAC_HIGH  GPIO_SetBits(GPIOC, GPIO_Pin_4); 
/**********************************************************************************************************
*	�� �� ��: AD5541_GPIOCoing
*	����˵��: ���ų�ʼ��
*	��    �Σ�
*	�� �� ֵ: ��
**********************************************************************************************************/
void AD5541_GPIOCoing(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//����GPIO�ṹ��
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_2MHz;//
	GPIO_Init(GPIOC, &GPIO_InitStruct);//
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_2MHz;//
	GPIO_Init(GPIOB, &GPIO_InitStruct);//
	
}
/**********************************************************************************************************
*	�� �� ��: AD5541_GPIOCoing
*	����˵��: ���ų�ʼ��
*	��    �Σ�
*	�� �� ֵ: ��
**********************************************************************************************************/
void AD5541_Send(vu16 date)
{
	vu8 cont;
	DAC_SYNC_LOW;//SYNC�õ�
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	NOP;
	NOP;
	DAC_SYNC_HIGH;//SYNC�ø�
	NOP;
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	DAC_SYNC_LOW;//SYNC�õ�
	for(cont = 0; cont < 16; cont++)
	{
		if(date & 0x8000)		// �ȴ���λ
		{
			DAC_SDA_HIGH;
		}
		else
		{
			DAC_SDA_LOW;
		}
		DAC_SCK_LOW;//SCK�õ�
		NOP;
		NOP;
		DAC_SCK_HIGH;//SCK�ø�
		NOP;
		NOP;
		date=date<<1;
	}
	DAC_SYNC_HIGH;//SYNC�ø�
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	NOP;
	DAC_LDAC_LOW;
	NOP;
	NOP;
	NOP;
	DAC_LDAC_HIGH;
}

/*
void AD5541_Send(vu16 date)
{
	vu8 cont;
	DAC_SYNC_LOW;//SYNC�õ�
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	NOP;
	NOP;
	DAC_SYNC_HIGH;//SYNC�ø�
	NOP;
	DAC_SCK_HIGH;//SCK�ø�
	DAC_SYNC_LOW;//SYNC�õ�
	for(cont = 0; cont < 16; cont++)
	{
		if(cont>7)
		{
			if(date & 0x8000)		// �ȴ���λ
			{
				DAC_SDA_HIGH;
			}
			else
			{
				DAC_SDA_LOW;
			}
			date=date<<1;
		}
		else//ǰ8λ����ǰ6λΪ��Чλ����2λΪ����ģʽ�趨���˴�Ϊ00����Ϊ��������ģʽ��
		{
			DAC_SDA_LOW;
		}
		DAC_SCK_LOW;//SCK�õ�
		NOP;
		NOP;
		NOP;
		DAC_SCK_HIGH;//SCK�ø�
	}
	DAC_SYNC_HIGH;//SYNC�ø�
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	NOP;
	DAC_LDAC_LOW;
	NOP;
	NOP;
	NOP;
	DAC_LDAC_HIGH;
}
*/

