/******************************************************************/
/* ���ƣHTIM3 PWM                                                */
/* Ч����                                                        */
/* ���ݣ�����һ��200HZ ��ռ�ձȣ�60.9% ��ռ�ձȣ�30.9%��PWM      */
/* ���ߣ�����                                                    */
/* ��ϵ��ʽ��QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "tim6.h"
/*****************************************************************/
/*****************************************************************/
void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 ������ ---------------------------------------------------
   TIM3 ����ʱ��(TIM3CLK) ����Ϊ APB2 ʱ�� (PCLK2)    
    => TIM3CLK = PCLK2 = SystemCoreClock
   TIM3CLK = SystemCoreClock, Prescaler = 0, TIM3 counter clock = SystemCoreClock
   SystemCoreClock Ϊ48 MHz */
  /* TIM16 ʱ��ʹ�� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
  /* Time ��ʱ��������*/
  TIM_TimeBaseStructure.TIM_Prescaler = 2000;//ʱ��Ԥ��Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  /* Time ��ʱ����Ϊ�����ؼ���ģʽ*/
  TIM_TimeBaseStructure.TIM_Period = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//������ʱ�������ж�
	TIM_SetAutoreload(TIM6, 0xFF);//����PWM�ֱ���
	TIM6_NVIC_Config();
  /* TIM3 ������ʹ��*/
  TIM_Cmd(TIM6, ENABLE);
}
/***********************************************************************/
static void TIM6_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
