
#ifndef  __SYS_IO_CFG_H
#define  __SYS_IO_CFG_H

#include "my_register.h"
#include "stm32f10x.h"


#define  IO_CC_MODE 					do {GPIOC->BRR = GPIO_Pin_10 ;flag_CC_MODE=1;}while(0) 	/* CC/CV 模式引脚 ---*/
#define  IO_CV_MODE 					do {GPIOC->BSRR = GPIO_Pin_10;flag_CC_MODE=0;}while(0)

#define  IO_CURR_HIG_Level 		do {GPIOA->BSRR = GPIO_Pin_7;Flag_I_LOW=0; CURR_Range=0;}while(0)	/* 电流高低档模式引脚 ---*/
#define  IO_CURR_LOW_Level 		do {GPIOA->BRR = GPIO_Pin_7;Flag_I_LOW=1; CURR_Range=1;}while(0)

#define  IO_VOLT_HIG_Level 		do {GPIOB->BSRR = GPIO_Pin_14; flag_V_level=1; VOLT_Range=0;}while(0)		/* 电压高低档模式引脚 ---*/
#define  IO_VOLT_LOW_Level 		do {GPIOB->BRR = GPIO_Pin_14; flag_V_level=0; VOLT_Range=1;}while(0)

#define  IO_LOAD_OFF 					do {GPIOA->BSRR = GPIO_Pin_15 ;Flag_Swtich_ON=0;}while(0)	/* 负载开关引脚 ---*/
#define  IO_LOAD_ON 					do {GPIOA->BRR = GPIO_Pin_15  ;Flag_Swtich_ON=1;}while(0)

#define  IO_BEEP_ON 					GPIOA->BSRR = GPIO_Pin_5		/* 蜂鸣器引脚 ---*/
#define  IO_BEEP_OFF 					GPIOA->BRR = GPIO_Pin_5

#define  IO_FAN_ON 						GPIOB->BSRR = GPIO_Pin_15		/* 风扇引脚 ---*/
#define  IO_FAN_OFF 					GPIOB->BRR = GPIO_Pin_15

#endif




/*----------------------------------COPY RIGHT BY XQ_GUO @2019---------------------------------------------*/

