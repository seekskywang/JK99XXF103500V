/*
*********************************************************************************************************
*
*	模块名称 : AD7655数据采集
*	文件名称 : ad7655.c
*	版    本 : V1.0
*	说    明 : AD7655挂在STM32的FSMC总线上。
*
*			本例子使用了 TIM4 作为硬件定时器，定时启动ADC转换
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*
*	Copyright (C), 2013-2014
*
*********************************************************************************************************/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "AD7655.h"
#include "sys_io_cfg.h"
#include "me_scpi.h"


/* 设置过采样的GPIO: PH9 PH10 PH11 */
//#define OS0_1()		GPIOH->BSRRL = GPIO_Pin_9
//#define OS0_0()		GPIOH->BSRRH = GPIO_Pin_9
//#define OS1_1()		GPIOH->BSRRL = GPIO_Pin_10
//#define OS1_0()		GPIOH->BSRRH = GPIO_Pin_10
//#define OS2_1()		GPIOH->BSRRL = GPIO_Pin_11
//#define OS2_0()		GPIOH->BSRRH = GPIO_Pin_11

/* 启动AD转换的GPIO : PD11  , PH12/TIM5_CH3/DCMI_D3 */
#define CONVST_1()	GPIOD->BSRR = GPIO_Pin_11
#define CONVST_0()	GPIOD->BRR = GPIO_Pin_11



/* 读取通道切换选择 ---*/
#define ACH_Slect 		GPIOA->BSRR = GPIO_Pin_11

/* 通道转换结束信号输入 ---*/
#define EOC_ConEndIN   ( ( GPIOD->IDR & GPIO_Pin_3 ) == (uint32_t)Bit_RESET )

/* 使能读取数据信号输入 ---*/
#define RD_ReadEnable     GPIOD->BRR = GPIO_Pin_4
#define RD_ReadDisable 		GPIOD->BSRR = GPIO_Pin_4

/* 忙信号输入  , 下降沿可用于检测数据采集完成---*/
#define BusyIN   ( ( GPIOB->IDR & GPIO_Pin_3 ) == (uint32_t)Bit_RESET )


/* 设置输入量程的GPIO :  */
//#define RANGE_1()	GPIOH->BSRRL = GPIO_Pin_14
//#define RANGE_0()	GPIOH->BSRRH = GPIO_Pin_14

/* AD7606复位口线 : PI4  */
#define RESET_1()	GPIOD->BSRR = GPIO_Pin_13
#define RESET_0()	GPIOD->BRR = GPIO_Pin_13

/* AD7606 FSMC总线地址，只能读，无需写 */
#define AD7606_RESULT()	*(__IO uint16_t *)0x6C400000



/* 定义滤波数组*/
vu16 AD7655_VMON[500];
vu16 AD7655_IMON[500];
vu16 AD7655_CHA_IMON;
vu16 AD7655_CHB_VMON;
vu32 Voltage_VPP;
vu32 Current_VPP;

AD7606_VAR_T g_tAD7606;		/* 定义1个全局变量，保存一些参数 */
AD7606_FIFO_T g_tAdcFifo;	/* 定义FIFO结构体变量 */
vu8 CH_AD7655_CONT;
static void AD7606_CtrlLinesConfig(void);
static void AD7606_FSMCConfig(void);
void DAM1_CH2_NVIC(void);
void FIft_ADC_Vloue(void);
/*
*********************************************************************************************************
*	函 数 名: bsp_InitAD7606
*	功能说明: 配置连接外部SRAM的GPIO和FSMC
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	AD7606_CtrlLinesConfig();
	AD7606_FSMCConfig();
	AD7606_Reset();
		
	VCH_Login;
	ACH_Slect;//数据选择A通道
	RD_ReadEnable; 			//使能读数据
	CONVST_1();					/* 启动转换的GPIO平时设置为高 */
	AD7606_StartRecord(0);
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_CtrlLinesConfig
*	功能说明: 配置LCD控制口线，FSMC管脚设置为复用功能
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
/*
	安富莱STM32-V5开发板接线方法：

	PD0/FSMC_D2
	PD1/FSMC_D3
	PD4/FSMC_NOE		--- 读控制信号，OE = Output Enable ， N 表示低有效
	PD5/FSMC_NWE		--- 写控制信号，AD7606 只有读，无写信号
	PD8/FSMC_D13
	PD9/FSMC_D14
	PD10/FSMC_D15

	PD14/FSMC_D0
	PD15/FSMC_D1

	PE4/FSMC_A20		--- 和主片选一起译码
	PE5/FSMC_A21		--- 和主片选一起译码
	PE7/FSMC_D4
	PE8/FSMC_D5
	PE9/FSMC_D6
	PE10/FSMC_D7
	PE11/FSMC_D8
	PE12/FSMC_D9
	PE13/FSMC_D10
	PE14/FSMC_D11
	PE15/FSMC_D12

	PG12/FSMC_NE4		--- 主片选（TFT, OLED 和 AD7606）

	其他的控制IO:

	PH9/DCMI_D0/AD7606_OS0			---> AD7606_OS0		OS2:OS0 选择数字滤波参数
	PH10/DCMI_D1/AD7606_OS1         ---> AD7606_OS1
	PH11/DCMI_D2/AD7606_OS2         ---> AD7606_OS2
	PH12/DCMI_D3/AD7606_CONVST      ---> AD7606_CONVST	启动ADC转换 (CONVSTA 和 CONVSTB 已经并联)
	PH14/DCMI_D4/AD7606_RAGE        ---> AD7606_RAGE	输入模拟电压量程，正负5V或正负10V
	PI4/DCMI_D5/AD7606_RESET        ---> AD7606_RESET	复位
	PI6/DCMI_D6/AD7606_BUSY         ---> AD7606_BUSY	忙信号	(未使用)

*/
static void AD7606_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能FSMC时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* 使能 GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOD , ENABLE);

	/* 设置 PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) 为复用推挽输出 */
	//GPIO_PinRemapConfig(GPIO_Remap_FSMC_NADV, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |GPIO_Pin_15;	                                                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13|GPIO_Pin_14 |GPIO_Pin_15;	                                                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	

	/*	配置几个控制用的GPIO

		PD11        ---> AD7606_CONVST	启动ADC转换
		PD13        ---> AD7606_RESET	复位
		PD4         ---> 使能读数据
		PD3         ---> 数据转换完成
		PA12        ---> 通道采集切换
		PA11        ---> 通道数据读取切换		
		PB3			    ---> AD7606_BUSY    转换结束的信号
	*/
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_13 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_FSMCConfig
*	功能说明: 配置FSMC并口访问时序
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AD7606_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  init;
	FSMC_NORSRAMTimingInitTypeDef  timing;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE); 
	

	/*
		AD7606规格书要求(3.3V时)：RD读信号低电平脉冲宽度最短21ns，高电平脉冲最短宽度15ns。

		按照如下配置 读数均正常。为了和同BANK的LCD配置相同，选择3-0-6-1-0-0
		3-0-5-1-0-0  : RD高持续75ns， 低电平持续50ns.  1us以内可读取8路样本数据到内存。
		1-0-1-1-0-0  : RD高75ns，低电平执行12ns左右，下降沿差不多也12ns.  数据读取正确。
	*/
	/* FSMC_Bank1_NORSRAM4 configuration */
	timing.FSMC_AddressSetupTime = 3;
	timing.FSMC_AddressHoldTime = 0;
	timing.FSMC_DataSetupTime = 6;
	timing.FSMC_BusTurnAroundDuration = 1;
	timing.FSMC_CLKDivision = 0;
	timing.FSMC_DataLatency = 0;
	timing.FSMC_AccessMode = FSMC_AccessMode_A;

	/*
	 LCD configured as follow:
	    - Data/Address MUX = Disable
	    - Memory Type = SRAM
	    - Data Width = 16bit
	    - Write Operation = Enable
	    - Extended Mode = Enable
	    - Asynchronous Wait = Disable
	*/
	init.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	init.FSMC_WrapMode = FSMC_WrapMode_Disable;
	init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	init.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	init.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

	init.FSMC_ReadWriteTimingStruct = &timing;
	init.FSMC_WriteTimingStruct = &timing;

	FSMC_NORSRAMInit(&init);

	/* - BANK 1 (of NOR/SRAM Bank 1~4) is enabled */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/*********************************************************************************************************
*	函 数 名: FSMC_DMA_INTE
*	功能说明: FSMC_DMA模式配置
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
void FSMC_DMA_INTE(void)
{
	DMA_InitTypeDef     DMA_InitStruct;
	/* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	/* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStruct.DMA_PeripheralBaseAddr = (vu32)0x6C400000;
  //DMA_InitStruct.DMA_MemoryBaseAddr = (vu32)&AD7655_Buffer;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStruct.DMA_BufferSize =50;//连续转换100次
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_M2M = DMA_M2M_Enable;
  DMA_Init(DMA1_Channel2, &DMA_InitStruct);
  
  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel2, ENABLE);
	/* 寮�鍚疍AM1閫氶亾1涓柇 */
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
  DAM1_CH2_NVIC();//DMA中断配置
}


void DAM1_CH2_NVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the adc1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*********************************************************************************************************
*	函 数 名: AD7606_Reset
*	功能说明: 硬件复位AD7606。复位之后恢复到正常工作状态。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
void AD7606_Reset(void)
{
	RESET_0();	/* 退出复位状态 */

	RESET_1();	/* 进入复位状态 */
	RESET_1();	/* 仅用于延迟。 RESET复位高电平脉冲宽度最小50ns。 */
	RESET_1();
	RESET_1();

	RESET_0();	/* 退出复位状态 */
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartConvst
*	功能说明: 启动1次ADC转换
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartConvst(void)
{
	/* page 7：  CONVST 高电平脉冲宽度和低电平脉冲宽度最短 25ns */
	/* CONVST平时为高 */
	CONVST_0();
	CONVST_0();
	CONVST_0();

	CONVST_1();
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadNowAdc
*	功能说明: 读取8路采样结果。结果存储在全局变量 g_tAD7606
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_ReadNowAdc(void)
{
	g_tAD7606.sNowAdc[0] = AD7606_RESULT();	/* 读第1路样本 */
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartRecord
*	功能说明: 开始采集
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartRecord(uint32_t _ulFreq)
{
	AD7606_StopRecord();

	AD7606_Reset();					/* 复位硬件 */
	AD7606_StartConvst();			/* 启动采样，避免第1组数据全0的问题 */

	g_tAdcFifo.usRead = 0;			/* 必须在开启TIM2之前清0 */
	g_tAdcFifo.usWrite = 0;
	g_tAdcFifo.usCount = 0;
	g_tAdcFifo.ucFull = 0;
}
/*
*********************************************************************************************************
*	函 数 名: AD7606_StopRecord
*	功能说明: 停止采集定时器
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StopRecord(void)
{
	CONVST_1();					/* 启动转换的GPIO平时设置为高 */
}

static void i2c_Delay(vu8 tim)
{
	vu8 i;

	/*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	for (i = 0; i < tim; i++);
}


/*
*********************************************************************************************************
*	函 数 名: CurtMAX_MIN(vu16 *date , vu16 num)
*	功能说明: 找出一个数的最大值和最小值
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************/
void CurtMAX_MIN(vu32 *date , vu16 num , vu32 *MAX , vu32 *MIN)
{
//	vu16  max=0;
//	vu16  min=0;
	vu16  i;
	
	for(i=0; i<num -1; i++)
	{
		if( date[i+1] > date[i] )
		{
			*MAX  = date[i+1];
			if(i==0)//记录第一次的最小值
			{
				*MIN = date[i];
			}
			else 
			{
				if(date[i]< *MIN)
				{
					*MIN = date[i];
				}
			}
		}
		else 
		{
			*MAX  = date[i];
			if(i==0)
			{
				*MIN = date[i+1];
			}
			else 
			{
				if(date[i+1]< *MIN)
				{
					*MIN = date[i+1];
				}
			}		
			date[i] = date[i+1];//比较完最大最小后切换顺序
			date[i+1] = *MAX ;
		}
	}
}


/*
*********************************************************************************************************
*	函 数 名: AD7606_StopRecord
*	功能说明: 停止采集定时器
*	形    参:  无
*	返 回 值: 无
**********************************************************************************************************/
#define filtnum   500

void GET_ADC_VALUE(void)
{

	static u8   step=0;

	switch( step )
	{
		case 0:
			if( BusyIN )//不忙 就可读数据
			{
				AD7655_CHB_VMON = AD7606_RESULT(); //sum/ filtnum;					
				step++; 								
			}
			break;
		
		case 1:  //切换电流通道	
			ACH_Login;
			step++;
			break;	
			
		case 2:  //切换电流通道	
			step++;
			break;	
		case 3:  //读电流值
			if( BusyIN )//不忙 就可读数据
			{
				AD7655_CHA_IMON = AD7606_RESULT(); //sum / filtnum;
				step++; 														
			}
			break;	
			
			case 4:  //切换电压通道	
			VCH_Login;
			step++;
			break;
			case 5:  //切换电流通道		
			step=0;
			break;	
			default: step=0; break;
	}
	FIft_ADC_Vloue();//滤波
}

/**********************************************************************************************************
*	函 数 名:  GET_RIppleVal(vu32  Vinput, vu32  Ainput)
*	功能说明:  读取纹波VA
*	形    参:  无
*	返 回 值: 无
**********************************************************************************************************/
void GET_RIppleVal(vu32  Vinput, vu32  Ainput)
{
	static u16  	Vcount=0 , Acount=0;
	static u32  Vadc_buff[500], Aadc_buff[500];
	static u8   step=0;
	u32  maxmin[3];
	switch( step )
	{
		case 0:
				if(Vcount< filtnum)
				{
					Vadc_buff[ Vcount ] = Vinput;
					Vcount++;
				}
				else 
				{
					CurtMAX_MIN( &Vadc_buff[3] , filtnum-3 , &maxmin[0] , &maxmin[1]);	
					MES_VOLT_PTP = ( maxmin[0]- maxmin[1] ); //电压纹波 = 100个数中的最大值减去最小值
					MES_VOLT_MAX=maxmin[0];
					MES_VOLT_MIN=maxmin[1];
					Vcount=0;
					step++; 
				}					
			break;
		
		case 1:  //读电流值
				if(Acount < filtnum)
				{
					Aadc_buff[ Acount ] = Ainput;
					Acount++;
				}
				else 
				{				
					CurtMAX_MIN( &Aadc_buff[3] , filtnum-3 , &maxmin[0] , &maxmin[1]);		
					MES_CURR_MAX=maxmin[0];
					MES_CURR_MIN=maxmin[1];
					MES_CURR_PTP = ( maxmin[0]- maxmin[1] ); //电压纹波 = 100个数中的最大值减去最小值
					Acount=0;
					step=0; 
				}													
			break;	
			default: step=0; break;
	}
}
/**********************************************************************************************************
*	函 数 名:  FIft_ADC_Vloue
*	功能说明:  AD值滤波
*	形    参:  无
*	返 回 值: 无
**********************************************************************************************************/
void FIft_ADC_Vloue(void)
{
	vu16 i,f;
	vu32 sum1;
	static vu16 I_cont,V_cont;
	AD7655_IMON[I_cont++]=AD7655_CHA_IMON;
	if(I_cont>500)
	{
		I_cont=0;
		sum1=0;
		for(f=0;f<500;f++)
		{
			sum1 +=AD7655_IMON[f];
		}
		Imon_value=sum1/500;
	}
	
	AD7655_VMON[V_cont++]=AD7655_CHB_VMON;
	if(V_cont>500)
	{
		V_cont=0;
		sum1=0;
		for(i=0;i<500;i++)
		{
			sum1 +=AD7655_VMON[i];
		}
		Vmon_value=sum1/500;
	}
}



void AD7655_TASK(void )
{
	AD7606_StartConvst();
	i2c_Delay(10);
	GET_ADC_VALUE();
	i2c_Delay(100);
}
/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
