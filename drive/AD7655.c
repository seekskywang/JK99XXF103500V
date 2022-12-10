/*
*********************************************************************************************************
*
*	ģ������ : AD7655���ݲɼ�
*	�ļ����� : ad7655.c
*	��    �� : V1.0
*	˵    �� : AD7655����STM32��FSMC�����ϡ�
*
*			������ʹ���� TIM4 ��ΪӲ����ʱ������ʱ����ADCת��
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
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


/* ���ù�������GPIO: PH9 PH10 PH11 */
//#define OS0_1()		GPIOH->BSRRL = GPIO_Pin_9
//#define OS0_0()		GPIOH->BSRRH = GPIO_Pin_9
//#define OS1_1()		GPIOH->BSRRL = GPIO_Pin_10
//#define OS1_0()		GPIOH->BSRRH = GPIO_Pin_10
//#define OS2_1()		GPIOH->BSRRL = GPIO_Pin_11
//#define OS2_0()		GPIOH->BSRRH = GPIO_Pin_11

/* ����ADת����GPIO : PD11  , PH12/TIM5_CH3/DCMI_D3 */
#define CONVST_1()	GPIOD->BSRR = GPIO_Pin_11
#define CONVST_0()	GPIOD->BRR = GPIO_Pin_11



/* ��ȡͨ���л�ѡ�� ---*/
#define ACH_Slect 		GPIOA->BSRR = GPIO_Pin_11

/* ͨ��ת�������ź����� ---*/
#define EOC_ConEndIN   ( ( GPIOD->IDR & GPIO_Pin_3 ) == (uint32_t)Bit_RESET )

/* ʹ�ܶ�ȡ�����ź����� ---*/
#define RD_ReadEnable     GPIOD->BRR = GPIO_Pin_4
#define RD_ReadDisable 		GPIOD->BSRR = GPIO_Pin_4

/* æ�ź�����  , �½��ؿ����ڼ�����ݲɼ����---*/
#define BusyIN   ( ( GPIOB->IDR & GPIO_Pin_3 ) == (uint32_t)Bit_RESET )


/* �����������̵�GPIO :  */
//#define RANGE_1()	GPIOH->BSRRL = GPIO_Pin_14
//#define RANGE_0()	GPIOH->BSRRH = GPIO_Pin_14

/* AD7606��λ���� : PI4  */
#define RESET_1()	GPIOD->BSRR = GPIO_Pin_13
#define RESET_0()	GPIOD->BRR = GPIO_Pin_13

/* AD7606 FSMC���ߵ�ַ��ֻ�ܶ�������д */
#define AD7606_RESULT()	*(__IO uint16_t *)0x6C400000



/* �����˲�����*/
vu16 AD7655_VMON[500];
vu16 AD7655_IMON[500];
vu16 AD7655_CHA_IMON;
vu16 AD7655_CHB_VMON;
vu32 Voltage_VPP;
vu32 Current_VPP;

AD7606_VAR_T g_tAD7606;		/* ����1��ȫ�ֱ���������һЩ���� */
AD7606_FIFO_T g_tAdcFifo;	/* ����FIFO�ṹ����� */
vu8 CH_AD7655_CONT;
static void AD7606_CtrlLinesConfig(void);
static void AD7606_FSMCConfig(void);
void DAM1_CH2_NVIC(void);
void FIft_ADC_Vloue(void);
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitAD7606
*	����˵��: ���������ⲿSRAM��GPIO��FSMC
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	AD7606_CtrlLinesConfig();
	AD7606_FSMCConfig();
	AD7606_Reset();
		
	VCH_Login;
	ACH_Slect;//����ѡ��Aͨ��
	RD_ReadEnable; 			//ʹ�ܶ�����
	CONVST_1();					/* ����ת����GPIOƽʱ����Ϊ�� */
	AD7606_StartRecord(0);
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_CtrlLinesConfig
*	����˵��: ����LCD���ƿ��ߣ�FSMC�ܽ�����Ϊ���ù���
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/*
	������STM32-V5��������߷�����

	PD0/FSMC_D2
	PD1/FSMC_D3
	PD4/FSMC_NOE		--- �������źţ�OE = Output Enable �� N ��ʾ����Ч
	PD5/FSMC_NWE		--- д�����źţ�AD7606 ֻ�ж�����д�ź�
	PD8/FSMC_D13
	PD9/FSMC_D14
	PD10/FSMC_D15

	PD14/FSMC_D0
	PD15/FSMC_D1

	PE4/FSMC_A20		--- ����Ƭѡһ������
	PE5/FSMC_A21		--- ����Ƭѡһ������
	PE7/FSMC_D4
	PE8/FSMC_D5
	PE9/FSMC_D6
	PE10/FSMC_D7
	PE11/FSMC_D8
	PE12/FSMC_D9
	PE13/FSMC_D10
	PE14/FSMC_D11
	PE15/FSMC_D12

	PG12/FSMC_NE4		--- ��Ƭѡ��TFT, OLED �� AD7606��

	�����Ŀ���IO:

	PH9/DCMI_D0/AD7606_OS0			---> AD7606_OS0		OS2:OS0 ѡ�������˲�����
	PH10/DCMI_D1/AD7606_OS1         ---> AD7606_OS1
	PH11/DCMI_D2/AD7606_OS2         ---> AD7606_OS2
	PH12/DCMI_D3/AD7606_CONVST      ---> AD7606_CONVST	����ADCת�� (CONVSTA �� CONVSTB �Ѿ�����)
	PH14/DCMI_D4/AD7606_RAGE        ---> AD7606_RAGE	����ģ���ѹ���̣�����5V������10V
	PI4/DCMI_D5/AD7606_RESET        ---> AD7606_RESET	��λ
	PI6/DCMI_D6/AD7606_BUSY         ---> AD7606_BUSY	æ�ź�	(δʹ��)

*/
static void AD7606_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��FSMCʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* ʹ�� GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOD , ENABLE);

	/* ���� PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) Ϊ����������� */
	//GPIO_PinRemapConfig(GPIO_Remap_FSMC_NADV, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |GPIO_Pin_15;	                                                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12 |GPIO_Pin_13|GPIO_Pin_14 |GPIO_Pin_15;	                                                        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	

	/*	���ü��������õ�GPIO

		PD11        ---> AD7606_CONVST	����ADCת��
		PD13        ---> AD7606_RESET	��λ
		PD4         ---> ʹ�ܶ�����
		PD3         ---> ����ת�����
		PA12        ---> ͨ���ɼ��л�
		PA11        ---> ͨ�����ݶ�ȡ�л�		
		PB3			    ---> AD7606_BUSY    ת���������ź�
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
*	�� �� ��: AD7606_FSMCConfig
*	����˵��: ����FSMC���ڷ���ʱ��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AD7606_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  init;
	FSMC_NORSRAMTimingInitTypeDef  timing;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE); 
	

	/*
		AD7606�����Ҫ��(3.3Vʱ)��RD���źŵ͵�ƽ���������21ns���ߵ�ƽ������̿��15ns��

		������������ ������������Ϊ�˺�ͬBANK��LCD������ͬ��ѡ��3-0-6-1-0-0
		3-0-5-1-0-0  : RD�߳���75ns�� �͵�ƽ����50ns.  1us���ڿɶ�ȡ8·�������ݵ��ڴ档
		1-0-1-1-0-0  : RD��75ns���͵�ƽִ��12ns���ң��½��ز��Ҳ12ns.  ���ݶ�ȡ��ȷ��
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
*	�� �� ��: FSMC_DMA_INTE
*	����˵��: FSMC_DMAģʽ����
*	��    ��: ��
*	�� �� ֵ: ��
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
  DMA_InitStruct.DMA_BufferSize =50;//����ת��100��
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
	/* 开启DAM1通道1中断 */
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
  DAM1_CH2_NVIC();//DMA�ж�����
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
*	�� �� ��: AD7606_Reset
*	����˵��: Ӳ����λAD7606����λ֮��ָ�����������״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************/
void AD7606_Reset(void)
{
	RESET_0();	/* �˳���λ״̬ */

	RESET_1();	/* ���븴λ״̬ */
	RESET_1();	/* �������ӳ١� RESET��λ�ߵ�ƽ��������С50ns�� */
	RESET_1();
	RESET_1();

	RESET_0();	/* �˳���λ״̬ */
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_StartConvst
*	����˵��: ����1��ADCת��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_StartConvst(void)
{
	/* page 7��  CONVST �ߵ�ƽ�����Ⱥ͵͵�ƽ��������� 25ns */
	/* CONVSTƽʱΪ�� */
	CONVST_0();
	CONVST_0();
	CONVST_0();

	CONVST_1();
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_ReadNowAdc
*	����˵��: ��ȡ8·�������������洢��ȫ�ֱ��� g_tAD7606
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_ReadNowAdc(void)
{
	g_tAD7606.sNowAdc[0] = AD7606_RESULT();	/* ����1·���� */
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_StartRecord
*	����˵��: ��ʼ�ɼ�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_StartRecord(uint32_t _ulFreq)
{
	AD7606_StopRecord();

	AD7606_Reset();					/* ��λӲ�� */
	AD7606_StartConvst();			/* ���������������1������ȫ0������ */

	g_tAdcFifo.usRead = 0;			/* �����ڿ���TIM2֮ǰ��0 */
	g_tAdcFifo.usWrite = 0;
	g_tAdcFifo.usCount = 0;
	g_tAdcFifo.ucFull = 0;
}
/*
*********************************************************************************************************
*	�� �� ��: AD7606_StopRecord
*	����˵��: ֹͣ�ɼ���ʱ��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_StopRecord(void)
{
	CONVST_1();					/* ����ת����GPIOƽʱ����Ϊ�� */
}

static void i2c_Delay(vu8 tim)
{
	vu8 i;

	/*��
	 	�����ʱ����ͨ��������AX-Pro�߼������ǲ��Եõ��ġ�
		CPU��Ƶ72MHzʱ�����ڲ�Flash����, MDK���̲��Ż�
		ѭ������Ϊ10ʱ��SCLƵ�� = 205KHz 
		ѭ������Ϊ7ʱ��SCLƵ�� = 347KHz�� SCL�ߵ�ƽʱ��1.5us��SCL�͵�ƽʱ��2.87us 
	 	ѭ������Ϊ5ʱ��SCLƵ�� = 421KHz�� SCL�ߵ�ƽʱ��1.25us��SCL�͵�ƽʱ��2.375us 
        
    IAR���̱���Ч�ʸߣ���������Ϊ7
	*/
	for (i = 0; i < tim; i++);
}


/*
*********************************************************************************************************
*	�� �� ��: CurtMAX_MIN(vu16 *date , vu16 num)
*	����˵��: �ҳ�һ���������ֵ����Сֵ
*	��    ��:  ��
*	�� �� ֵ: ��
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
			if(i==0)//��¼��һ�ε���Сֵ
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
			date[i] = date[i+1];//�Ƚ��������С���л�˳��
			date[i+1] = *MAX ;
		}
	}
}


/*
*********************************************************************************************************
*	�� �� ��: AD7606_StopRecord
*	����˵��: ֹͣ�ɼ���ʱ��
*	��    ��:  ��
*	�� �� ֵ: ��
**********************************************************************************************************/
#define filtnum   500

void GET_ADC_VALUE(void)
{

	static u8   step=0;

	switch( step )
	{
		case 0:
			if( BusyIN )//��æ �Ϳɶ�����
			{
				AD7655_CHB_VMON = AD7606_RESULT(); //sum/ filtnum;					
				step++; 								
			}
			break;
		
		case 1:  //�л�����ͨ��	
			ACH_Login;
			step++;
			break;	
			
		case 2:  //�л�����ͨ��	
			step++;
			break;	
		case 3:  //������ֵ
			if( BusyIN )//��æ �Ϳɶ�����
			{
				AD7655_CHA_IMON = AD7606_RESULT(); //sum / filtnum;
				step++; 														
			}
			break;	
			
			case 4:  //�л���ѹͨ��	
			VCH_Login;
			step++;
			break;
			case 5:  //�л�����ͨ��		
			step=0;
			break;	
			default: step=0; break;
	}
	FIft_ADC_Vloue();//�˲�
}

/**********************************************************************************************************
*	�� �� ��:  GET_RIppleVal(vu32  Vinput, vu32  Ainput)
*	����˵��:  ��ȡ�Ʋ�VA
*	��    ��:  ��
*	�� �� ֵ: ��
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
					MES_VOLT_PTP = ( maxmin[0]- maxmin[1] ); //��ѹ�Ʋ� = 100�����е����ֵ��ȥ��Сֵ
					MES_VOLT_MAX=maxmin[0];
					MES_VOLT_MIN=maxmin[1];
					Vcount=0;
					step++; 
				}					
			break;
		
		case 1:  //������ֵ
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
					MES_CURR_PTP = ( maxmin[0]- maxmin[1] ); //��ѹ�Ʋ� = 100�����е����ֵ��ȥ��Сֵ
					Acount=0;
					step=0; 
				}													
			break;	
			default: step=0; break;
	}
}
/**********************************************************************************************************
*	�� �� ��:  FIft_ADC_Vloue
*	����˵��:  ADֵ�˲�
*	��    ��:  ��
*	�� �� ֵ: ��
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
/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
