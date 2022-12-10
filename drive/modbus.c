/******************** (C) COPYRIGHT 2019 AVER********************
 * �ļ���  :MODBUS.C
 * ����   :
 * ����    :MODBUS?????
 * ����    :KL220A_Master
 * Ӳ������: 485
 * �޸�����:2014-12-22
********************************************************************/
#include "my_register.h" 
#include "usart.h" 
#include "modbus.h" 
#include "stm32f10x.h"
#include "flash.h"
//===================================================================//
extern struct bitDefine
{
	unsigned bit0: 1;
	unsigned bit1: 1;
	unsigned bit2: 1;
	unsigned bit3: 1;
	unsigned bit4: 1;
	unsigned bit5: 1;
	unsigned bit6: 1;
	unsigned bit7: 1;
} flagA,flagB,flagC,flagD,flagE;
/*************************У׼����************************************/
vu8 DAC_Flag;//DAC�Ƿ���ر�־
vu32 Modify_A_READ;
vu32 Modify_C_READ;
vu32 Modify_A_ACT;
	
vu32 Modify_B_READ;
vu32 Modify_D_READ;
vu32 Modify_B_ACT;

vu32 var16;
vu32 var32a;
vu32 var32b;

vu32 var16a;
vu32 var32c;
vu32 var32d;

vu16 Contr_DACVlue;//DAC����ֵ
vu32 Correct_Parametet[26];//У׼����
vu32 Correct_Strong[26];//У׼ϵ��
vu8 coefficient[13];
/*************************��������***********************************/
vu32 Run_Control[55];//�������ݼĴ���
vu32 Transition_Date[10];//���ɱ���
//============================��������===============================//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count);
//===========================MODBUSЭ��=============================//
void UART_Action(void)
{//RUT��ʽ��
	//ADDR  ������  ���Ĵ�������ʼ��ַ��   ���Ĵ�������ʼ��ַ��  �������ָ������ֽ�   �����ݸ������ֽ�  CRC�� CRC��
	//���ظ�ʽ��ADDR ������ ���������ֽ���  ���ݸ�  ���ݵ� ..... CRC��  CRC��
	if ((UART_Buffer_Rece[0] == 0x01)||(UART_Buffer_Rece[0] == 0))
	{
		if (UART_Buffer_Rece[1] == (0x03))	//����3 ������   
		{																		 
			vu8 i;
			vu16 crc_result;
			crc_result = (UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];
			if ((crc_result == Hardware_CRC(UART_Buffer_Rece,6)) ||(crc_result == 0) )
			{
				if (UART_Buffer_Rece[3] < 0xFF)    								//����Ĵ����ڿɶ���Χ��
				{
					if ((UART_Buffer_Rece[3] + UART_Buffer_Rece[5]) < 0xFF)		//������һ����ȡ�ļĴ�����ַ�ڿɶ���Χ��
					{							
						UART_Buffer_Send[0] = 0x01;
						UART_Buffer_Send[1] = 0x03;
						UART_Buffer_Send[2] = UART_Buffer_Rece[5]*4;
						for (i=0;i<UART_Buffer_Send[2];i++)
						{
							if ((i % 4) == 0) 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 24;//��λ��ǰ
							}
							else if((i % 4) == 1)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 16;//��λ��ǰ
							}
							else if((i % 4) == 2)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 8;//��λ��ǰ
							}
							else 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4];			
							}															
						}
						crc_result = Hardware_CRC(UART_Buffer_Send,UART_Buffer_Send[2] + 3);
						UART_Buffer_Send[3 + UART_Buffer_Send[2]] = crc_result >> 8;
						UART_Buffer_Send[4 + UART_Buffer_Send[2]] = crc_result;
						Transmit_BUFFERsize = UART_Buffer_Send[2] + 5;
						UART_SEND_flag=1;
						UART2_Send();
					}
				}
			}	
		}
	} 
//===============================д�Ĵ���=================================
	if ((UART_Buffer_Rece[0] == 0x01) || (UART_Buffer_Rece[0] == 0))	 
	{
		vu8 var8;
		vu8 a=0;
		vu16 var16;
		vu16 crc_result;
		vu32 var32;
//=========================��������6 д�����Ĵ���===========================
		if (UART_Buffer_Rece[1] == 0X06)                                 //�жϵڶ����ֽ��Ƿ�Ϊ����6
		{
			if (UART_Buffer_Rece[3] < 0xFF)							  //�ж���Ҫд�ĵ�ַ�Ƿ��ڿ�д��Χ��
			{
				crc_result = (UART_Buffer_Rece[8] << 8) + UART_Buffer_Rece[9];
				if ((crc_result == Hardware_CRC(UART_Buffer_Rece,8)) ||(crc_result == 0) )		  //���CRC
				{
					var32 = (UART_Buffer_Rece[4] << 8) + UART_Buffer_Rece[5];	//��5 6���ֽ�ΪҪд�������
					var16=(UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];	//��6  7���ֽ�ΪҪд�������
					var8 = UART_Buffer_Rece[3];	        						//��3 4���ֽ�ΪҪд��ĵ�ַ
					var32=(var32<<16)+var16;
					Run_Control[var8] = var32;			    //������д��ָ���ĵ�ַ

					if (UART_Buffer_Rece[0] == 0x01)							//�㲥ģʽ�²���������
					{
						for (a=0;a<10;a++)
						{UART_Buffer_Send[a] = UART_Buffer_Rece[a];}
						Transmit_BUFFERsize = 10;						//ԭ�����ݷ��أ�������CRC
						UART_SEND_flag=1;
						UART2_Send();
					}
				}
			}
		}
//=======================================����������16����д�Ĵ���===========================================
//������16��ʽ:
//     ��ַ ���� д����ʼ��ַ��  д����ʼ��ַ�� д��������  д�������� д��Ĵ�������  ���ݸ� ���ݵ� ......CRC�� CRC��
//�������ݸ�ʽ:
//     ��ַ ���� д����ʼ��ַ��  д����ʼ��ַ��  д���ֽ����� д���ֽ�����  CRC��  CRC�� 
		if (UART_Buffer_Rece[1] == 0X10)										  
		{	
			crc_result = ((UART_Buffer_Rece[(UART_Buffer_Rece[6]*4)+7]) << 8) + UART_Buffer_Rece[(UART_Buffer_Rece[6]*4)+8];
			if (crc_result == Hardware_CRC(UART_Buffer_Rece,(UART_Buffer_Rece[6]*4)+7)) 	  //���CRC
			{												
				for (var8=0;var8<UART_Buffer_Rece[6];var8++) 
				{
					var32 = (UART_Buffer_Rece[var8*4+7] << 8) + UART_Buffer_Rece[var8*4+8];
					var16=(UART_Buffer_Rece[var8*4+9] << 8) + UART_Buffer_Rece[var8*4+10];	//������д�����32λ�Ĵ���������*4
					var32=(var32<<16)+var16;
					Run_Control[UART_Buffer_Rece[3]+var8] = var32;			    //������д��ָ���ĵ�ַ
				}

				if (UART_Buffer_Rece[0] == 0x01)					  //�㲥ģʽ����������
				{
					UART_Buffer_Send[0] = ADDR;
					UART_Buffer_Send[1] = 16;
					UART_Buffer_Send[2] = UART_Buffer_Rece[2];
					UART_Buffer_Send[3] = UART_Buffer_Rece[3];
					UART_Buffer_Send[4] = UART_Buffer_Rece[4];
					UART_Buffer_Send[5] = UART_Buffer_Rece[5];
					crc_result = Hardware_CRC(UART_Buffer_Send,6);	 //����CRC��
					UART_Buffer_Send[6] = crc_result>>8;
					UART_Buffer_Send[7] = crc_result;				 
					Transmit_BUFFERsize = 8;					     //���÷����ֽ�������
					UART_SEND_flag=1;
					UART2_Send();
				}
			}
		}			 
	}
/*************************************����ΪУ׼����**************************************************************************/
	if ((UART_Buffer_Rece[0] == 0x01)&&(UART_Buffer_Rece[1] == 0xA5))	 
	{
		vu16 crc_result;
		crc_result = (UART_Buffer_Rece[7] << 8) + UART_Buffer_Rece[8];
	    if(crc_result == Hardware_CRC(UART_Buffer_Rece,7))
		{
	/*******************��ѹ�����͵�У׼**************************************/	
			if(UART_Buffer_Rece[2] == 0x01)
			{
//				x1 = Vmon_value;
//				x3 = Contr_DACVlue;
//				y1 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y1 = (y1<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y3 = y1;
				
				Modify_A_READ = Vmon_value;//������ѹֵ
				Modify_C_READ = Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
			}
			if (UART_Buffer_Rece[2] == 0x02)			   //��ѹ����У׼���
			{
//				x2 = Vmon_value;
//				x4 = Contr_DACVlue;
//				y2 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y2 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y4 = y2;
//				
//				CalPara.TestVL = ((float)y2 - (float)y1)/((float)x2 - (float)x1);
//				CalPara.OffsetVL = (float)y2 - CalPara.TestVL*(float)x2;
//				
//				CalPara.SetVL = ((float)y4 - (float)y3)/((float)x4 - (float)x3);
//				CalPara.OffsetSetVL = (float)y4 - CalPara.SetVL*(float)x4;
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionV_LOW = var32a;
				var32a=0;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionV_LOW;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_LOW = var32b;
					Polar0 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_LOW = var32a;
					Polar0 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
//				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
//				var32c = var32c - Modify_A_ACT;
//				var32c = var32c << 12;
//				var16a=Modify_D_READ-Modify_C_READ;
//				var16a=(var16a*2);
//				var32c=var32c/var16a;
//				SET_CorrectionV_LOW = var32c;
//				var32c = Modify_B_ACT;
//				var32c = var32c << 12;
//				var32d = SET_CorrectionV_LOW;
//				var32d = var32d * (Modify_D_READ*2);
//				if (var32c < var32d)
//				{
//					var32d = var32d - var32c;
//					SET_ReadV_Offset_LOW = var32d;
//					Polar0 |= 0x04;
//				}
//				else 
//				{
//					var32c = var32c - var32d;
//					SET_ReadV_Offset_LOW = var32c;
//					Polar0 &= ~0x04;
//				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			/*******************��ѹ�����ߵ�λУ׼**************************************/	
			if(UART_Buffer_Rece[2] == 0x03)
			{
//				x1 = Vmon_value;
//				x3 = Contr_DACVlue;
//				y1 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y1 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y3 = y1;
				
				Modify_A_READ = Vmon_value;//������ѹֵ
				Modify_C_READ = Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
			}
			/*******************��ѹ�����ж�У׼*************************/	
			if (UART_Buffer_Rece[2] == 0x04)			   
			{
//				x2 = Vmon_value;
//				x4 = Contr_DACVlue;
//				y2 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y2 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y4 = y2;
//				
//				CalPara.TestVM = ((float)y2 - (float)y1)/((float)x2 - (float)x1);
//				CalPara.OffsetVM = (float)y2 - CalPara.TestVM*(float)x2;
//				
//				CalPara.SetVM = ((float)y4 - (float)y3)/((float)x4 - (float)x3);
//				CalPara.OffsetSetVM = (float)y4 - CalPara.SetVM*(float)x4;
				
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionV_HIG = var32a;
				var32a=0;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionV_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_HIG = var32b;
					Polar1 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_HIG = var32a;
					Polar1 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
//				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
//				var32c = var32c - Modify_A_ACT;
//				var32c = var32c << 12;
//				var16a=Modify_D_READ-Modify_C_READ;
//				var16a=(var16a*2);
//				var32c=var32c/var16a;
//				SET_CorrectionV_HIG = var32c;
//				var32c = Modify_B_ACT;
//				var32c = var32c << 12;
//				var32d = SET_CorrectionV_HIG;
//				var32d = var32d * (Modify_D_READ*2);
//				if (var32c < var32d)
//				{
//					var32d = var32d - var32c;
//					SET_ReadV_Offset_HIG = var32d;
//					Polar1 |= 0x04;
//				}
//				else 
//				{
//					var32c = var32c - var32d;
//					SET_ReadV_Offset_HIG = var32c;
//					Polar1 &= ~0x04;
//				}
				Flash_Write_all();	//����д��FLASH
			}
			/*******************��ѹ�����Ϳ��Ƹ߶�У׼*****************/
			if (UART_Buffer_Rece[2] == 0x05)			   //��ѹ����У׼���
			{
//				x5 = Vmon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestVH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetVH = (float)y5 - CalPara.TestVH*(float)x5;
//				
//				CalPara.SetVH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetVH = (float)y6 - CalPara.SetVH*(float)x6;

//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ=0;
				Modify_C_READ=0;
				Modify_A_ACT=0;
				Modify_A_READ =Vmon_value;//������ѹֵ
				Modify_C_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionV_MID_HIG = var32a;
				var32a=0;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionV_MID_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_MID_HIG = var32b;
					Polar4 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_MID_HIG = var32a;
					Polar4 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
//				var32c = Modify_A_ACT; //CVģʽ��ѹ����У׼
//				var32c = var32c - Modify_B_ACT;
//				var32c = var32c << 12;
//				var16a=Modify_C_READ-Modify_D_READ;
//				var16a=(var16a*2);
//				var32c=var32c/var16a;
//				SET_CorrectionV_MID_HIG = var32c;
//				var32c = Modify_A_ACT;
//				var32c = var32c << 12;
//				var32d = SET_CorrectionV_MID_HIG;
//				var32d = var32d * (Modify_C_READ*2);
//				if (var32c < var32d)
//				{
//					var32d = var32d - var32c;
//					SET_ReadV_Offset_MID_HIG = var32d;
//					Polar4 |= 0x04;
//				}
//				else 
//				{
//					var32c = var32c - var32d;
//					SET_ReadV_Offset_MID_HIG = var32c;
//					Polar4 &= ~0x04;
//				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x06)			   //��ѹ����У׼���
			{
//				x5 = Vmon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestVH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetVH = (float)y5 - CalPara.TestVH*(float)x5;
//				
//				CalPara.SetVH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetVH = (float)y6 - CalPara.SetVH*(float)x6;

//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ=0;
				Modify_C_READ=0;
				Modify_A_ACT=0;
				Modify_A_READ =Vmon_value;//������ѹֵ
				Modify_C_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionV_MID_HIG1 = var32a;
				var32a=0;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionV_MID_HIG1;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_MID_HIG1 = var32b;
					Polar10 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_MID_HIG1 = var32a;
					Polar10 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
//				var32c = Modify_A_ACT; //CVģʽ��ѹ����У׼
//				var32c = var32c - Modify_B_ACT;
//				var32c = var32c << 12;
//				var16a=Modify_C_READ-Modify_D_READ;
//				var16a=(var16a*2);
//				var32c=var32c/var16a;
//				SET_CorrectionV_MID_HIG = var32c;
//				var32c = Modify_A_ACT;
//				var32c = var32c << 12;
//				var32d = SET_CorrectionV_MID_HIG;
//				var32d = var32d * (Modify_C_READ*2);
//				if (var32c < var32d)
//				{
//					var32d = var32d - var32c;
//					SET_ReadV_Offset_MID_HIG = var32d;
//					Polar4 |= 0x04;
//				}
//				else 
//				{
//					var32c = var32c - var32d;
//					SET_ReadV_Offset_MID_HIG = var32c;
//					Polar4 &= ~0x04;
//				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x07)			   //��ѹ����У׼���
			{
//				x5 = Vmon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestVH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetVH = (float)y5 - CalPara.TestVH*(float)x5;
//				
//				CalPara.SetVH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetVH = (float)y6 - CalPara.SetVH*(float)x6;

//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ=0;
				Modify_C_READ=0;
				Modify_A_ACT=0;
				Modify_A_READ =Vmon_value;//������ѹֵ
				Modify_C_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionV_MID_HIG2 = var32a;
				var32a=0;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionV_MID_HIG2;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_MID_HIG2 = var32b;
					Polar11 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_MID_HIG2 = var32a;
					Polar11 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
//				var32c = Modify_A_ACT; //CVģʽ��ѹ����У׼
//				var32c = var32c - Modify_B_ACT;
//				var32c = var32c << 12;
//				var16a=Modify_C_READ-Modify_D_READ;
//				var16a=(var16a*2);
//				var32c=var32c/var16a;
//				SET_CorrectionV_MID_HIG = var32c;
//				var32c = Modify_A_ACT;
//				var32c = var32c << 12;
//				var32d = SET_CorrectionV_MID_HIG;
//				var32d = var32d * (Modify_C_READ*2);
//				if (var32c < var32d)
//				{
//					var32d = var32d - var32c;
//					SET_ReadV_Offset_MID_HIG = var32d;
//					Polar4 |= 0x04;
//				}
//				else 
//				{
//					var32c = var32c - var32d;
//					SET_ReadV_Offset_MID_HIG = var32c;
//					Polar4 &= ~0x04;
//				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x08)			   //��ѹ����У׼���
			{
//				x5 = Vmon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestVH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetVH = (float)y5 - CalPara.TestVH*(float)x5;
//				
//				CalPara.SetVH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetVH = (float)y6 - CalPara.SetVH*(float)x6;

//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ=0;
				Modify_C_READ=0;
				Modify_A_ACT=0;
				Modify_A_READ =Vmon_value;//������ѹֵ
				Modify_C_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionV_MID_HIG3 = var32a;
				var32a=0;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionV_MID_HIG3;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_MID_HIG3 = var32b;
					Polar12 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_MID_HIG3 = var32a;
					Polar12 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
//				var32c = Modify_A_ACT; //CVģʽ��ѹ����У׼
//				var32c = var32c - Modify_B_ACT;
//				var32c = var32c << 12;
//				var16a=Modify_C_READ-Modify_D_READ;
//				var16a=(var16a*2);
//				var32c=var32c/var16a;
//				SET_CorrectionV_MID_HIG = var32c;
//				var32c = Modify_A_ACT;
//				var32c = var32c << 12;
//				var32d = SET_CorrectionV_MID_HIG;
//				var32d = var32d * (Modify_C_READ*2);
//				if (var32c < var32d)
//				{
//					var32d = var32d - var32c;
//					SET_ReadV_Offset_MID_HIG = var32d;
//					Polar4 |= 0x04;
//				}
//				else 
//				{
//					var32c = var32c - var32d;
//					SET_ReadV_Offset_MID_HIG = var32c;
//					Polar4 &= ~0x04;
//				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			
			
			/*******************��ѹ���Ƶ͵�У׼**************************************/	
			if(UART_Buffer_Rece[2] == 0x09)
			{
				Modify_A_READ = Vmon_value;//������ѹֵ
				Modify_C_READ = Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
			}
			if (UART_Buffer_Rece[2] == 0x0A)			   
			{

//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_LOW = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_LOW;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_LOW = var32d;
					Polar0 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_LOW = var32c;
					Polar0 &= ~0x04;
				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			/*******************��ѹ�����ߵ�λУ׼**************************************/	
			if(UART_Buffer_Rece[2] == 0x0B)
			{
//				x1 = Vmon_value;
//				x3 = Contr_DACVlue;
//				y1 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y1 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y3 = y1;
//				
				Modify_A_READ = Vmon_value;//������ѹֵ
				Modify_C_READ = Contr_DACVlue;//���õ�ѹֵ
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�

			}
			/*******************��ѹ�����ж�У׼*************************/	
			if (UART_Buffer_Rece[2] == 0x0C)			   
			{

				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_HIG = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_HIG;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_HIG = var32d;
					Polar1 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_HIG = var32c;
					Polar1 &= ~0x04;
				}
				Flash_Write_all();	//����д��FLASH
			}
			/*******************��ѹ�����Ϳ��Ƹ߶�У׼*****************/
			if (UART_Buffer_Rece[2] == 0x0D)			   //��ѹ����У׼���
			{
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_MID_HIG = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_MID_HIG;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_MID_HIG = var32d;
					Polar4 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_MID_HIG = var32c;
					Polar4 &= ~0x04;
				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x0E)			   //��ѹ����У׼���
			{
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_MID_HIG1 = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_MID_HIG1;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_MID_HIG1 = var32d;
					Polar10 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_MID_HIG1 = var32c;
					Polar10 &= ~0x04;
				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x0F)			   //��ѹ����У׼���
			{
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_MID_HIG2 = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_MID_HIG2;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_MID_HIG2 = var32d;
					Polar11 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_MID_HIG2 = var32c;
					Polar11 &= ~0x04;
				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x10)			   //��ѹ����У׼���
			{
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				Modify_B_READ =Vmon_value;//������ѹֵ
				Modify_D_READ =Contr_DACVlue;//���õ�ѹֵ
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CVģʽ��ѹ����У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_MID_HIG3 = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_MID_HIG3;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_MID_HIG3 = var32d;
					Polar12 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_MID_HIG3 = var32c;
					Polar12 &= ~0x04;
				}
				Flash_Write_all();	//����д��FLASH
				DAC_Flag=0;
			}
			/******************���������Ϳ��Ƶ͵�У׼********************/	
			if (UART_Buffer_Rece[2] == 0x11)			   //CCģʽ��������У׼
			{
//				x1 = Imon_value;
//				x3 = Contr_DACVlue;
//				y1 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y1 = (y1<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y3 = y1;
				
				Modify_A_READ = Imon_value;//��������
				Modify_C_READ = Contr_DACVlue;//���õ���
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//��ȡ�߶�
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
			}

			if (UART_Buffer_Rece[2] == 0x12)			   //��������У׼���
			{
//				x2 = Imon_value;
//				x4 = Contr_DACVlue;
//				y2 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y2 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y4 = y2;
//				
//				CalPara.TestCL = ((float)y2 - (float)y1)/((float)x2 - (float)x1);
//				CalPara.OffsetCL = (float)y2 - CalPara.TestCL*(float)x2;
//				
//				CalPara.SetCL = ((float)y4 - (float)y3)/((float)x4 - (float)x3);
//				CalPara.OffsetSetCL = (float)y4 - CalPara.SetCL*(float)x4;
				
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_B_READ = Imon_value;
				Modify_D_READ = Contr_DACVlue;
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionA_LOW = var32a;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionA_LOW;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_LOW = var32b;
					Polar2 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_LOW = var32a;
					Polar2 &= ~0x01;					//��������ϵ�������У׼���
				}
				
				var32c = Modify_B_ACT; //���õ���У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_LOW = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_LOW;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_LOW = var32d;
					Polar2 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_LOW = var32c;
					Polar2 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x13)			   //��������У׼���
			{
//				x2 = Imon_value;
//				x4 = Contr_DACVlue;
//				y2 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y2 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y4 = y2;
//				
//				CalPara.TestCL = ((float)y2 - (float)y1)/((float)x2 - (float)x1);
//				CalPara.OffsetCL = (float)y2 - CalPara.TestCL*(float)x2;
//				
//				CalPara.SetCL = ((float)y4 - (float)y3)/((float)x4 - (float)x3);
//				CalPara.OffsetSetCL = (float)y4 - CalPara.SetCL*(float)x4;
				
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_B_READ = Imon_value;
				Modify_D_READ = Contr_DACVlue;
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionA_LOW2 = var32a;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionA_LOW2;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_LOW2 = var32b;
					Polar6 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_LOW2 = var32a;
					Polar6 &= ~0x01;					//��������ϵ�������У׼���
				}
				
				var32c = Modify_B_ACT; //���õ���У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_LOW2 = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_LOW2;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_LOW2 = var32d;
					Polar6 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_LOW2 = var32c;
					Polar6 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x14)			   //��������У׼���
			{
//				x2 = Imon_value;
//				x4 = Contr_DACVlue;
//				y2 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y2 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y4 = y2;
//				
//				CalPara.TestCL = ((float)y2 - (float)y1)/((float)x2 - (float)x1);
//				CalPara.OffsetCL = (float)y2 - CalPara.TestCL*(float)x2;
//				
//				CalPara.SetCL = ((float)y4 - (float)y3)/((float)x4 - (float)x3);
//				CalPara.OffsetSetCL = (float)y4 - CalPara.SetCL*(float)x4;
				
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_B_READ = Imon_value;
				Modify_D_READ = Contr_DACVlue;
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionA_LOW3 = var32a;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionA_LOW3;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_LOW3 = var32b;
					Polar7 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_LOW3 = var32a;
					Polar7 &= ~0x01;					//��������ϵ�������У׼���
				}
				
				var32c = Modify_B_ACT; //���õ���У׼
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_LOW3 = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_LOW3;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_LOW3 = var32d;
					Polar7 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_LOW3 = var32c;
					Polar7 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
	/******************���������Ϳ����ж�У׼*********************/
			if (UART_Buffer_Rece[2] == 0x15)			  
			{
//				x1 = Imon_value;
//				x3 = Contr_DACVlue;
//				y1 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y1 = (y1<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y3 = y1;
				
				Modify_A_READ = Imon_value;
				Modify_C_READ = Contr_DACVlue;
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
			}

			if (UART_Buffer_Rece[2] == 0x16)			  
			{
//				x2 = Imon_value;
//				x4 = Contr_DACVlue;
//				y2 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y2 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y4 = y2;
//				
//				CalPara.TestCM = ((float)y2 - (float)y1)/((float)x2 - (float)x1);
//				CalPara.OffsetCM = (float)y2 - CalPara.TestCM*(float)x2;
//				
//				CalPara.SetCM = ((float)y4 - (float)y3)/((float)x4 - (float)x3);
//				CalPara.OffsetSetCM = (float)y4 - CalPara.SetCM*(float)x4;
				
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_B_READ = Imon_value;
				Modify_D_READ = Contr_DACVlue;
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionA_HIG = var32a;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionA_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_HIG = var32b;
					Polar3 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_HIG = var32a;
					Polar3 &= ~0x01;						
				}
				var32c = Modify_B_ACT; 
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_HIG = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_HIG;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_HIG = var32d;
					Polar3 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_HIG = var32c;
					Polar3 &= ~0x04;
				}
				Flash_Write_all ();	
			}
			
			/****************���������Ϳ��Ƹ߶�У׼*****************/
			if (UART_Buffer_Rece[2] == 0x17)			  
			{
//				x5 = Imon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestCH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetCH = (float)y5 - CalPara.TestCH*(float)x5;
//				
//				CalPara.SetCH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetCH = (float)y6 - CalPara.SetCH*(float)x6;
				
//				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ = Imon_value;
				Modify_C_READ = Contr_DACVlue;
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionA_MID_HIG = var32a;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionA_MID_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_MID_HIG = var32b;
					Polar5 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_MID_HIG = var32a;
					Polar5 &= ~0x01;						
				}
				var32c = Modify_A_ACT; 
				var32c = var32c - Modify_B_ACT;
				var32c = var32c << 12;
				var16a=Modify_C_READ-Modify_D_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_MID_HIG = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_MID_HIG;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_MID_HIG = var32d;
					Polar5 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_MID_HIG = var32c;
					Polar5 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x18)			  
			{
//				x5 = Imon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestCH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetCH = (float)y5 - CalPara.TestCH*(float)x5;
//				
//				CalPara.SetCH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetCH = (float)y6 - CalPara.SetCH*(float)x6;
				
				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ = Imon_value;
				Modify_C_READ = Contr_DACVlue;
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionA_HIG1 = var32a;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionA_HIG1;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_HIG1 = var32b;
					Polar8 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_HIG1 = var32a;
					Polar8 &= ~0x01;						
				}
				var32c = Modify_A_ACT; 
				var32c = var32c - Modify_B_ACT;
				var32c = var32c << 12;
				var16a=Modify_C_READ-Modify_D_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_HIG1 = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_HIG1;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_HIG1 = var32d;
					Polar8 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_HIG1 = var32c;
					Polar8 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
			if (UART_Buffer_Rece[2] == 0x19)			  
			{
//				x5 = Imon_value;
//				x6 = Contr_DACVlue;
//				y5 = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
//				y5 = (Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
//				y6 = y5;
//				
//				CalPara.TestCH = ((float)y5 - (float)y2)/((float)x5 - (float)x2);
//				CalPara.OffsetCH = (float)y5 - CalPara.TestCH*(float)x5;
//				
//				CalPara.SetCH = ((float)y6 - (float)y4)/((float)x6 - (float)x4);
//				CalPara.OffsetSetCH = (float)y6 - CalPara.SetCH*(float)x6;
				
//				
//				vu32 var16;
//				vu32 var32a;
//				vu32 var32b;
//				
//				vu32 var16a;
//				vu32 var32c;
//				vu32 var32d;
				
				Modify_A_READ = Imon_value;
				Modify_C_READ = Contr_DACVlue;
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//��ȡ�Ͷ�
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionA_HIG2 = var32a;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionA_HIG2;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_HIG2 = var32b;
					Polar9 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_HIG2 = var32a;
					Polar9 &= ~0x01;						
				}
				var32c = Modify_A_ACT; 
				var32c = var32c - Modify_B_ACT;
				var32c = var32c << 12;
				var16a=Modify_C_READ-Modify_D_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_HIG2 = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_HIG2;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_HIG2 = var32d;
					Polar9 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_HIG2 = var32c;
					Polar9 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
	/********************����DAֵ********************************/
			if (UART_Buffer_Rece[2] == 0xEE)//���ܵ�ѹDACֵ
			{
				DAC_Flag=0x01;
				Contr_DACVlue = (UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
			}
		}
 }
}
//===============================ADֵת���ɲ���ֵ============================================//
void Transformation_ADC(void)  
{
	vu32 var32;
	vu32 var32a;
/*****************************������ѹת��*******************************************/
	if(V_Gear_SW==0)//��ѹ�͵�λ
	{
		var32 = Vmon_value;
		var32 = var32 * REG_CorrectionV_LOW;  //Y=KX+B �������Y=KX
		if ((Polar0 & 0x01) == 0x01)		  //
		{
			if (var32 < REG_ReadV_Offset_LOW) 
			{
				var32 = 0;
			}
			else var32 = var32 - REG_ReadV_Offset_LOW;
		}
		else var32 = var32 + REG_ReadV_Offset_LOW;
		var32 = var32 >> 12;
		if (var32 < 4) var32 = 0;				  //10mV��������
		Voltage = var32;
		var32 = 0;
		if(I_Gear_SW==0)//�����͵�
		{
			Power_DATE=(Voltage/10)*(Current/100);//����ʵʱ����
			Power_DATE=Power_DATE/1000;//ͳһ��ȷ��С�����3λ
			R_DATE=(Voltage*10000/Current);//����ʵʱ����  ͳһ��ȷ��С�����3λ
		}
		else//�����ߵ�
		{
			Power_DATE=(Voltage/10)*(Current/10);//����ʵʱ����
			Power_DATE=Power_DATE/1000;
			R_DATE=(Voltage*1000/Current);//����ʵʱ����
		}
	}
	else//��ѹ�ߵ�λ
	{
		var32 = Vmon_value;
		var32 = var32 * REG_CorrectionV_HIG;  //Y=KX+B �������Y=KX
		if ((Polar1 & 0x01) == 0x01)		  //
		{
			if (var32 < REG_ReadV_Offset_HIG) 
			{
				var32 = 0;
			}
			else var32 = var32 - REG_ReadV_Offset_HIG;
		}
		else var32 = var32 + REG_ReadV_Offset_HIG;
		var32 = var32 >> 12;
		if (var32 < 4) var32 = 0;				  //10mV��������
		Voltage = var32;
		var32=0;
		if(Voltage>20000)
		{
			var32 = Vmon_value;
			var32 = var32 * REG_CorrectionV_MID_HIG;  //Y=KX+B �������Y=KX
			if ((Polar4 & 0x01) == 0x01)		  //
			{
				if (var32 < REG_ReadV_Offset_MID_HIG) 
				{
					var32 = 0;
				}
				else var32 = var32 - REG_ReadV_Offset_MID_HIG;
			}
			else var32 = var32 + REG_ReadV_Offset_MID_HIG;
			var32 = var32 >> 12;
			if (var32 < 4) var32 = 0;				  //10mV��������
			Voltage = var32;
			var32 = 0;
		}
		if(Voltage>40000)
		{
			var32 = Vmon_value;
//			var32 = var32 * CalPara.TestVH + CalPara.OffsetVH;
			
			var32 = Vmon_value;
			var32 = var32 * REG_CorrectionV_MID_HIG1;  //Y=KX+B �������Y=KX
			if ((Polar10 & 0x01) == 0x01)		  //
			{
				if (var32 < REG_ReadV_Offset_MID_HIG1) 
				{
					var32 = 0;
				}
				else var32 = var32 - REG_ReadV_Offset_MID_HIG1;
			}
			else var32 = var32 + REG_ReadV_Offset_MID_HIG1;
			var32 = var32 >> 12;
//			if (var32 < 3000) var32 = 0;				  //10mV��������
			Voltage = var32;
			var32 = 0;
		}
		
		if(Voltage>60000)
		{
			var32 = Vmon_value;
//			var32 = var32 * CalPara.TestVH + CalPara.OffsetVH;
			
			var32 = Vmon_value;
			var32 = var32 * REG_CorrectionV_MID_HIG2;  //Y=KX+B �������Y=KX
			if ((Polar11 & 0x01) == 0x01)		  //
			{
				if (var32 < REG_ReadV_Offset_MID_HIG2) 
				{
					var32 = 0;
				}
				else var32 = var32 - REG_ReadV_Offset_MID_HIG2;
			}
			else var32 = var32 + REG_ReadV_Offset_MID_HIG2;
			var32 = var32 >> 12;
//			if (var32 < 3000) var32 = 0;				  //10mV��������
			Voltage = var32;
			var32 = 0;
		}
		
		if(Voltage>80000)
		{
			var32 = Vmon_value;
//			var32 = var32 * CalPara.TestVH + CalPara.OffsetVH;
			
			var32 = Vmon_value;
			var32 = var32 * REG_CorrectionV_MID_HIG3;  //Y=KX+B �������Y=KX
			if ((Polar12 & 0x01) == 0x01)		  //
			{
				if (var32 < REG_ReadV_Offset_MID_HIG3) 
				{
					var32 = 0;
				}
				else var32 = var32 - REG_ReadV_Offset_MID_HIG3;
			}
			else var32 = var32 + REG_ReadV_Offset_MID_HIG3;
			var32 = var32 >> 12;
//			if (var32 < 3000) var32 = 0;				  //10mV��������
			Voltage = var32;		
			var32 = 0;
		}
		if(I_Gear_SW==0)
		{
			Power_DATE=Voltage*(Current/10/*0*/);//����ʵʱ����
			Power_DATE=Power_DATE/1000;
			R_DATE=(Voltage*100000/Current);//����ʵʱ����
		}
		else
		{
			Power_DATE=Voltage*(Current/1/*0*/);//����ʵʱ����
			Power_DATE=Power_DATE/1000;
			R_DATE=(Voltage*10000/Current);//����ʵʱ����
		}
	}
/*****************************�Ʋ���ѹת��*******************************************/
	if(V_Gear_SW==0)//��ѹ�͵�λ
	{
		var32 = AD7655_CHB_VMON;
		var32 = var32 * REG_CorrectionV_LOW;  //Y=KX+B �������Y=KX
		if ((Polar0 & 0x01) == 0x01)		  //
		{
			if (var32 < REG_ReadV_Offset_LOW) 
			{
				var32 = 0;
			}
			else var32 = var32 - REG_ReadV_Offset_LOW;
		}
		else var32 = var32 + REG_ReadV_Offset_LOW;
		var32 = var32 >> 12;
		if (var32 < 4) var32 = 0;				  //10mV��������
		Voltage_VPP = var32;
		var32 = 0;
	}
	else//��ѹ�ߵ�λ
	{
		var32 = AD7655_CHB_VMON;
		var32 = var32 * REG_CorrectionV_HIG;  //Y=KX+B �������Y=KX
		if ((Polar1 & 0x01) == 0x01)		  //
		{
			if (var32 < REG_ReadV_Offset_HIG) 
			{
				var32 = 0;
			}
			else var32 = var32 - REG_ReadV_Offset_HIG;
		}
		else var32 = var32 + REG_ReadV_Offset_HIG;
		var32 = var32 >>12;
		if (var32 < 4) var32 = 0;				  //10mV��������
		Voltage_VPP = var32;
		var32 = 0;
	}	
	
/*****************************��������ת��*******************************************/
	if(I_Gear_SW==0)//�͵�λ
	{
		var32 = Imon_value;
		var32 = var32 * REG_CorrectionA_LOW;	   
		if ((Polar2 & 0x01) == 0x01)			   
		{
			if (var32 < REG_ReadA_Offset_LOW) var32 = 0;
			else var32 = var32 - REG_ReadA_Offset_LOW;
		}
		else
		{
			var32 = var32 + REG_ReadA_Offset_LOW;
		}	
		var32 = var32 >> 12;
		Current = var32;
		if(Current > ILOW1)
		{
			var32 = Imon_value;
			var32 = var32 * REG_CorrectionA_LOW2;	   
			if ((Polar6 & 0x01) == 0x01)			   
			{
				if (var32 < REG_ReadA_Offset_LOW2) var32 = 0;
				else var32 = var32 - REG_ReadA_Offset_LOW2;
			}
			else
			{
				var32 = var32 + REG_ReadA_Offset_LOW2;
			}	
			var32 = var32 >> 12;
			Current = var32;
		}
		if(Current > ILOW2)
		{
			var32 = Imon_value;
			var32 = var32 * REG_CorrectionA_LOW3;	   
			if ((Polar7 & 0x01) == 0x01)			   
			{
				if (var32 < REG_ReadA_Offset_LOW3) var32 = 0;
				else var32 = var32 - REG_ReadA_Offset_LOW3;
			}
			else
			{
				var32 = var32 + REG_ReadA_Offset_LOW3;
			}	
			var32 = var32 >> 12;
			Current = var32;
		}
		var32 = 0;
		if((Current<=1)||(onoff_ch==0)||(Voltage==0))
		{
			Current = 0;
		}
	}
	else//�ߵ�λ
	{
		var32 = Imon_value;
		var32 = var32 * REG_CorrectionA_HIG;	   
		if ((Polar3 & 0x01) == 0x01)			   
		{
			if (var32 < REG_ReadA_Offset_HIG) var32 = 0;
			else var32 = var32 - REG_ReadA_Offset_HIG;
		}
		else
		{
			var32 = var32 + REG_ReadA_Offset_HIG;
		}	
		var32 = var32 >> 12;
		Current = var32;
		if(Current>IHIGH2)
		{
			var32=0;
			var32 = Imon_value;
//			var32 = var32 * CalPara.TestVH + CalPara.OffsetVH;
			
			var32 = Imon_value;
			var32 = var32 * REG_CorrectionA_MID_HIG;	   
			if ((Polar5 & 0x01) == 0x01)			   
			{
				if (var32 < REG_ReadA_Offset_MID_HIG) var32 = 0;
				else var32 = var32 - REG_ReadA_Offset_MID_HIG;
			}
			else
			{
				var32 = var32 + REG_ReadA_Offset_MID_HIG;
			}	
			var32 = var32 >> 12;
			Current = var32;
		}
		if(Current>IHIGH3)
		{
			var32=0;
			var32 = Imon_value;
//			var32 = var32 * CalPara.TestVH + CalPara.OffsetVH;
			
			var32 = Imon_value;
			var32 = var32 * REG_CorrectionA_HIG1;	   
			if ((Polar8 & 0x01) == 0x01)			   
			{
				if (var32 < REG_ReadA_Offset_HIG1) var32 = 0;
				else var32 = var32 - REG_ReadA_Offset_HIG1;
			}
			else
			{
				var32 = var32 + REG_ReadA_Offset_HIG1;
			}	
			var32 = var32 >> 12;
			Current = var32;
		}
		if(Current>IHIGH4)
		{
			var32=0;
			var32 = Imon_value;
//			var32 = var32 * CalPara.TestVH + CalPara.OffsetVH;
			
			var32 = Imon_value;
			var32 = var32 * REG_CorrectionA_HIG2;	   
			if ((Polar9 & 0x01) == 0x01)			   
			{
				if (var32 < REG_ReadA_Offset_HIG2) var32 = 0;
				else var32 = var32 - REG_ReadA_Offset_HIG2;
			}
			else
			{
				var32 = var32 + REG_ReadA_Offset_HIG2;
			}	
			var32 = var32 >> 12;
			Current = var32;
		}
		if(onoff_ch==0||(Voltage==0))
		{
			Current=0;
		}
		var32 = 0;

	}
/****************************�����Ʋ�ת��*****************************************/
	if(I_Gear_SW==0)//�͵�λ
	{
		var32 = AD7655_CHA_IMON;
		var32 = var32 * REG_CorrectionA_LOW;	   
		if ((Polar2 & 0x01) == 0x01)			   
		{
			if (var32 < REG_ReadA_Offset_LOW) var32 = 0;
			else var32 = var32 - REG_ReadA_Offset_LOW;
		}
		else
		{
			var32 = var32 + REG_ReadA_Offset_LOW;
		}	
		var32 = var32 >> 12;
		Current_VPP = var32;
		var32 = 0;
	}
	else//�ߵ�λ
	{
		var32 = AD7655_CHA_IMON;
		var32 = var32 * REG_CorrectionA_HIG;	   
		if ((Polar3 & 0x01) == 0x01)			   
		{
			if (var32 < REG_ReadA_Offset_HIG) var32 = 0;
			else var32 = var32 - REG_ReadA_Offset_HIG;
		}
		else
		{
			var32 = var32 + REG_ReadA_Offset_HIG;
		}	
		var32 = var32 >> 12;
		Current_VPP = var32;
		var32 = 0;

	}
	
/**************************���õ�ѹת��******************************************/
	if(MODE==1)//CVģʽ
	{
		if(SET_V_TRAN!=SET_Voltage)
		{
			TIME_1MS_OVER=0;
		}
		if(TIME_1MS_flag==1)//CVģʽ��ѹ�½��ʵ���
		{
			TIME_1MS_flag=0;
			if(V_Gear_SW==0)//�͵�λ
			{
				
				if(CV_Down_Time >= SET_V_TRAN)
				{
					SET_V_TRAN=SET_Voltage;
					TIME_1MS_OVER=1;//����������־
				}else{
					SET_V_TRAN=SET_V_TRAN-CV_Down_Time;
					if(SET_V_TRAN<=SET_Voltage)
					{
						SET_V_TRAN=SET_Voltage;
						TIME_1MS_OVER=1;//����������־
					}
				}
			}else if(V_Gear_SW==1){//�ߵ�λ
				
				if(CV_Down_Time/10 >= SET_V_TRAN)
				{
					SET_V_TRAN=SET_Voltage/10;
					TIME_1MS_OVER=1;//����������־
				}else{
					SET_V_TRAN=SET_V_TRAN-CV_Down_Time/10;
					if(SET_V_TRAN<=SET_Voltage/10)
					{
						SET_V_TRAN=SET_Voltage/10;
						TIME_1MS_OVER=1;//����������־
					}
				}
			}
		} 
		if(V_Gear_SW==0)//�͵�λ
		{ 
			var32 = SET_V_TRAN;
//			var32 = var32 * CalPara.SetVL + CalPara.OffsetSetVL;
			
			var32=var32<<12; 
			if ((Polar0 & 0x04) == 0)			   
			{
				if (var32 < SET_ReadV_Offset_LOW) var32 = 0;
				else var32 = var32 - SET_ReadV_Offset_LOW;
			}
			else var32 = var32 + SET_ReadV_Offset_LOW;
			var32 = var32/SET_CorrectionV_LOW;
			var32=var32>>1;
			Contr_DACVlue = var32;
			var32 = 0;
		}
		else//�ߵ�λ
		{ 
			var32 = SET_V_TRAN;
			
			
			var32=var32<<12; 
			if(SET_Voltage<200000)
			{
//				var32 = var32 * CalPara.SetVM + CalPara.OffsetSetVM;
				if ((Polar1 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_HIG;
				}
				else var32 = var32 + SET_ReadV_Offset_HIG;
				var32 = var32/SET_CorrectionV_HIG;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}else if(SET_Voltage>=200000 && SET_Voltage<400000){
//				var32 = var32 * CalPara.SetVH + CalPara.OffsetSetVH;
				if ((Polar4 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_MID_HIG) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_MID_HIG;
				}
				else var32 = var32 + SET_ReadV_Offset_MID_HIG;
				var32 = var32/SET_CorrectionV_MID_HIG;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}else if(SET_Voltage>=400000 && SET_Voltage<600000){
//				var32 = var32 * CalPara.SetVH + CalPara.OffsetSetVH;
				if ((Polar10 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_MID_HIG1) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_MID_HIG1;
				}
				else var32 = var32 + SET_ReadV_Offset_MID_HIG1;
				var32 = var32/SET_CorrectionV_MID_HIG1;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}else if(SET_Voltage>=600000 && SET_Voltage<800000){
//				var32 = var32 * CalPara.SetVH + CalPara.OffsetSetVH;
				if ((Polar11 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_MID_HIG2) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_MID_HIG2;
				}
				else var32 = var32 + SET_ReadV_Offset_MID_HIG2;
				var32 = var32/SET_CorrectionV_MID_HIG2;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}else if(SET_Voltage>=800000){
//				var32 = var32 * CalPara.SetVH + CalPara.OffsetSetVH;
				if ((Polar12 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_MID_HIG3) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_MID_HIG3;
				}
				else var32 = var32 + SET_ReadV_Offset_MID_HIG3;
				var32 = var32/SET_CorrectionV_MID_HIG3;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}
		}
	}
/**************************���õ���ת��**************************************/
	else//CCģʽ��CR,CP��CCģʽģ�����
	{
		if(I_Gear_SW==0)//�͵�
		{
			if(MODE==0)//CCģʽ
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_Current)
					{
						SET_I_TRAN=SET_Current;
					}
				}
				var32 = SET_I_TRAN;
//				var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
				if(SET_Current < ILOW1)
				{
					var32=var32<<12;   
					if ((Polar2 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW;
					var32 = var32/SET_CorrectionA_LOW;
					var32=var32>>1;
				}else if(SET_Current >= ILOW1 && SET_Current < ILOW2){
					var32=var32<<12;   
					if ((Polar6 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW2) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW2;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW2;
					var32 = var32/SET_CorrectionA_LOW2;
					var32=var32>>1;
				}else if(SET_Current >= ILOW2 && SET_Current <= ILOW3){
					var32=var32<<12;   
					if ((Polar7 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW3) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW3;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW3;
					var32 = var32/SET_CorrectionA_LOW3;
					var32=var32>>1;
				}
			}
			else if(MODE==2)//CRģʽ
			{
				if(V_Gear_SW==0)//��ѹ�͵�ģʽʱ
				{
					SET_R_Current=(Voltage*10)/SET_Resist;//�����͵�ģʽCRģʽ��ȷ��0.001
				}
				else//��ѹ�ߵ�ģʽʱ
				{
					SET_R_Current=(Voltage*100)/SET_Resist;//�����͵�ģʽCRģʽ��ȷ��0.001
				}
//				var32 = SET_R_Current;
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_R_Current)
					{
						SET_I_TRAN=SET_R_Current;
					}
				}
				var32 = SET_I_TRAN;
//				var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
				if(SET_R_Current < ILOW1)
				{
					var32=var32<<12;   
					if ((Polar2 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW;
					var32 = var32/SET_CorrectionA_LOW;
					var32=var32>>1;
				}else if(SET_R_Current >= ILOW1 && SET_R_Current < ILOW2){
					var32=var32<<12;   
					if ((Polar6 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW2) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW2;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW2;
					var32 = var32/SET_CorrectionA_LOW2;
					var32=var32>>1;
				}else if(SET_R_Current >= ILOW2 && SET_R_Current <= ILOW3){
					var32=var32<<12;   
					if ((Polar7 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW3) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW3;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW3;
					var32 = var32/SET_CorrectionA_LOW3;
					var32=var32>>1;
				}
			}
			else if(MODE==3)//CPģʽ
			{
				if(V_Gear_SW==0)
				{
//					SET_P_Current=(SET_Power*100000)/(Voltage);//CPģʽ�����ߵ���С�ֱ���Ϊ0.001��
					SET_P_Current=(vu32)((((float)(SET_Power)/1000)/((float)Voltage/10000))*10000);
				}
				else
				{
//					SET_P_Current=(SET_Power*100000)/(Voltage*10);//CPģʽ�����ߵ���С�ֱ���Ϊ0.001��
					SET_P_Current=(vu32)((((float)(SET_Power)/1000)/((float)Voltage/1000))*10000);
				}
				var32 = SET_P_Current;
//				var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
				if(SET_P_Current < ILOW1)
				{
					var32=var32<<12;   
					if ((Polar2 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW;
					var32 = var32/SET_CorrectionA_LOW;
					var32=var32>>1;
				}else if(SET_P_Current >= ILOW1 && SET_P_Current < ILOW2){
					var32=var32<<12;   
					if ((Polar6 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW2) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW2;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW2;
					var32 = var32/SET_CorrectionA_LOW2;
					var32=var32>>1;
				}else if(SET_P_Current >= ILOW2 && SET_P_Current <= ILOW3){
					var32=var32<<12;   
					if ((Polar7 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_LOW3) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_LOW3;
					}
					else var32 = var32 + SET_ReadA_Offset_LOW3;
					var32 = var32/SET_CorrectionA_LOW3;
					var32=var32>>1;
				}
			}else if(MODE==4)//��̬ģʽ
			{
				if(DYNA_MODE == 0 || DYNA_MODE == 1 || DYNA_MODE == 2)//����ģʽ��������ģʽ
				{

					if(dynaflagA == 1)
					{
						if(TIME_1MS_flag==1 && SET_I_TRAN < DYNA_Ia)
						{
							SET_I_TRAN=SET_I_TRAN+DYNA_IRise;
							if(SET_I_TRAN>=DYNA_Ia)
							{
								SET_I_TRAN=DYNA_Ia;
							}
						}else if(TIME_1MS_flag==1 && SET_I_TRAN > DYNA_Ia){
							if(DYNA_IDown >= SET_I_TRAN)
							{
								SET_I_TRAN = DYNA_Ia;
							}else{
								SET_I_TRAN=SET_I_TRAN-DYNA_IDown;
								if(SET_I_TRAN<=DYNA_Ia)
								{
									SET_I_TRAN=DYNA_Ia;
								}
							}
						}
						var32 = SET_I_TRAN;
//						var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
//						var32=var32<<12;   
						if(SET_I_TRAN < ILOW1)
						{
							var32=var32<<12;   
							if ((Polar2 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_LOW;
							}
							else var32 = var32 + SET_ReadA_Offset_LOW;
							var32 = var32/SET_CorrectionA_LOW;
							var32=var32>>1;
						}else if(SET_I_TRAN >= ILOW1 && SET_I_TRAN < ILOW2){
							var32=var32<<12;   
							if ((Polar6 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_LOW2) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_LOW2;
							}
							else var32 = var32 + SET_ReadA_Offset_LOW2;
							var32 = var32/SET_CorrectionA_LOW2;
							var32=var32>>1;
						}else if(SET_I_TRAN >= ILOW2 && SET_I_TRAN <= ILOW3){
							var32=var32<<12;   
							if ((Polar7 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_LOW3) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_LOW3;
							}
							else var32 = var32 + SET_ReadA_Offset_LOW3;
							var32 = var32/SET_CorrectionA_LOW3;
							var32=var32>>1;
						}
					}else if(dynaflagB == 1){
						if(TIME_1MS_flag==1 && SET_I_TRAN < DYNA_Ib)
						{
							SET_I_TRAN=SET_I_TRAN+DYNA_IRise;
							if(SET_I_TRAN>=DYNA_Ib)
							{
								SET_I_TRAN=DYNA_Ib;
							}
						}else if(TIME_1MS_flag==1 && SET_I_TRAN > DYNA_Ib){
							if(DYNA_IDown >= SET_I_TRAN)
							{
								SET_I_TRAN = DYNA_Ib;
							}else{
								SET_I_TRAN=SET_I_TRAN-DYNA_IDown;
								if(SET_I_TRAN<=DYNA_Ib)
								{
									SET_I_TRAN=DYNA_Ib;
								}
							}
							
						}
						var32 = SET_I_TRAN;
//						var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
						if(SET_I_TRAN < ILOW1)
						{
							var32=var32<<12;   
							if ((Polar2 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_LOW;
							}
							else var32 = var32 + SET_ReadA_Offset_LOW;
							var32 = var32/SET_CorrectionA_LOW;
							var32=var32>>1;
						}else if(SET_I_TRAN >= ILOW1 && SET_I_TRAN < ILOW2){
							var32=var32<<12;   
							if ((Polar6 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_LOW2) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_LOW2;
							}
							else var32 = var32 + SET_ReadA_Offset_LOW2;
							var32 = var32/SET_CorrectionA_LOW2;
							var32=var32>>1;
						}else if(SET_I_TRAN >= ILOW2 && SET_I_TRAN <= ILOW3){
							var32=var32<<12;   
							if ((Polar7 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_LOW3) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_LOW3;
							}
							else var32 = var32 + SET_ReadA_Offset_LOW3;
							var32 = var32/SET_CorrectionA_LOW3;
							var32=var32>>1;
						}
					}
				}
				
			}
			else if(MODE==5)//LEDģʽ ʵ�ʼ���ΪCRģʽ  LEDģʽ��ѹ��λĬ���л�Ϊ�ߵ�λ
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_R_Current)
					{
						SET_I_TRAN=SET_R_Current;
					}
				}
				var32 = SET_I_TRAN;
//				var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			else if(MODE==6)//��·ģʽ
			{
				var32 = SET_S_Current;
//				var32 = var32 * CalPara.SetCL + CalPara.OffsetSetCL;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			Contr_DACVlue = var32;
			var32 = 0;
		}
		else//�ߵ�
		{
			if(MODE==0)//CCģʽ
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time/10;
					if(SET_I_TRAN>=SET_Current/10)
					{
						SET_I_TRAN=SET_Current/10;
					}
				}
				var32 = SET_I_TRAN;
				if(SET_Current<IHIGH2)//�ֶο��ƣ���һ��
				{
//					var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
					var32=var32<<12;   
					if ((Polar3 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG;
					var32 = var32/SET_CorrectionA_HIG;
					var32=var32>>1;
					if(SET_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_Current>=IHIGH2 && SET_Current<IHIGH3){//�ڶ���
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar5 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_MID_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_MID_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_MID_HIG;
					var32 = var32/SET_CorrectionA_MID_HIG;
					var32=var32>>1;
					if(SET_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_Current>=IHIGH3/* && SET_Current<IHIGH4*/){//������
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar8 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG1) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG1;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG1;
					var32 = var32/SET_CorrectionA_HIG1;
					var32=var32>>1;
					if(SET_Current==0)
					{
						Contr_DACVlue=0;
					}
				}
				else if(SET_Current>=IHIGH4){//���Ķ�
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar9 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG2) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG2;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG2;
					var32 = var32/SET_ReadA_Offset_HIG2;
					var32=var32>>1;
					if(SET_Current==0)
					{
						Contr_DACVlue=0;
					}
				}
			}
			else if(MODE==2)//CRģʽ
			{
				if(V_Gear_SW==0)
				{
					SET_R_Current=(Voltage*1)/SET_Resist;//CRģʽ�����ߵ���С�ֱ���Ϊ0.01ŷ
				}
				else
				{
					SET_R_Current=(Voltage*10)/SET_Resist;//CRģʽ�����ߵ���С�ֱ���Ϊ0.01ŷ
				}
				var32 = SET_R_Current;
//				var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
				if(SET_R_Current<IHIGH2)//�ֶο��ƣ���һ��
				{
//					var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
					var32=var32<<12;   
					if ((Polar3 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG;
					var32 = var32/SET_CorrectionA_HIG;
					var32=var32>>1;
					if(SET_R_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_R_Current>=IHIGH2 && SET_R_Current<IHIGH3){//�ڶ���
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar5 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_MID_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_MID_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_MID_HIG;
					var32 = var32/SET_CorrectionA_MID_HIG;
					var32=var32>>1;
					if(SET_R_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_R_Current>=IHIGH3 && SET_R_Current<IHIGH4){//������
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar8 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG1) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG1;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG1;
					var32 = var32/SET_CorrectionA_HIG1;
					var32=var32>>1;
					if(SET_R_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_R_Current>=IHIGH4){//���Ķ�
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar9 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG2) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG2;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG2;
					var32 = var32/SET_ReadA_Offset_HIG2;
					var32=var32>>1;
					if(SET_R_Current==0)
					{
						Contr_DACVlue=0;
					}
				}
			}
			else if(MODE==3)//CPģʽ
			{
				if(V_Gear_SW==0)
				{
					SET_P_Current=(SET_Power*1000)/(Voltage/10);//CPģʽ�����ߵ���С�ֱ���Ϊ0.01��
				}
				else
				{
					SET_P_Current=(SET_Power*1000)/Voltage;//CPģʽ�����ߵ���С�ֱ���Ϊ0.01��
				}
				var32 = SET_P_Current;
//				var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
				if(SET_P_Current<IHIGH2)//�ֶο��ƣ���һ��
				{
//					var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
					var32=var32<<12;   
					if ((Polar3 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG;
					var32 = var32/SET_CorrectionA_HIG;
					var32=var32>>1;
					if(SET_P_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_P_Current>=IHIGH2 && SET_P_Current<IHIGH3){//�ڶ���
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar5 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_MID_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_MID_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_MID_HIG;
					var32 = var32/SET_CorrectionA_MID_HIG;
					var32=var32>>1;
					if(SET_P_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_P_Current>=IHIGH3 && SET_P_Current<IHIGH4){//������
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar8 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG1) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG1;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG1;
					var32 = var32/SET_CorrectionA_HIG1;
					var32=var32>>1;
					if(SET_P_Current==0)
					{
						Contr_DACVlue=0;
					}
				}else if(SET_P_Current>=IHIGH4){//���Ķ�
//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
					var32=var32<<12;   
					if ((Polar9 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG2) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG2;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG2;
					var32 = var32/SET_ReadA_Offset_HIG2;
					var32=var32>>1;
					if(SET_P_Current==0)
					{
						Contr_DACVlue=0;
					}
				}
			}else if(MODE==4)//��̬ģʽ
			{
				if(DYNA_MODE == 0 || DYNA_MODE == 1 || DYNA_MODE == 2)//����ģʽ
				{

					if(dynaflagA == 1)
					{
						if(TIME_1MS_flag==1 && SET_I_TRAN < DYNA_Ia/10)
						{
							SET_I_TRAN=SET_I_TRAN+DYNA_IRise/10;
							if(SET_I_TRAN>=DYNA_Ia/10)
							{
								SET_I_TRAN=DYNA_Ia/10;
							}
						}else if(TIME_1MS_flag==1 && SET_I_TRAN > DYNA_Ia/10){
							if(DYNA_IDown/10 >= SET_I_TRAN)
							{
								SET_I_TRAN = DYNA_Ia/10;
							}else{
								SET_I_TRAN=SET_I_TRAN-DYNA_IDown/10;
								if(SET_I_TRAN<=DYNA_Ia/10)
								{
									SET_I_TRAN=DYNA_Ia/10;
								}
							}
							
						}
						var32 = SET_I_TRAN;
//						var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
						if(SET_I_TRAN<IHIGH2)//�ֶο��ƣ���һ��
						{
		//					var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
							var32=var32<<12;   
							if ((Polar3 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_HIG;
							}
							else var32 = var32 + SET_ReadA_Offset_HIG;
							var32 = var32/SET_CorrectionA_HIG;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}else if(SET_I_TRAN>=IHIGH2 && SET_I_TRAN<IHIGH3){//�ڶ���
		//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
							var32=var32<<12;   
							if ((Polar5 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_MID_HIG) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_MID_HIG;
							}
							else var32 = var32 + SET_ReadA_Offset_MID_HIG;
							var32 = var32/SET_CorrectionA_MID_HIG;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}else if(SET_I_TRAN>=IHIGH3 && SET_I_TRAN<IHIGH4){//������
		//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
							var32=var32<<12;   
							if ((Polar8 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_HIG1) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_HIG1;
							}
							else var32 = var32 + SET_ReadA_Offset_HIG1;
							var32 = var32/SET_CorrectionA_HIG1;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}else if(SET_I_TRAN>=IHIGH4){//���Ķ�
		//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
							var32=var32<<12;   
							if ((Polar9 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_HIG2) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_HIG2;
							}
							else var32 = var32 + SET_ReadA_Offset_HIG2;
							var32 = var32/SET_ReadA_Offset_HIG2;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}
					}else if(dynaflagB == 1){
						if(TIME_1MS_flag==1 && SET_I_TRAN < DYNA_Ib/10)
						{
							SET_I_TRAN=SET_I_TRAN+DYNA_IRise/10;
							if(SET_I_TRAN>=DYNA_Ib/10)
							{
								SET_I_TRAN=DYNA_Ib/10;
							}
						}else if(TIME_1MS_flag==1 && SET_I_TRAN > DYNA_Ib/10){
							if(DYNA_IDown/10 >= SET_I_TRAN)
							{
								SET_I_TRAN = DYNA_Ib/10;
							}else{
								SET_I_TRAN=SET_I_TRAN-DYNA_IDown/10;
								if(SET_I_TRAN<=DYNA_Ib/10)
								{
									SET_I_TRAN=DYNA_Ib/10;
								}
							}
							
						}
						var32 = SET_I_TRAN;
//						var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
						if(SET_I_TRAN<IHIGH2)//�ֶο��ƣ���һ��
						{
		//					var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
							var32=var32<<12;   
							if ((Polar3 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_HIG;
							}
							else var32 = var32 + SET_ReadA_Offset_HIG;
							var32 = var32/SET_CorrectionA_HIG;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}else if(SET_I_TRAN>=IHIGH2 && SET_I_TRAN<IHIGH3){//�ڶ���
		//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
							var32=var32<<12;   
							if ((Polar5 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_MID_HIG) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_MID_HIG;
							}
							else var32 = var32 + SET_ReadA_Offset_MID_HIG;
							var32 = var32/SET_CorrectionA_MID_HIG;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}else if(SET_I_TRAN>=IHIGH3 && SET_I_TRAN<IHIGH4){//������
		//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
							var32=var32<<12;   
							if ((Polar8 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_HIG1) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_HIG1;
							}
							else var32 = var32 + SET_ReadA_Offset_HIG1;
							var32 = var32/SET_CorrectionA_HIG1;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}else if(SET_I_TRAN>=IHIGH4){//���Ķ�
		//					var32 = var32 * CalPara.SetCH + CalPara.OffsetSetCH;
							var32=var32<<12;   
							if ((Polar9 & 0x04) == 0)			   
							{
								if (var32 < SET_ReadA_Offset_HIG2) var32 = 0;
								else var32 = var32 - SET_ReadA_Offset_HIG2;
							}
							else var32 = var32 + SET_ReadA_Offset_HIG2;
							var32 = var32/SET_ReadA_Offset_HIG2;
							var32=var32>>1;
							if(SET_I_TRAN==0)
							{
								Contr_DACVlue=0;
							}
						}
					}
				}
				
			}
			else if(MODE==5)//LEDģʽ ʵ�ʼ���ΪCRģʽ  LEDģʽ��ѹ��λĬ���л�Ϊ�ߵ�λ
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_R_Current)
					{
						SET_I_TRAN=SET_R_Current;
					}
				}
				var32 = SET_I_TRAN;
//				var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
				var32=var32<<12;   
				if ((Polar3 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_HIG;
				}
				else var32 = var32 + SET_ReadA_Offset_HIG;
				var32 = var32/SET_CorrectionA_HIG;
				var32=var32>>1;
			}
			else if(MODE==6)//��·ģʽ
			{
				var32 = SET_S_Current;
//				var32 = var32 * CalPara.SetCM + CalPara.OffsetSetCM;
				var32=var32<<12;   
				if ((Polar3 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_HIG;
				}
				else var32 = var32 + SET_ReadA_Offset_HIG;
				var32 = var32/SET_CorrectionA_HIG;
				var32=var32>>1;
			}
			Contr_DACVlue = var32;
			var32=0;
//			if(SET_Current==0)
//			{
//				Contr_DACVlue=0;
//			}
		}
	}
}
//-----------------------------CRC���--------------------------------------------//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count)    //CRC16
{
	vu16 CRC_Result=0xffff;
	vu8 i;
	if(count==0)
	{
		count=1;
	}
	while(count--)
	{
		CRC_Result^=*p_buffer;
		for(i=0;i<8;i++)
		{
			if(CRC_Result&1)
			{
				CRC_Result>>=1;
				CRC_Result^=0xA001;
			}
			else 
			{
				CRC_Result>>=1;
			}
		}
		p_buffer++;
	}
	return CRC_Result;
}
