/******************** (C) COPYRIGHT 2015 AVER************************
 * �ļ���  ��
 * ����    ������
 * ����    ��ͷ�ļ���ֻ�����������ɶ���
 * ����    ��
 * �޸����ڣ�2015-8-18
*********************************************************************/
#include "my_register.h"
#ifndef _flash_h_
#define _flash_h_
/*********************��������*************************************/
void Init_EEPROM(void);
void EEPROM_Write_8(vu8 addr, vu8 data);
vu8 EEPROM_READ_Byte(vu8 addr_eep);
void Flash_Write_all (void);
void EEPROM_READ_Coeff(void);
void Wite_Runcont(void);//�����в���д��EEPROM
void Read_Runcont(void);//��ȡ���в���
void READ_TSET(void);//
/******************************************************************/
#endif
/******************* (C) COPYRIGHT 2015 AVER *****END OF FILE***************************/
