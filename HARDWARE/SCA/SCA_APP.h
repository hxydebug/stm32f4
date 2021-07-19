/**
  ******************************************************************************
  * @��	�� �� SCA_APP.h
  * @��	�� �� INNFOS Software Team
  * @��	�� �� V1.5.1
  * @��	�� �� 2019.09.10
  * @ժ	Ҫ �� SCA ���Գ���
  ******************************************************************************/ 
  
#ifndef __SCA_APP_H
#define __SCA_APP_H
#include "sys.h"
#include "SCA_API.h"

#define	DEMO	0	//�Ƿ�ֻ�ǵ������

#define SCA_ID	0x06

extern SCA_Handler_t* pSCA_ID1;
extern uint8_t cmd;					//�ⲿ��������

void SCA_Init(void);	//��������ʼ��
void SCA_Set(void);		//����������ã��Լ�д�ģ�
void SCA_Homing(void);	//ִ��������
#if DEMO
void SCA_Exp1(void);  	//����תDemo
void SCA_Exp2(void);	//�ߵ���Demo
void SCA_Lookup(void);	//���Ҵ��ڵ�ID
void homePositionReset(void);
void resetAll(void);  
  
void Log(void);
void CMD_Usart(void);
void CMD_Handler(uint8_t cmd);
#endif

#endif


