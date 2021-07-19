/**
  ******************************************************************************
  * @文	件 ： SCA_APP.h
  * @作	者 ： INNFOS Software Team
  * @版	本 ： V1.5.1
  * @日	期 ： 2019.09.10
  * @摘	要 ： SCA 测试程序
  ******************************************************************************/ 
  
#ifndef __SCA_APP_H
#define __SCA_APP_H
#include "sys.h"
#include "SCA_API.h"

#define	DEMO	0	//是否只是电机调试

#define SCA_ID	0x06

extern SCA_Handler_t* pSCA_ID1;
extern uint8_t cmd;					//外部控制命令

void SCA_Init(void);	//控制器初始化
void SCA_Set(void);		//电机参数设置（自己写的）
void SCA_Homing(void);	//执行器归零
#if DEMO
void SCA_Exp1(void);  	//正反转Demo
void SCA_Exp2(void);	//高低速Demo
void SCA_Lookup(void);	//查找存在的ID
void homePositionReset(void);
void resetAll(void);  
  
void Log(void);
void CMD_Usart(void);
void CMD_Handler(uint8_t cmd);
#endif

#endif


