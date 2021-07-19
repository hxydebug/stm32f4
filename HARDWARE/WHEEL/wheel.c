#include "wheel.h"
#include "usart.h"
#include "string.h"
#include "delay.h"
#include "dac.h"
#include "encoder.h"
//////////////////////////////////////////////////////////////////////////////////	 
//ALIENTEK STM32F429开发板
//通过模拟电压，控制轮毂电机驱动器  
//修改日期:2020/9/7
//版本：V1.0
//
//********************************************************************************
WHEEL wheel;

void wheelInit(void){
//	wheel.target_speed = 0;
//	wheel.encoder_count = 0;
	wheel.isDriverEnable = 1;//使能
	wheel.driverVol = 1000;
	
	ENPort_Init();
	DAC1_Init();
	Encoder_Init();
	
	setDriver();
}

void wheelSet(u16 driverVol){
	if(driverVol){
		wheel.isDriverEnable = 1;
		wheel.driverVol = driverVol;
	}else{
		wheel.isDriverEnable = 0;
	}
	setDriver();
}

void setDriver(void){
	if(wheel.isDriverEnable){
		DriverEnable;
		DAC1_Set_Vol(wheel.driverVol);
	}else{
		DriverDisable;
		wheel.driverVol = 0;
		DAC1_Set_Vol(wheel.driverVol);
	}
}

//初始化PB7为输出，作为使能口    
void ENPort_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_7; //PB7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
    DriverDisable;	//驱动器失能  
}
