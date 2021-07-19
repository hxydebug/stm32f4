#ifndef _WHEEL_H
#define _WHEEL_H
#include "sys.h"
#include "balance.h"

#define	DriverEnable	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)
#define DriverDisable	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)

typedef struct{
	int target_speed;
	u32 encoder_count;
	u8 isDriverEnable;
	u16 driverVol;
	//»¹ÓÐÉ²³µ
}WHEEL;

extern WHEEL wheel;

void wheelInit(void);
void wheelSet(u16 driverVol);
void setDriver(void);
void ENPort_Init(void);

#endif
