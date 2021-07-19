#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "timer.h"
#include "JY901.h"
#include "wheel.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "SCA_Protocol.h"
#include "can.h"
#include "balance.h"
#include "leg.h"
#include "Nuci7.h"
/************************************************
 ALIENTEK 阿波罗STM32F429开发板	自行车平衡实验
 作者：王增豪
************************************************/

/*备注：
 *对IMU roll angle而言，向右倾斜为正方向
 *编码器白线接PD12，黑线接PD13
 *转向电机右转为正，与计算所需的坐标系相反
 *
 */

void motorInit(void);
void getSteeringInfo(void);

int main(void)
{
    HAL_Init();                     //初始化HAL库   
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
    uart_init(115200);//初始化USART
	
	LED_Init(); 	KEY_Init();
	
	motorInit();
	TIM3_Init(400-1,4500-1);//定时器3初始化，定时器时钟为90M，分频系数为4500-1，
                             //所以定时器3的频率为90M/4500=20K，自动重装载为400-1，那么定时器周期就是20ms
							//每20ms处理一次
	while(KEY0 != 0);//等待按键被按下，开始行动
//	BalanceInit();//bikebot结构体初始化
//	setValuePrint();
	SCA_Init();		delay_ms(100);
	activateActuatorMode(SCA_ID,SCA_Position_Mode,Block);
//	getSteeringInfo();
	ready_light();	delay_ms(800);
	
	LED0_ON();
//	legRise();
//	wheelSet(1500);//车轮启动 空转时1500大约对应1m/s 1545
	delay_ms(100);
	
	Nuci7Reset();
	printf("begin");
	HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
	
//	HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE

	TIM4->CNT = 0;//清空编码器计数值
	while(1)//无限循环，一定频率控制自行车
	{	
		if(KEY1 == 0){ //如果KEY1被按下  //若没反应，可能是can阻塞了
			//printf("KEY1 Press!\r\n");
			bikebotStop();
		}
		if(itflag == 1){
			Bikedrive();
			Nuci7Write();
			
			itflag = 0;
		}
	} 
}

void motorInit(void){
	/*转向电机初始化，回到零点后失能*/
	CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS1_2TQ,CAN_BS2_2TQ,9,CAN_MODE_NORMAL);	
	SCA_Init();		delay_ms(100);
	SCA_Set();		delay_ms(200);
	SCA_Homing();	delay_ms(1500);
//	activateActuatorMode(SCA_ID,SCA_Position_Mode,Block);
//	for(int i = 0; i < 10; i++){
//		setPositionFast(pSCA_ID1, 0.2*i);	delay_ms(10);
//	}
//	for(int i = 8; i >= 0; i--){
//		setPositionFast(pSCA_ID1, 0.2*i);	delay_ms(10);
//	}
//	getSteeringInfo();
	disableAllActuators();
	/*车轮相关的初始化*/
	wheelInit();
}

void getSteeringInfo(void){
	regainAttrbute(SCA_ID,Unblock);
	printf("位置环: KP %.3f KI %.2f\r\n", pSCA_ID1->Position_Filter_P, pSCA_ID1->Position_Filter_I);
	printf("速度环：KP %.2f KI %.2f\r\n", pSCA_ID1->Velocity_Filter_P, pSCA_ID1->Velocity_Filter_I);
	printf("梯形位置：最大速度 %.2f 最大加速度 %.2f 最大减速度%.2f\r\n", 
		pSCA_ID1->PP_Max_Velocity, pSCA_ID1->PP_Max_Acceleration, pSCA_ID1->PP_Max_Deceleration);
	printf("当前位置：%.2fr\r\n", (pSCA_ID1->Position_Real));
	printf("mode:%d  位置环滤波器：%d\r\n",pSCA_ID1->Mode, pSCA_ID1->Position_Filter_State);
	printf("V: %.2f 速度环输出上限:%.2f\r\n", pSCA_ID1->Voltage ,pSCA_ID1->Velocity_Filter_Limit_H);
	printf("位置环上限: %.2f\r\n", pSCA_ID1->Position_Filter_Limit_H);
}

