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
 ALIENTEK ������STM32F429������	���г�ƽ��ʵ��
 ���ߣ�������
************************************************/

/*��ע��
 *��IMU roll angle���ԣ�������бΪ������
 *���������߽�PD12�����߽�PD13
 *ת������תΪ������������������ϵ�෴
 *
 */

void motorInit(void);
void getSteeringInfo(void);

int main(void)
{
    HAL_Init();                     //��ʼ��HAL��   
    Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
    delay_init(180);                //��ʼ����ʱ����
    uart_init(115200);//��ʼ��USART
	
	LED_Init(); 	KEY_Init();
	
	motorInit();
	TIM3_Init(400-1,4500-1);//��ʱ��3��ʼ������ʱ��ʱ��Ϊ90M����Ƶϵ��Ϊ4500-1��
                             //���Զ�ʱ��3��Ƶ��Ϊ90M/4500=20K���Զ���װ��Ϊ400-1����ô��ʱ�����ھ���20ms
							//ÿ20ms����һ��
	while(KEY0 != 0);//�ȴ����������£���ʼ�ж�
//	BalanceInit();//bikebot�ṹ���ʼ��
//	setValuePrint();
	SCA_Init();		delay_ms(100);
	activateActuatorMode(SCA_ID,SCA_Position_Mode,Block);
//	getSteeringInfo();
	ready_light();	delay_ms(800);
	
	LED0_ON();
//	legRise();
//	wheelSet(1500);//�������� ��תʱ1500��Լ��Ӧ1m/s 1545
	delay_ms(100);
	
	Nuci7Reset();
	printf("begin");
	HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
	
//	HAL_TIM_Base_Start_IT(&TIM3_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE

	TIM4->CNT = 0;//��ձ���������ֵ
	while(1)//����ѭ����һ��Ƶ�ʿ������г�
	{	
		if(KEY1 == 0){ //���KEY1������  //��û��Ӧ��������can������
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
	/*ת������ʼ�����ص�����ʧ��*/
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
	/*������صĳ�ʼ��*/
	wheelInit();
}

void getSteeringInfo(void){
	regainAttrbute(SCA_ID,Unblock);
	printf("λ�û�: KP %.3f KI %.2f\r\n", pSCA_ID1->Position_Filter_P, pSCA_ID1->Position_Filter_I);
	printf("�ٶȻ���KP %.2f KI %.2f\r\n", pSCA_ID1->Velocity_Filter_P, pSCA_ID1->Velocity_Filter_I);
	printf("����λ�ã�����ٶ� %.2f �����ٶ� %.2f �����ٶ�%.2f\r\n", 
		pSCA_ID1->PP_Max_Velocity, pSCA_ID1->PP_Max_Acceleration, pSCA_ID1->PP_Max_Deceleration);
	printf("��ǰλ�ã�%.2fr\r\n", (pSCA_ID1->Position_Real));
	printf("mode:%d  λ�û��˲�����%d\r\n",pSCA_ID1->Mode, pSCA_ID1->Position_Filter_State);
	printf("V: %.2f �ٶȻ��������:%.2f\r\n", pSCA_ID1->Voltage ,pSCA_ID1->Velocity_Filter_Limit_H);
	printf("λ�û�����: %.2f\r\n", pSCA_ID1->Position_Filter_Limit_H);
}

