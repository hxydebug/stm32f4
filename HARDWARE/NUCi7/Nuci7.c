#include "Nuci7.h"
#include "usart.h"
#include "string.h"
#include "balance.h"

Bikemsg bicycle;
Command bikecmd;
unsigned char TxBuffer[14] = {0};
float last_vel = 0;

int itflag = 0;

void Nuci7Write(void){
		//读取数据
		requestCVPValueFast(pSCA_ID1,Block);
		bicycle.data[0] = (float)(pSCA_ID1->Position_Real)*(-0.174533);
		bicycle.data[1] = (float)(pSCA_ID1->Velocity_Real)*(-0.167);
		bicycle.data[2] = (double)read_encoder() * PI*METER_WHEEL_D/16;
		if(bicycle.data[2] > 10) bicycle.data[2] = VR;
		
		//数据打包
		TxBuffer[0] = 0x55;
		memcpy(&TxBuffer[1],&bicycle,12);
		TxBuffer[13] = 0x51;
	
		HAL_UART_Transmit(&UART3_Handler, (uint8_t *)TxBuffer, 14,0xFFFF);

}

void Nuci7_callback(void){
		static unsigned char nucRxBuffer[9];
		static unsigned char nucRxCnt = 0;	
		
		nucRxBuffer[nucRxCnt++] = aRxBuffer[0];
		if(nucRxBuffer[0] != 0x55){//数据头不对
			nucRxCnt = 0;
			
//			HAL_UART_Receive_IT(&UART3_Handler, (uint8_t *)aRxBuffer, 1);
			return;
		}
		if(nucRxCnt < 9){//数据不满9个
//			HAL_UART_Receive_IT(&UART3_Handler, (uint8_t *)aRxBuffer, 1);
			return;
		}
		else
		{
			memcpy(&bikecmd,&nucRxBuffer[1],8);
			itflag = 1;
//			printf("%.2f   ",phi_cmd);
			nucRxCnt=0;
//			HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, 1);
		}
}

void Bikedrive(void){
		float phi_cmd;
		u16 vel;
		phi_cmd = bikecmd.phi;
		vel = bikecmd.speed;
		wheelSet(vel);
		setPositionFast(pSCA_ID1, phi_cmd);
		printf("%.2f   ",phi_cmd);
}

void Nuci7Reset(void){
		TxBuffer[0] = 0x55;
		TxBuffer[13] = 0x50;
		HAL_UART_Transmit(&UART3_Handler, (uint8_t *)TxBuffer, 14, 0xFFFF);
}
