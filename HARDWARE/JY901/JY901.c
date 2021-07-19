#include "usart.h"
#include "JY901.h"
#include "string.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//ALIENTEK STM32F429������
//JY-GPSIMU���ݶ�ȡ		   
//�޸�����:2020/6/21
//�汾��V1.0
//********************************************************************************

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 		stcAngle;
//struct SMag 		stcMag;
//struct SDStatus 	stcDStatus;
//struct SPress 		stcPress;
//struct SLonLat 		stcLonLat;
//struct SGPSV 		stcGPSV;
//struct SQ      		stcQ;

//float acc[3];
//float gyro[3];
//float angle[3];


void JYread()
{
		static unsigned char ucRxBuffer[12];
		static unsigned char ucRxCnt = 0;	
		
		ucRxBuffer[ucRxCnt++] = aRxBuffer[0];
		if(ucRxBuffer[0] != 0x55){//����ͷ����
			ucRxCnt = 0;
			
//			HAL_UART_Receive_IT(&UART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
			return;
		}
		if(ucRxCnt < 11){//���ݲ���11��
//			HAL_UART_Receive_IT(&UART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
			return;
		}
		else
		{
			switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ�󿽱�����Ӧ�ṹ��
			{
//				case 0x50:
//					memcpy(&stcTime,&ucRxBuffer[2],8);
//					break;
//				case 0x51:	
//					memcpy(&stcAcc,&ucRxBuffer[2],8);
//					acc[0] = (float)stcAcc.a[0]/32768*4;
//					acc[1] = (float)stcAcc.a[1]/32768*4;
//					acc[2] = (float)stcAcc.a[2]/32768*4;
//					break;
				case 0x52:
					memcpy(&stcGyro,&ucRxBuffer[2],8);
//					gyro[0] = (float)stcGyro.w[0]/32768*500;
//					gyro[1] = (float)stcGyro.w[1]/32768*500;
//					gyro[2] = (float)stcGyro.w[2]/32768*500;
					break;
				case 0x53:
					memcpy(&stcAngle,&ucRxBuffer[2],8);
//					angle[0] = (float)stcAngle.Angle[0]/32768*180;
//					angle[1] = (float)stcAngle.Angle[1]/32768*180;
//					angle[2] = (float)stcAngle.Angle[2]/32768*180;
					break;
				//case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
				//case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
				//case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
				//case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
				//case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
				//case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
			}
			ucRxCnt=0;//��ջ�����
//			HAL_UART_Receive_IT(&UART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//������һ�ν���
		}
}


void printJYResult()
{	
	//���ʱ��
	printf("Time:20%d-%d-%d %02d:%02d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,
		stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,
		(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);	
	
	//������ٶ�
//	printf("Acc:%.3f %.3f %.3f\r\n",acc[0], acc[1], acc[2]);
//	delay_ms(1);
	
//	//������ٶ�
//	printf("Gyro:%.3f %.3f %.3f\r\n",gyro[0], gyro[1], gyro[2]);
//	delay_ms(1);
//	
//	//����Ƕ�
//	printf("Angle:%.3f %.3f %.3f\r\n",angle[0], angle[1], angle[2]);
//	delay_ms(1);
	
//	//����ų�
//	printf("Mag:%d %d %d\r\n",stcMag.h[0],stcMag.h[1],stcMag.h[2]);
//	delay_ms(10);
//	
//	//�����ѹ���߶�
//	printf("Pressure:%ld Height%.2f\r\n",stcPress.lPressure,(float)stcPress.lAltitude/100);
//	delay_ms(10);
//	
//	//����˿�״̬
//	printf("DStatus:%d %d %d %d\r\n",stcDStatus.sDStatus[0],stcDStatus.sDStatus[1],
//		stcDStatus.sDStatus[2],stcDStatus.sDStatus[3]);
//	delay_ms(10);
//	
//	//�����γ��
//	printf("Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",stcLonLat.lLon/10000000,
//		(double)(stcLonLat.lLon % 10000000)/1e5,stcLonLat.lLat/10000000,
//		(double)(stcLonLat.lLat % 10000000)/1e5);
//	delay_ms(10);
//	
//	//�������
//	printf("GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)stcGPSV.sGPSHeight/10,
//		(float)stcGPSV.sGPSYaw/10,(float)stcGPSV.lGPSVelocity/1000);
//	delay_ms(10);
//	
//	//�����Ԫ��
//	printf("Four elements:%.5f %.5f %.5f %.5f\r\n\r\n",(float)stcQ.q[0]/32768,
//		(float)stcQ.q[1]/32768,(float)stcQ.q[2]/32768,(float)stcQ.q[3]/32768);
//	delay_ms(10);//�ȴ��������
}
