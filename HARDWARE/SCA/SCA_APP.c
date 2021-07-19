/**
  ******************************************************************************
  * @��	�� �� SCA_APP.c
  * @��	�� �� ������
  * @��	�� �� V1.0
  * @��	�� �� 2020.6.28
  * @ժ	Ҫ �� SCA Ӧ�ó���
  ******************************************************************************/ 
/* Update log --------------------------------------------------------------------*/
//V1.1.0 2020.6.28  ����SCA���Գ���

/* Includes ----------------------------------------------------------------------*/
#include "can.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "SCA_APP.h"
#include "SCA_API.h"

/* Variable defines --------------------------------------------------------------*/
SCA_Handler_t* pSCA_ID1 = NULL;		//��дָ�룬�����ڻ�ȡִ�������������FAST�ͺ���

/* CAN�˿���Ϣ���壬���ڰ�SCA�����ʵ�ֶ�˿ڿ��ơ���ֲʱ����ʵ���������� */
CAN_Handler_t CAN_Port1;

/* Funcation defines -------------------------------------------------------------*/

/**
  * @��	��	��������ʼ��
  * @��	��	��
  * @��	��	��
  */
void SCA_Init()
{
	/* ��ʼ��CAN�˿ڲ��� */
	CAN_Port1.CanPort = 1;			//��Ƕ˿ں�
	CAN_Port1.Retry = 2;			//ʧ���ط�����
	CAN_Port1.Send = CAN1_Send_Msg;	//CAN1�˿ڷ��ͺ���
	
	
	/* װ��ִ������ID����ʹ�õ�CAN�˿ں� */
	setupActuators(SCA_ID, &CAN_Port1);	//ID1 ��CAN1
	
	/* ��ȡID1��2�Ĳ������ */
	pSCA_ID1 = getInstance(SCA_ID);
	
	/* ��������ִ���� */
	enableAllActuators();
}

/**
  * @��	��	�����������
  * @��	��	��
  * @��	��	��
  */
void SCA_Set()
{
	setPositionKp(SCA_ID,0.3,Block);//0.45
	setVelocityKp(SCA_ID,4,Block);//5
	setProfilePositionAcceleration(SCA_ID, 300 ,Block);
	setProfilePositionDeceleration(SCA_ID, -3000 ,Block);
	setProfilePositionMaxVelocity(SCA_ID, 3000 ,Block);
}

/**
  * @��	��	λ�ù���
  * @��	��	��
  * @��	��	��
  */
void SCA_Homing()
{
	/* δ����ֱ���˳� */
	if(pSCA_ID1->Power_State == Actr_Disable)	{
		//printf("\r\nδ����\r\n");
		return;
	}
	
	/* �л�ִ��������ģʽ������λ��ģʽ */
	activateActuatorMode(SCA_ID,SCA_Profile_Position_Mode,Block);
//	activateActuatorMode(SCA_ID,SCA_Position_Mode,Block);
	/* ���� ִ���� */
	setPosition(SCA_ID,0);
	
	/* �ȴ�����ɹ� */
	do
	{
		getPosition(SCA_ID,Unblock);
		delay_ms(100);
	}
	while((pSCA_ID1->Position_Real > 0.1f)||(pSCA_ID1->Position_Real < -0.1f));
}

#if DEMO
/**
  * @��	��	��CAN�����ϲ��Ҵ��ڵ�SCA������ӡ�ҵ���ID
  * @��	��	��
  * @��	��	��
  * @ע	��	ÿִ̨���������Լ���ID��������ʹ�ò�֪��
  *			��Ӧ��ID�����ô˺������ҡ��˹�����Ҫ��SCA_DEBUGER
  */
void SCA_Lookup()
{
	/* ��ʼ��CAN�˿ڲ��� */
	CAN_Port1.CanPort = 1;			//��Ƕ˿ں�
	CAN_Port1.Retry = 2;			//ʧ���ط�����
	CAN_Port1.Send = CAN1_Send_Msg;	//CAN1�˿ڷ��ͺ���
	
	
	/* ���ú������Ҷ�Ӧ�����ϴ��ڵ�ID */
	lookupActuators(&CAN_Port1);	//��ѯCAN1����
}

/**
  * @��	��	����ת�л�����
  * @��	��	��
  * @��	��	��
  */
void SCA_Exp1()
{
	/* δ����ֱ���˳� */
	if(pSCA_ID1->Power_State == Actr_Disable)	return;
	
	/* ���� */
	SCA_Homing();
	
	/* ��������ת */
	setPosition(SCA_ID,3);			//��ͨ������ID����ִ����
	delay_ms(1000);
	
	setPosition(SCA_ID,-3);
	delay_ms(1000);
	
	setPosition(SCA_ID,3);
	delay_ms(1000);
	
	setPosition(SCA_ID,-3);
	delay_ms(1000);
} 

/**
  * @��	��	�ߵ����л�
  * @��	��	��
  * @��	��	��
  */
void SCA_Exp2()
{
	/* δ����ֱ���˳� */
	if(pSCA_ID1->Power_State == Actr_Disable)	return;
	
	/* �л�ִ��������ģʽ�������ٶ�ģʽ */
	activateActuatorMode(SCA_ID,SCA_Profile_Velocity_Mode,Block);
	
	/* �ߵ����л� */
	setVelocity(SCA_ID,300);
	delay_ms(1000);
	
	setVelocity(SCA_ID,600);
	delay_ms(1000);
	
	setVelocity(SCA_ID,300);
	delay_ms(1000);
	
	setVelocity(SCA_ID,600);
	delay_ms(1000);
	
	/* ֹͣ */
	setVelocity(SCA_ID,0);
}

/**
  * @��	��	��λ����
  * @��	��	��
  * @��	��	��
  */
void homePositionReset()
{
	getPosition(SCA_ID, Block);
	setHomingPosition(SCA_ID, pSCA_ID1->Position_Real, Block);
}


/**
  * @��	��	��λ������
  * @��	��	��
  * @��	��	��
  */
void resetAll()
{
	resetController(SCA_ID);
}

////////////////////////////////////////////////////////////////////////////////////

/**
  * @��	��	���ڴ�ӡ��ʾ��Ϣ
  * @��	��	��
  * @��	��	��
  */
void Log()
{
	printf("\r\n��ӭʹ�� INNFOS SCA �������ԣ� \r\n");
	printf("��ϸͨ��Э��μ� INNFOS WIKI�� \r\n");
	printf("���� 1 ��ѯ�����ϵ�ִ����ID�� \r\n");
	printf("���� 2 ʹ��Ĭ��ID��ʼ��SCA�������� \r\n");
	printf("���� 3 ����λ�ù�����Գ��� \r\n");
	printf("���� 4 ��������ת���Գ��� \r\n");
	printf("���� 5 ����ߵ��ٲ��Գ��� \r\n");
	printf("���� 6 ��ִ�����ػ��� \r\n");
	printf("���� 7 ���õ�ǰλ��Ϊ��λ�� \r\n");
	printf("���� 8 ��λ��\r\n");
}

uint8_t cmd = 0;					//�ⲿ��������

void CMD_Usart(void)
{
	if(cmd == 0)	
		cmd = USART1->DR;
	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
}
/**
  * @��	��	�����������
  * @��	��	cmd�����յ���ָ��
  * @��	��	��
  */
void CMD_Handler(uint8_t cmd)
{
	switch(cmd)
	{
		case 1:
			printf("\r\nִ����ѯ����\r\n");
		
			/* ������ѯ���� */
			SCA_Lookup();
		
			printf("��ѯ������\r\n");
		break;
		
		case 2:
			printf("\r\nSCA��ʼ����\r\n");
		
			/* ���ó�ʼ������ */
			SCA_Init();
		
			/* �ȴ�ִ�����ȶ� */
			delay_ms(500);
		
			printf("SCA��ʼ��������\r\n");
			break;
		
		case 3:
			printf("\r\n����λ�ù�����ԣ�\r\n");
		
			/* ���ò��Գ��� λ�ù��� */
			SCA_Homing();
		
			printf("λ�ù�����Խ�����\r\n");
			break;
			
		case 4:
			printf("\r\n��������ת�л����ԣ�\r\n");
		
			/* ���ò��Գ��� ����ת�л� */
			SCA_Exp1();
		
			printf("����ת�л����Խ�����\r\n");
			break;
		
		case 5:
			printf("\r\n����ߵ����л����ԣ�\r\n");
			
			/* ���ò��Գ��� �ߵ����л� */
			SCA_Exp2();
		
			printf("�ߵ����л����Խ�����\r\n");
			break;
		
		case 6:
			printf("\r\nִ�����ػ���\r\n");
			
			/* �ر�����ִ���� */
			disableAllActuators();
		
			printf("ִ�����ػ�������\r\n");
			break;
		
		case 7:
			printf("\r\nִ������λ���ã�\r\n");
			
			/* ����ִ�������õ�ǰλ��Ϊ��λ */
			homePositionReset();;
		
			printf("ִ������λ���ý�����\r\n");
			break;
				
		case 8:
			printf("\r\n��λ\r\n");
			resetAll();
			printf("\r\n��λ����\r\n");
		
		default:
			Log();
			break;
	}
}
#endif
