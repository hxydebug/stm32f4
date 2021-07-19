/**
  ******************************************************************************
  * @��	�� �� SCA_API.c
  * @��	�� �� INNFOS Software Team
  * @��	�� �� V1.5.3
  * @��	�� �� 2019.09.10
  * @ժ	Ҫ �� SCA ���ƽӿڲ�
  ******************************************************************************/ 
/* Update log --------------------------------------------------------------------*/
//V1.1.0 2019.08.05 ����API���ýӿڸ�ΪID����PC SDK����һ�£��������в����Ķ�дAPI
//V1.5.0 2019.08.16 �������ݽ��շ�ʽ���жϽ��գ������������ͨ�Ź��ܣ���Ӧ���ݷ�������
//					����������ȡ�ϴιػ�״̬��API���Ż��������̡�
//V1.5.1 2019.09.10 ������ѯ����
//V1.5.3 2019.11.15 �Ż����ػ�����

/* Includes ----------------------------------------------------------------------*/
#include "can.h"
#include "sys.h"
#include "usart.h"
#include "SCA_API.h"
#include <string.h>

/* Variable defines --------------------------------------------------------------*/

/* ÿ��SCA����Ҫһ������������Ӧ����Ϣ������ʵ��ʹ���������ж��� SCA_NUM_USE */
SCA_Handler_t SCA_Handler_List[SCA_NUM_USE];

/* Funcation declaration ---------------------------------------------------------*/
extern void warnBitAnaly(SCA_Handler_t* pSCA);

/* Funcation defines -------------------------------------------------------------*/

/****************************�������*******************************/

/**
  * @��	��	��CAN�����ϲ��Ҵ��ڵ�SCA������ӡ�ҵ���ID
  * @��	��	canPort����Ҫ��ѯ������
  * @��	��	��
  * @ע	��	ÿִ̨���������Լ���ID��������ʹ�ò�֪��
  *			��Ӧ��ID�����ô˺�������
  */
void lookupActuators(CAN_Handler_t* canPort)
{
	uint16_t ID;
	uint8_t Found = 0;
	SCA_Handler_t temp;
	
	/* �����б����ԭʼ���� */
	temp = SCA_Handler_List[0];
	
	/* ʹ��һ���б�����в�ѯ */
	SCA_Handler_List[0].Can = canPort;
	
	for(ID = 1; ID <= 0xFF; ID++)
	{
		/* װ���µ�ID */
		SCA_Handler_List[0].ID = ID;
		
		/* �յ���ID�����������ID���� */
		if(isOnline(ID,Block) == SCA_NoError)
		{
			/* ��¼�ҵ��ĸ�������ӡ�ҵ���ID */
			Found++;
			SCA_Debug("Found ID %d in canPort %d\r\n",ID,canPort->CanPort);
		}
	}
	/* �ָ����ĵ����� */
	SCA_Handler_List[0] = temp;
	
	/* �����ʾ��Ϣ */
	SCA_Debug("canPort %d polling done ! Found %d Actuators altogether!\r\n\r\n",canPort->CanPort,Found);
}

/**
  * @��	��	��ʼ��������������ID��CAN�˿���Ϣ
  * @��	��	id����ʼ��ִ������ID
  *			pCan��ʹ�õ�CAN�˿ڵ�ַ
  * @��	��	��
  * @ע	��	���������Ҫ����SCA_NUM_USE
  */
void setupActuators(uint8_t id, CAN_Handler_t* pCan)
{
	static uint32_t i = 0;
	
	/* ������������ʹ������ */
	if(i >= SCA_NUM_USE)	return;
	
	/* �������Ϣ */
	SCA_Handler_List[i].ID = id;
	SCA_Handler_List[i].Can = pCan;
	
	/* �б������� */
	i++;
}

/**
  * @��	��	��λ������������SCA������µ���������
  * @��	��	id��0��ʾȫ����λ����Ϊ0ʱ��λָ��ID�Ŀ�����
  * @��	��	��
  * @ע	��	������ֺ�ƻ�����״̬������SCA�����Ƚ�
  *			SCA�����ϵ磬�ָ����Ƶ�״̬Ȼ��ִ�д˺�
  *			��,��ִ�п����������������������
  */
void resetController(uint8_t id)
{
	uint8_t i,id_temp;
	CAN_Handler_t* pCan_temp = NULL;
	
	if(id == 0)
	{
		/* ���������Ϣ��� */
		for(i = 0; i < SCA_NUM_USE; i++)
		{
			/* ����ID��CAN�˿ڵ�ַ */
			id_temp = SCA_Handler_List[i].ID;
			pCan_temp = SCA_Handler_List[i].Can;
			
			/* �ṹ������ */
			memset(&SCA_Handler_List[i], 0, sizeof(SCA_Handler_List[i]));
			
			/* �ָ�ID��CAN�˿ڵ�ַ */
			SCA_Handler_List[i].ID = id_temp;
			SCA_Handler_List[i].Can = pCan_temp;
		}
	}else
	{
		/* ��ȡ��ID����Ϣ��� */
		SCA_Handler_t* pSCA = getInstance(id);
		if(pSCA == NULL)	return;
		
		/* ����CAN�˿ڵ�ַ */
		pCan_temp = pSCA->Can;
		
		/* �ṹ������ */
		memset(pSCA, 0, sizeof(SCA_Handler_List[0]));
		
		/* �ָ�ID��CAN�˿ڵ�ַ */
		pSCA->ID = id;
		pSCA->Can = pCan_temp;
	}
}

/**
  * @��	��	��ȡָ��ID��SCA��Ϣ���
  * @��	��	id ��Ҫ��ȡ��Ϣ��ִ����ID
  * @��	��	NULL��δ���ҵ���ID����Ϣ���
  *			���������ҵ�����Ϣ���
  */
SCA_Handler_t* getInstance(uint8_t id)
{
	uint8_t i;
	
    for(i = 0; i < SCA_NUM_USE; i++)
        if(SCA_Handler_List[i].ID == id)
            return &SCA_Handler_List[i];
	
    return NULL;
}

/**
  * @��	��	���ִ���������������ߣ�״̬
  * @��	��	id ��Ҫ����ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError����ִ��������
  *			SCA_OverTime����ִ��������
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t isOnline(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���������״̬ */
	pSCA->Online_State = Actr_Disable;
	
	/* ���ö�ȡ������SCAͨ�ţ���������Ӧ��SCA����� */
	Error = SCA_Read(pSCA, R1_Heartbeat);
	if(Error)	return Error;
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);	
		return Error;
	}
	
	/* ����ʽͨ�� */
	while((pSCA->Online_State != Actr_Enable) && (waitime++ < CanOvertime));
	
	if(waitime >= CanOvertime)	{
		return SCA_OperationFailed;
	}
	
	return Error;
}

/**
  * @��	��	���ִ������ʹ��״̬
  * @��	��	id ��Ҫ����ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	Actr_Enable����ִ������ʹ��
  *			Actr_Disable����ִ����δʹ��
  *			
  */
uint8_t isEnable(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ����ն�ȡ��־λ */
	pSCA->paraCache.R_Power_State = Actr_Disable;
	
	/* ���ö�ȡ������SCAͨ�ţ���������Ӧ��SCA����� */
	Error = SCA_Read(pSCA, R1_PowerState);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);	
		return Error;
	}
	
	/* ����ʽͨ�� */
	while((pSCA->paraCache.R_Power_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	���ִ�����Ĳ�������״̬
  * @��	��	id ��Ҫ����ִ����id
  * @��	��	Actr_Enable���в�������
  *			Actr_Disable��û�в�������
  */
uint8_t isUpdate(uint8_t id)
{
	uint8_t State;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* �������״̬������λ */
	State = pSCA->Update_State;
	pSCA->Update_State = Actr_Disable;
	
	return State;
}

/**
  * @��	��	ʹ������ִ����������ʽ
  * @��	��	��
  * @��	��	��
  */
void enableAllActuators()
{
	uint8_t i;

	for(i = 0; i < SCA_NUM_USE; i++)
		enableActuator(SCA_Handler_List[i].ID);
}

/**
  * @��	��	ʧ������ִ����������ʽ
  * @��	��	��
  * @��	��	��
  */
void disableAllActuators()
{
	uint8_t i;

	for(i = 0; i < SCA_NUM_USE; i++)
		disableActuator(SCA_Handler_List[i].ID);
}

/**
  * @��	��	ִ����ʹ��,����ʽ
  * @��	��	id��Ҫʹ�ܵ�ִ����ID
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t enableActuator(uint8_t id)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ��ѯһ�ε�ǰ��ʹ��״̬ */
	Error = isEnable(id, Block);
	if(Error)	return Error;

	/* ����ǰ�Ѿ�����Ŀ��״̬��ֱ�ӷ��سɹ� */
	if(pSCA->Power_State == Actr_Enable)	goto PowerOn;
	
	/* Ŀ�����д�뻺������� */
	pSCA->paraCache.Power_State = Actr_Enable;
	
	/* ִ�п������� */
	Error = SCA_Write_1(pSCA, W1_PowerState, Actr_Enable);
	if(Error)	return Error;

	/* �ȴ������ɹ������¾����Ϣ */
	while((pSCA->Power_State != Actr_Enable) && (waitime++ < CanPowertime));
	if(waitime >= CanPowertime)	return SCA_OperationFailed;			
	
	PowerOn:
	/* ��������״̬ */
	pSCA->Online_State = Actr_Enable;
	
	/* �����豸���кţ�����ID�� */
	getActuatorSerialNumber(id,Block);

	/* ��һ���ϴιػ����쳣״̬ */
	getActuatorLastState(id,Block);
	if(pSCA->Last_State == 0)		//��ʾ�ϴιػ�״̬�쳣
		SCA_Debug("ID:%d Last_State Error\r\n",pSCA->ID);
	
	/*  ����ִ�����������̵���ֵ���ڶ�д����������ʱʹ�ã�
		��ͬ�ͺŵ�SCA��ֵ��ͬ��Ҳ�����ֶ����µ������Ϣ�� 
		�ò���ֵ�Ǳ����ȡ�ġ�*/
	getCurrentRange(id,Block);
	if(pSCA->Current_Max == 0)	//δ��ȡ������������ֵ���޷�д�����ֵ
		SCA_Debug("ID:%d Current_Max Error\r\n",pSCA->ID);
	
	/* ����һ�����в���������У�Ϊ���̿���ʱ����÷����� */
	regainAttrbute(id,Unblock);

	return Error;
}

/**
  * @��	��	ִ����ʧ��,����
  * @��	��	id��Ҫʧ�ܵ�ִ����ID
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t disableActuator(uint8_t id)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ��ѯһ�ε�ǰ��ʹ��״̬ */
	Error = isEnable(id, Block);
	if(Error)	return Error;

	/* ����ǰ�Ѿ�����Ŀ��״̬��ֱ�ӷ��سɹ� */
	if(pSCA->Power_State == Actr_Disable)	return SCA_NoError;
	
	/* Ŀ�����д�뻺������� */
	pSCA->paraCache.Power_State = Actr_Disable;
	
	/* ִ�йػ����� */
	Error = SCA_Write_1(pSCA, W1_PowerState, Actr_Disable);
	if(Error)	return Error;
	
	/* �ȴ��ػ��ɹ� */
	while((pSCA->Power_State != Actr_Disable) && (waitime++ < CanPowertime));
	if(waitime >= CanPowertime)	return SCA_OperationFailed;	
	
	return Error;
}

/**
  * @��	��	ִ�����л�����ģʽ
  * @��	��	id��Ҫ������ִ����id
  *			mode������ģʽ����� SCA_Protocol.h
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t activateActuatorMode(uint8_t id, uint8_t ActuatorMode, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ����ǰ�Ѿ�����Ŀ��״̬��ֱ�ӷ��سɹ� */
	if(pSCA->Mode == ActuatorMode)	return SCA_NoError;
	
	/* Ŀ�����д�뻺������� */
	pSCA->paraCache.Mode = ActuatorMode;

	/* ִ��ģʽ�л����� */
	Error = SCA_Write_1(pSCA, W1_Mode, ActuatorMode);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Mode != ActuatorMode) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;	
	
	return Error;	
}

/**
  * @��	��	ִ������ȡ��ǰ����ģʽ
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getActuatorMode(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ����ն�ȡ�ȴ���־λ */
	pSCA->paraCache.R_Mode = Actr_Disable;
	
	/* ��װ��ȡ����������ֱֵ�ӱ��浽����� */
	Error = SCA_Read(pSCA, R1_Mode);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Mode != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;	
	
	return Error;
}

/**
  * @��	��	ִ������ȡ������Ϣ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getErrorCode(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ����ն�ȡ�ȴ���־λ */
	pSCA->paraCache.R_Error_Code = Actr_Disable;
	
	/* ִ�ж�ȡ������Ϣ���� */
	Error = SCA_Read(pSCA, R2_Error);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Error_Code != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;	
	
	return Error;
}

/**
  * @��	��	ִ�������������Ϣ
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t clearError(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ����ǰ�޴���������[�� */
	if(pSCA->SCA_Warn.Error_Code == 0)	return SCA_NoError;

	/* ִ�М[������ */
	Error = SCA_Write_4(pSCA, W4_ClearError);
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->SCA_Warn.Error_Code != 0) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;	
	
	return Error;
}

/**
  * @��	��	ִ������ȡ��ǰ���в���
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	��
  */
void regainAttrbute(uint8_t id,uint8_t isBlock)
{
	getErrorCode(id,isBlock);
	requestCVPValue(id,isBlock);
	getActuatorMode(id,isBlock);
	getPositionKp(id,isBlock);
	getPositionKi(id,isBlock);
	getPositionUmax(id,isBlock);
	getPositionUmin(id,isBlock);
	getPositionOffset(id,isBlock);
	getMaximumPosition(id,isBlock);
	getMinimumPosition(id,isBlock);
	isPositionLimitEnable(id,isBlock);
	isPositionFilterEnable(id,isBlock);
	getPositionCutoffFrequency(id,isBlock);
	getProfilePositionAcceleration(id,isBlock);
	getProfilePositionDeceleration(id,isBlock);
	getProfilePositionMaxVelocity(id,isBlock);
	getVelocityKp(id,isBlock);
	getVelocityKi(id,isBlock);
	getVelocityUmax(id,isBlock);
	getVelocityUmin(id,isBlock);
	isVelocityFilterEnable(id,isBlock);
	getVelocityCutoffFrequency(id,isBlock);
	getVelocityLimit(id,isBlock);
	getProfileVelocityAcceleration(id,isBlock);
	getProfileVelocityDeceleration(id,isBlock);
	getProfileVelocityMaxVelocity(id,isBlock);
	getCurrentKp(id,isBlock);
    getCurrentKi(id,isBlock);
	isCurrentFilterEnable(id,isBlock);
	getCurrentCutoffFrequency(id,isBlock);
	getCurrentLimit(id,isBlock);
	getVoltage(id,isBlock);
	getLockEnergy(id,isBlock);
	getMotorTemperature(id,isBlock);
	getInverterTemperature(id,isBlock);
	getMotorProtectedTemperature(id,isBlock);
	getMotorRecoveryTemperature(id,isBlock);
	getInverterProtectedTemperature(id,isBlock);
	getInverterRecoveryTemperature(id,isBlock);
}
/**
  * @��	��	ִ�������浱ǰ���в���
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t saveAllParams(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ��մ洢״̬λ */
	pSCA->Save_State = Actr_Disable;
	
	Error = SCA_Write_4(pSCA, W4_Save);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�гɹ� */
	while((pSCA->Save_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanPowertime)	return SCA_OperationFailed;	
	
	return Error;
}


/****************************λ�����*******************************/

/**
  * @��	��	ִ�������õ�ǰλ��ֵ
  * @��	��	id��Ҫ������ִ����id
  *			pos��Ŀ��λ��ֵ��ʵ��ֵ����Χ -127.0R ~ +127.0R
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPosition(uint8_t id, float pos)
{
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	return SCA_Write_3(pSCA, W3_Position, pos);
}

/**
  * @��	��	ִ�������õ�ǰλ��ֵ������
  * @��	��	pSCA��Ҫ������ִ�������ָ����ַ
  *			pos��Ŀ��λ��ֵ��ʵ��ֵ����Χ -127.0R ~ +127.0R
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionFast(SCA_Handler_t* pSCA, float pos)
{
	return SCA_Write_3(pSCA, W3_Position, pos);
}

/**
  * @��	��	ִ������ȡ��ǰλ��ֵ,�����������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPosition(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Real = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_Position);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�гɹ� */
	while((pSCA->paraCache.R_Position_Real != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanPowertime)	return SCA_OperationFailed;	
	
	return Error;
}

/**
  * @��	��	ִ������ȡ��ǰλ��ֵ,����������У�����
  * @��	��	pSCA��Ҫ������ִ���������ַ��ָ��
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionFast(SCA_Handler_t* pSCA, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Real = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_Position);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�гɹ� */
	while((pSCA->paraCache.R_Position_Real != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanPowertime)	return SCA_OperationFailed;	
	
	return Error;
}

/**
  * @��	��	����ִ����λ�û� Kpֵ
  * @��	��	id��Ҫ������ִ����id
  *			Kp��Ŀ��λ�û� Kpֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionKp(uint8_t id,float Kp, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Filter_P = Kp;
	
	Error = SCA_Write_3(pSCA, W3_PositionFilterP, Kp);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Filter_P != Kp) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;	

	return Error;
}

/**
  * @��	��	��ȡִ����λ�û� Kpֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionKp(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Filter_P = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionFilterP);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Filter_P != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;	

	return Error;
}

/**
  * @��	��	����ִ����λ�û� Kiֵ
  * @��	��	id��Ҫ������ִ����id
  *			Ki��Ŀ��λ�û� Kiֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionKi(uint8_t id,float Ki, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Filter_I = Ki;
	
	Error = SCA_Write_3(pSCA, W3_PositionFilterI, Ki);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}

	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Filter_I != Ki) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ�û� Kiֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionKi(uint8_t id, uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Filter_I = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionFilterI);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}

	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Filter_I != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ����λ�û��������ֵ
  * @��	��	id��Ҫ������ִ����id
  *			max��Ŀ��λ�û��������ֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionUmax(uint8_t id,float max,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Filter_Limit_H = max;
	
	Error = SCA_Write_3(pSCA, W3_PositionFilterLimitH, max);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Filter_Limit_H != max) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ�û��������ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionUmax(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Filter_Limit_H = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionFilterLimitH);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}

	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Filter_Limit_H != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ����λ�û��������ֵ
  * @��	��	id��Ҫ������ִ����id
  *			min��Ŀ��λ�û��������ֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionUmin(uint8_t id,float min,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Filter_Limit_L = min;
	
	Error = SCA_Write_3(pSCA, W3_PositionFilterLimitL, min);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Filter_Limit_L != min) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ����λ�û��������ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionUmin(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Filter_Limit_L = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionFilterLimitL);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}

	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Filter_Limit_L != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ����λ��ƫ��ֵ
  * @��	��	id��Ҫ������ִ����id
  *			offset��Ŀ��λ��ƫ��ֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionOffset(uint8_t id, float offset,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Offset = offset;
	
	Error = SCA_Write_3(pSCA, W3_PositionOffset, offset);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Offset != offset) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ����λ��ƫ��ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionOffset(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Offset = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionOffset);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Offset != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ����λ�����ֵ
  * @��	��	id��Ҫ������ִ����id
  *			maxPos��Ŀ��λ�����ֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setMaximumPosition(uint8_t id,float maxPos,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Limit_H = maxPos;
	
	Error = SCA_Write_3(pSCA, W3_PositionLimitH, maxPos);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Limit_H != maxPos) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ�����ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getMaximumPosition(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Limit_H = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionLimitH);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Limit_H != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ����λ����Сֵ
  * @��	��	id��Ҫ������ִ����id
  *			minPos��Ŀ��λ����Сֵ��ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setMinimumPosition(uint8_t id,float minPos,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Limit_L = minPos;
	
	Error = SCA_Write_3(pSCA, W3_PositionLimitL, minPos);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Limit_L != minPos) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ����Сֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getMinimumPosition(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Limit_L = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PositionLimitL);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Limit_L != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	ʹ�ܻ�ʧ��ִ����λ����λ
  * @��	��	id��Ҫ������ִ����id
  *			enable��ʹ��״̬��Actr_Enableʹ�ܣ�Actr_Disableʧ��
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t enablePositionLimit(uint8_t id, uint8_t enable,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Limit_State = enable;
	
	Error = SCA_Write_1(pSCA, W1_PositionLimitState, enable);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Limit_State != enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ����λʹ��״̬�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t isPositionLimitEnable(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Limit_State = Actr_Disable;
	
	Error = SCA_Read(pSCA, R1_PositionLimitState);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Limit_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ�������λ�ã����¼���������λ
  * @��	��	id��Ҫ������ִ����id
  *			homingPos�����λ�ã�ʵ��ֵ����λ R
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setHomingPosition(uint8_t id,float homingPos,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Homing_Value = homingPos;
	
	Error = SCA_Write_3(pSCA, W3_HomingValue, homingPos);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Homing_Value != homingPos) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	ʹ��ִ����λ�û��˲���
  * @��	��	id��Ҫ������ִ����id
  *			enable��ʹ��״̬��Actr_Enableʹ�ܣ�Actr_Disableʧ��
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t enablePositionFilter(uint8_t id,uint8_t enable,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Filter_State = enable;
	
	Error = SCA_Write_1(pSCA, W1_PositionFilterState, enable);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Filter_State != enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ�û��˲���ʹ��״̬�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t isPositionFilterEnable(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Filter_State = Actr_Disable;
	
	Error = SCA_Read(pSCA, R1_PositionFilterState);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Filter_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ����λ�û��˲�������
  * @��	��	id��Ҫ������ִ����id
  *			frequency���˲�������ʵ��ֵ����λ hz
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setPositionCutoffFrequency(uint8_t id, float frequency,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Position_Filter_Value = frequency;
	
	Error = SCA_Write_2(pSCA, W2_PositionFilterValue, frequency);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Position_Filter_Value != frequency) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����λ�û��˲������������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getPositionCutoffFrequency(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Position_Filter_Value = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_PositionFilterValue);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Position_Filter_Value != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	���homing��Ϣ���������Ҽ��޺�0λ������
  * @��	��	id��ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */	
uint8_t clearHomingInfo(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.W_ClearHome = Actr_Disable;
	
	Error = SCA_Write_4(pSCA, W4_ClearHome);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.W_ClearHome != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ��������λ�û������ٶ�
  * @��	��	id��Ҫ������ִ����id
  *			acceleration�������ٶȣ�ʵ��ֵ����λ RPM/S^2
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setProfilePositionAcceleration(uint8_t id, float acceleration,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.PP_Max_Acceleration = acceleration;
	
	/*  ���μ��ٶȴ���ֵ����ʵֵ��IQ20�����������д�ӿ�����
		IQ24��ʽ����ģ���Ҫ��IQ4�ı����������⣬����ֵ��
		��λ��RPM���轫����ֵ����60���RPM��λ��
		��������ֵ = 2^4 * 60 = 960
	*/
	acceleration /= Profile_Scal;
	
	Error = SCA_Write_3(pSCA, W3_PPMaxAcceleration, acceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->PP_Max_Acceleration != acceleration) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ��������λ�û������ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getProfilePositionAcceleration(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_PP_Max_Acceleration = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PPMaxAcceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_PP_Max_Acceleration != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ��������λ�û������ٶ�
  * @��	��	id��Ҫ������ִ����id
  *			deceleration�������ٶȣ�ʵ��ֵ����λ RPM/S^2
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setProfilePositionDeceleration(uint8_t id, float deceleration,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.PP_Max_Deceleration = deceleration;
	
	deceleration /= Profile_Scal;
	
	Error = SCA_Write_3(pSCA, W3_PPMaxDeceleration, deceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->PP_Max_Deceleration != deceleration) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ��������λ�û������ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getProfilePositionDeceleration(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_PP_Max_Deceleration = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PPMaxDeceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_PP_Max_Deceleration != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ��������λ�û�����ٶ�
  * @��	��	id��Ҫ������ִ����id
  *			maxVelocity������ٶȣ�ʵ��ֵ����λ RPM
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setProfilePositionMaxVelocity(uint8_t id, float maxVelocity,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.PP_Max_Velocity = maxVelocity;
	
	maxVelocity /= Profile_Scal;
	
	Error = SCA_Write_3(pSCA, W3_PPMaxVelocity, maxVelocity);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->PP_Max_Velocity != maxVelocity) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ��������λ�û�����ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getProfilePositionMaxVelocity(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_PP_Max_Velocity = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PPMaxVelocity);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_PP_Max_Velocity != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}


/****************************�ٶ����*******************************/

/**
  * @��	��	����ִ������ǰ�ٶ�ֵ
  * @��	��	id��Ҫ������ִ����id
  *			vel��Ŀ���ٶȣ�ʵ��ֵ����λ RPM
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */	
uint8_t setVelocity(uint8_t id,float vel)
{
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	return SCA_Write_3(pSCA, W3_Velocity, vel);
}

/**
  * @��	��	����ִ������ǰ�ٶ�ֵ,����
  * @��	��	pSCA��Ҫ������ִ�������ָ����ַ
  *			vel��Ŀ���ٶȣ�ʵ��ֵ����λ RPM
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */	
uint8_t setVelocityFast(SCA_Handler_t* pSCA,float vel)
{
	return SCA_Write_3(pSCA, W3_Velocity, vel);
}


/**
  * @��	��	��ȡִ������ǰ�ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */	
uint8_t getVelocity(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Real = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_Velocity);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Real != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ������ǰ�ٶȣ������������,����
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */	
uint8_t getVelocityFast(SCA_Handler_t* pSCA,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Real = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_Velocity);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Real != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ������������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVelocityKp(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Filter_P = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_VelocityFilterP);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Filter_P != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ�����ٶȻ�����
  * @��	��	id��Ҫ������ִ����id
  *			Kp���ٶȻ�������ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setVelocityKp(uint8_t id,float Kp,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Filter_P = Kp;
	
	Error = SCA_Write_3(pSCA, W3_VelocityFilterP, Kp);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Filter_P != Kp) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ����֣������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVelocityKi(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Filter_I = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_VelocityFilterI);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Filter_I != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ�����ٶȻ�����
  * @��	��	id��Ҫ������ִ����id
  *			Ki���ٶȻ����֣�ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setVelocityKi(uint8_t id, float Ki,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Filter_I = Ki;
	
	Error = SCA_Write_3(pSCA, W3_VelocityFilterI, Ki);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Filter_I != Ki) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ��������޷��������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVelocityUmax(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Filter_Limit_H = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_VelocityFilterLimitH);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Filter_Limit_H != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ�����ٶȻ��������޷�
  * @��	��	id��Ҫ������ִ����id
  *			max���������޷���ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setVelocityUmax(uint8_t id, float max,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Filter_Limit_H = max;
	
	Error = SCA_Write_3(pSCA, W3_VelocityFilterLimitH, max);	
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Filter_Limit_H != max) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ���С����޷��������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVelocityUmin(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Filter_Limit_L = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_VelocityFilterLimitL);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Filter_Limit_L != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ�����ٶȻ���С����޷�
  * @��	��	id��Ҫ������ִ����id
  *			min����С����޷���ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setVelocityUmin(uint8_t id, float min,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Filter_Limit_L = min;
	
	Error = SCA_Write_3(pSCA, W3_VelocityFilterLimitL, min);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Filter_Limit_L != min) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ��ٶ�����
  * @��	��	id��Ҫ������ִ����id
  * @��	��	�ٶȻ��ٶ����̣�ʵ��ֵ
  */
float getVelocityRange(uint8_t id)
{
	return Velocity_Max;
}

/**
  * @��	��	ʹ��ִ�����ٶȻ��˲���
  * @��	��	id��Ҫ������ִ����id
  *			enable��ʹ��״̬��Actr_Enableʹ�ܣ�Actr_Disableʧ��
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t enableVelocityFilter(uint8_t id,uint8_t enable,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Filter_State = enable;
	
	Error = SCA_Write_1(pSCA, W1_VelocityFilterState, enable);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Filter_State != enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ��˲���ʹ��״̬�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t isVelocityFilterEnable(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Filter_State = Actr_Disable;
	
	Error = SCA_Read(pSCA, R1_VelocityFilterState);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Filter_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ��˲������������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVelocityCutoffFrequency(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Filter_Value = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_VelocityFilterValue);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Filter_Value != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ�����ٶȻ��˲�������
  * @��	��	id��Ҫ������ִ����id
  *			frequency���˲�������ʵ��ֵ����λ hz
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setVelocityCutoffFrequency(uint8_t id, float frequency,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Filter_Value = frequency;
	
	Error = SCA_Write_2(pSCA, W2_VelocityFilterValue, frequency);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Filter_Value != frequency) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ�����ٶȻ������޷�
  * @��	��	id��Ҫ������ִ����id
  *			limit�������޷�
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setVelocityLimit(uint8_t id,float limit,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Velocity_Limit = limit;
	
	Error = SCA_Write_3(pSCA, W3_VelocityLimit, limit);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Velocity_Limit != limit) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����ٶȻ������޷��������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVelocityLimit(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_Velocity_Limit = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_VelocityLimit);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Velocity_Limit != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ���������ٶȻ����ٶ�
  * @��	��	id��Ҫ������ִ����id
  *			acceleration�����ٶȣ�ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setProfileVelocityAcceleration(uint8_t id,float acceleration,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.PV_Max_Acceleration = acceleration;
	
	acceleration /= Profile_Scal;
	
	Error = SCA_Write_3(pSCA, W3_PVMaxAcceleration, acceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->PV_Max_Acceleration != acceleration) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ���������ٶȻ����ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getProfileVelocityAcceleration(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_PV_Max_Acceleration = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PVMaxAcceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_PV_Max_Acceleration != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ���������ٶȻ����ٶ�
  * @��	��	id��Ҫ������ִ����id
  *			deceleration�����ٶȣ�ʵ��ֵ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setProfileVelocityDeceleration(uint8_t id,float deceleration,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.PV_Max_Deceleration = deceleration;
	
	deceleration /= Profile_Scal;
	
	Error = SCA_Write_3(pSCA, W3_PVMaxDeceleration, deceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->PV_Max_Deceleration != deceleration) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ���������ٶȻ����ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getProfileVelocityDeceleration(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_PV_Max_Deceleration = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PVMaxDeceleration);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_PV_Max_Deceleration != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ���������ٶȻ�����ٶ�
  * @��	��	id��Ҫ������ִ����id
  *			maxVelocity������ٶȣ�ʵ��ֵ����λ RPM
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setProfileVelocityMaxVelocity(uint8_t id, float maxVelocity,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.PV_Max_Velocity = maxVelocity;
	
	maxVelocity /= Profile_Scal;
	
	Error = SCA_Write_3(pSCA, W3_PVMaxVelocity, maxVelocity);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->PV_Max_Velocity != maxVelocity) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ���������ٶȻ�����ٶȣ������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getProfileVelocityMaxVelocity(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	/* ���״̬λ */
	pSCA->paraCache.R_PV_Max_Velocity = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_PVMaxVelocity);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_PV_Max_Velocity != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}


/****************************�������*******************************/

/**
  * @��	��	����ִ������ǰ����ֵ
  * @��	��	id��Ҫ������ִ����id
  *			current����ǰ����ֵ��ʵ��ֵ����λ A
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setCurrent(uint8_t id,float current)
{
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	return SCA_Write_3(pSCA, W3_Current, current);
}

/**
  * @��	��	����ִ������ǰ����ֵ������
  * @��	��	pSCA��Ҫ������ִ�������ָ����ַ
  *			current����ǰ����ֵ��ʵ��ֵ����λ A
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setCurrentFast(SCA_Handler_t* pSCA,float current)
{
	return SCA_Write_3(pSCA, W3_Current, current);
}

/**
  * @��	��	��ȡִ������ǰ����ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrent(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Real = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_Current);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)	
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Real != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ������ǰ����ֵ�������������,����
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrentFast(SCA_Handler_t* pSCA,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Real = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_Current);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Real != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ��������������ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrentKp(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Filter_P = Actr_Disable;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;
	
	Error = SCA_Read(pSCA, R3_CurrentFilterP);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Filter_P != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ�������������֣������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrentKi(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Filter_I = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_CurrentFilterI);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Filter_I != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	��ȡִ�����������̣������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrentRange(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Max = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_Current_Max);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Max != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	ʹ��ִ�����������˲���
  * @��	��	id��Ҫ������ִ����id
  *			enable��ʹ��״̬��Actr_Enableʹ�ܣ�Actr_Disableʧ��
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t enableCurrentFilter(uint8_t id,uint8_t enable,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Current_Filter_State = enable;
	
	Error = SCA_Write_1(pSCA, W1_CurrentFilterState, enable);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Current_Filter_State != enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ�����������˲���ʹ��״̬�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t isCurrentFilterEnable(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Filter_State = Actr_Disable;
	
	Error = SCA_Read(pSCA, R1_CurrentFilterState);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Filter_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	��ȡִ�����������˲������������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrentCutoffFrequency(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Filter_Value = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_CurrentFilterValue);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Filter_Value != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	����ִ�����������˲�������
  * @��	��	id��Ҫ������ִ����id
  *			frequency��Ŀ���ֹƵ�ʣ���λ hz
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setCurrentCutoffFrequency(uint8_t id, float frequency,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Current_Filter_Value = frequency;
	
	Error = SCA_Write_2(pSCA, W2_CurrentFilterValue, frequency);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Current_Filter_Value != frequency) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ���������������޷�
  * @��	��	id��Ҫ������ִ����id
  *			limit�������޷�
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setCurrentLimit(uint8_t id,float limit,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Current_Limit = limit;
	
	Error = SCA_Write_3(pSCA, W3_CurrentLimit, limit);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Current_Limit != limit) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ���������������޷��������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getCurrentLimit(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Current_Limit = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_CurrentLimit);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Current_Limit != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/****************************��������*******************************/

/**
  * @��	��	��ȡִ������ѹ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getVoltage(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Voltage = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_Voltage);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Voltage != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ������ת�����������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getLockEnergy(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Blocked_Energy = Actr_Disable;
	
	Error = SCA_Read(pSCA, R3_BlockEngy);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Blocked_Energy != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	����ִ������ת����ֵ
  * @��	��	id��Ҫ������ִ����id
  *			energy����ת����ֵ��ʵ��ֵ����λ J
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setLockEnergy(uint8_t id,float energy,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Blocked_Energy = energy;
	
	Error = SCA_Write_3(pSCA, W3_BlockEngy, energy);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Blocked_Energy != energy) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ��������¶�ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getMotorTemperature(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Motor_Temp = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_MotorTemp);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Motor_Temp != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	��ȡִ����������¶�ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getInverterTemperature(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Inverter_Temp = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_InverterTemp);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Inverter_Temp != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	��ȡִ������������¶�ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getMotorProtectedTemperature(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Inverter_Protect_Temp = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_MotorProtectTemp);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Inverter_Protect_Temp != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	����ִ������������¶�ֵ
  * @��	��	id��Ҫ������ִ����id
  *			temp����������¶�ֵ��ʵ��ֵ����λ ���϶�
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setMotorProtectedTemperature(uint8_t id,float temp,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Motor_Protect_Temp = temp;
	
	Error = SCA_Write_2(pSCA, W2_MotorProtectTemp, temp);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Motor_Protect_Temp != temp) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ��������ָ��¶�ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getMotorRecoveryTemperature(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Motor_Recover_Temp = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_MotorRecoverTemp);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Motor_Recover_Temp != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	����ִ��������ָ��¶�ֵ
  * @��	��	id��Ҫ������ִ����id
  *			temp������ָ��¶�ֵ��ʵ��ֵ����λ ���϶�
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setMotorRecoveryTemperature(uint8_t id,float temp,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Motor_Recover_Temp = temp;
	
	Error = SCA_Write_2(pSCA, W2_MotorRecoverTemp, temp);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Motor_Recover_Temp != temp) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ��������������¶�ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getInverterProtectedTemperature(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Inverter_Protect_Temp = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_InverterProtectTemp);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Inverter_Protect_Temp != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	����ִ��������������¶�ֵ
  * @��	��	id��Ҫ������ִ����id
  *			temp������������¶�ֵ��ʵ��ֵ����λ ���϶�
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setInverterProtectedTemperature(uint8_t id,float temp,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Inverter_Protect_Temp = temp;
	
	Error = SCA_Write_2(pSCA, W2_InverterProtectTemp, temp);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Inverter_Protect_Temp != temp) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	��ȡִ����������ָ��¶�ֵ�������������
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getInverterRecoveryTemperature(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Inverter_Recover_Temp = Actr_Disable;
	
	Error = SCA_Read(pSCA, R2_InverterRecoverTemp);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Inverter_Recover_Temp != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	����ִ����������ָ��¶�ֵ
  * @��	��	id��Ҫ������ִ����id
  *			temp��������ָ��¶�ֵ��ʵ��ֵ����λ ���϶�
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setInverterRecoveryTemperature(uint8_t id,float temp,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.Inverter_Recover_Temp = temp;
	
	Error = SCA_Write_2(pSCA, W2_InverterRecoverTemp, temp);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->Inverter_Recover_Temp != temp) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;

	return Error;
}

/**
  * @��	��	����ִ������id
  * @��	��	newID����id
  *			currentID����ǰid
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t setActuatorID(uint8_t currentID, uint8_t newID,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;

	/* ���Ŀ��ID�Ƿ��Ѵ��� */
	pSCA = getInstance(newID);
	if(pSCA != NULL)	return SCA_OperationFailed;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(currentID);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* Ŀ�����д�뻺�棬�ȴ����� */
	pSCA->paraCache.ID = newID;
	
	Error = SCA_Write_5(pSCA, W5_ChangeID, newID);
	if(Error)	return Error;

	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->ID != newID) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

/**
  * @��	��	��ȡִ���������кţ����浽�����
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getActuatorSerialNumber(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Serial_Num = Actr_Disable;
	
	Error = SCA_Read(pSCA, R5_ShakeHands);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Serial_Num != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

/**
  * @��	��	��ȡִ�����ϴεĹػ�״̬�����浽�����
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t getActuatorLastState(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_Last_State = Actr_Disable;
	
	Error = SCA_Read(pSCA, R1_LastState);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_Last_State != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;

}

 /**
  * @��	��	��ȡ�����ٶ�λ�õ�ֵ������������У�Ч�ʸ�
  * @��	��	id��Ҫ������ִ����id
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t requestCVPValue(uint8_t id,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;
	SCA_Handler_t* pSCA = NULL;
	
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if(pSCA == NULL)	return SCA_UnknownID;

	/* ���״̬λ */
	pSCA->paraCache.R_CVP = Actr_Disable;
	
	Error = SCA_Read(pSCA, R4_CVP);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_CVP != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}

 /**
  * @��	��	��ȡ�����ٶ�λ�õ�ֵ������������У�Ч�ʸߣ�����
  * @��	��	pSCA��Ҫ������ִ�������ָ����ַ
  *			isBlock��BlockΪ����ʽ��UnblockΪ������ʽ
  * @��	��	SCA_NoError�������ɹ�
  *			����ͨ�Ŵ���μ� SCA_Error �����б�
  */
uint8_t requestCVPValueFast(SCA_Handler_t* pSCA,uint8_t isBlock)
{
	uint8_t Error;
	uint32_t waitime = 0;

	/* ���״̬λ */
	pSCA->paraCache.R_CVP = Actr_Disable;
	
	Error = SCA_Read(pSCA, R4_CVP);
	if(Error)	return Error;
	
	/* ������ */
	if(isBlock == Unblock)
	{
		/* ���������ͺ���ʱ������ֹ���߹��� */
		SCA_Delay(SendInterval);
		return Error;
	}
	
	/* �ȴ�ִ�н�� */
	while((pSCA->paraCache.R_CVP != Actr_Enable) && (waitime++ < CanOvertime));
	if(waitime >= CanOvertime)	return SCA_OperationFailed;
	
	return Error;
}
