#ifndef BALANCE_H
#define BALANCE_H

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "JY901.h"
#include "wheel.h"
#include "encoder.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "leg.h"

#define		UPDATE_V	1 
#define		VR 			1.15
//IMU�����б������Ϊ����ת��������Ϊ��
#define 	PI			3.1415926
#define		DEG2RAD(x)	((double)(x)*PI/180)
#define		RAD2DEG(x)	((double)(x)*180/PI)

#define		BODY_mb		20
#define		BODY_hb		0.4
#define		BODY_Jb		0.8
#define		BODY_l		0.87
#define		BODY_lb		0.42
#define		BODY_R		0.21
#define		EPSILON		(DEG2RAD(17))		//��λ��rad
#define		BODY_lt		(BODY_R*tan(EPSILON))//0.064
#define		GRAVITY		9.8					//g

#define		ANGLE_G		1.0
/*������ϵ��*/
#define		BALANCE_K	1.05
#define 	BALANCE_A1	25		//25
#define 	BALANCE_A0	180		//180
#define		BALANCE_B2	10   	//10
#define		BALANCE_B1	6	//6
#define		BALANCE_B0	3		//3

#define		METER_WHEEL_D	0.06	//������ֱ����m

#define 	MAX_Roll		0.5		//rad, 30/180*PI = 0.52, �����б��Լ30��
#define 	LIMIT_STEER 	5		//������ת5r����Ӧ���ת��50��

typedef struct{
/*���г�����*/
	double mb, hb, Jb, R;//���������ĸ߶ȡ�ת�����������ְ뾶
	double g;
	double l, lb, lt;//���ηֱ�Ϊǰ���־࣬���������ֵľ��룬ǰ��β�� (m)
	double perimeter;//�������ܳ�
	double angle_G;	//���г����ĵ�ƫ�ƽǶ� (rad)
	double psi_offset;//ƫ���ǵ�ƫ�� (rad)
	
/*λ��*/
	double x, y;
	double xe, ye;
	double vr;		//ǰ���ٶ� (m/s)
	double varphi, dvarphi;	//��б��,��б���ٶ� �������ƣ�
	double psi, dpsi;//ƫ���ǣ� ƫ�����ٶȣ������ƣ�
	double phi, dphi;//ת��ǣ�ת����ٶȣ������ƣ�
	double curr;	//ת��������
	
/*�������*/
	double varphie;
	
/*����ϵ��*/
	double b2, b1, b0;
	double k, a1, a0;
	
/*�������*/
	double new_phi;	//�����ת���
	double last_target, target;//��һ��Ŀ��ת��ǣ���ǰĿ��ת��ǣ����Ȧ��
}Balance_data;

void BalanceInit(void);
void setValuePrint(void);
void stateUpdate(void);
void balanceCalc(void);
void bikeMotion(void);
void dataRecord(void);
void bikebotStop(void);
double solution(double, double, double);
#endif
