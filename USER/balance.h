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
//IMU测得倾斜角向右为正，转向电机向右为正
#define 	PI			3.1415926
#define		DEG2RAD(x)	((double)(x)*PI/180)
#define		RAD2DEG(x)	((double)(x)*180/PI)

#define		BODY_mb		20
#define		BODY_hb		0.4
#define		BODY_Jb		0.8
#define		BODY_l		0.87
#define		BODY_lb		0.42
#define		BODY_R		0.21
#define		EPSILON		(DEG2RAD(17))		//单位：rad
#define		BODY_lt		(BODY_R*tan(EPSILON))//0.064
#define		GRAVITY		9.8					//g

#define		ANGLE_G		1.0
/*控制器系数*/
#define		BALANCE_K	1.05
#define 	BALANCE_A1	25		//25
#define 	BALANCE_A0	180		//180
#define		BALANCE_B2	10   	//10
#define		BALANCE_B1	6	//6
#define		BALANCE_B0	3		//3

#define		METER_WHEEL_D	0.06	//计米轮直径，m

#define 	MAX_Roll		0.5		//rad, 30/180*PI = 0.52, 最大倾斜角约30°
#define 	LIMIT_STEER 	5		//电机最多转5r，对应最大转向50°

typedef struct{
/*自行车常量*/
	double mb, hb, Jb, R;//质量、重心高度、转动惯量、车轮半径
	double g;
	double l, lb, lt;//依次分别为前后轮距，重心至后轮的距离，前轮尾迹 (m)
	double perimeter;//计米轮周长
	double angle_G;	//自行车重心的偏移角度 (rad)
	double psi_offset;//偏航角的偏差 (rad)
	
/*位姿*/
	double x, y;
	double xe, ye;
	double vr;		//前轮速度 (m/s)
	double varphi, dvarphi;	//倾斜角,倾斜角速度 （弧度制）
	double psi, dpsi;//偏航角， 偏航角速度（弧度制）
	double phi, dphi;//转向角，转向角速度（弧度制）
	double curr;	//转向电机电流
	
/*计算过程*/
	double varphie;
	
/*控制系数*/
	double b2, b1, b0;
	double k, a1, a0;
	
/*控制输出*/
	double new_phi;	//算出的转向角
	double last_target, target;//上一次目标转向角，当前目标转向角（电机圈）
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
