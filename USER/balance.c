#include "balance.h"
#include "math.h"
#include "Nuci7.h"

Balance_data bikebot;
void TIM3_IRQHandler(void){//20ms中断
	if(TIM3->SR&0X0001)//溢出中断
	{
//		stateUpdate();
//		balanceCalc();
//		bikeMotion();
//		dataRecord();
		Bikedrive();
		Nuci7Write();
		TIM3->SR&=~(1<<0);//清除中断标志位 	   
	}
}

void BalanceInit(void)
{
	/*自行车常量*/
	bikebot.mb = BODY_mb;	bikebot.hb = BODY_hb;
	bikebot.Jb = BODY_Jb;	bikebot.R  = BODY_R;
	bikebot.g  = GRAVITY;	bikebot.l  = BODY_l;
	bikebot.lb = BODY_lb;	bikebot.lt = BODY_lt;
	bikebot.perimeter = PI*METER_WHEEL_D;//计米轮周长 （米）
	
	bikebot.angle_G = DEG2RAD(ANGLE_G);
	bikebot.psi_offset = (double)stcAngle.Angle[2]/32768*PI;
	
	/*控制器系数*/
	bikebot.b2 = BALANCE_B2;	bikebot.b1 = BALANCE_B1;	bikebot.b0 = BALANCE_B0;	
	bikebot.a1 = BALANCE_A1;	bikebot.a0 = BALANCE_A0;
	bikebot.k  = BALANCE_K;
	
	/*其他初值*/
	bikebot.vr = VR;
	bikebot.psi = 0;
	bikebot.x = 0;
	bikebot.y = 0;
	bikebot.last_target = 0;
}

void setValuePrint(void){
	printf("B2:%.1f B1:%.1f B0:%.1f A1:%.1f A0:%.1f\r\n", bikebot.b2, bikebot.b1, bikebot.b0, bikebot.a1, bikebot.a0);
}

void stateUpdate(void){
	/*位姿更新*/
	bikebot.varphi = (double)stcAngle.Angle[0]/32768*PI - bikebot.angle_G;  //  单位：rad
	bikebot.dvarphi = (double)stcGyro.w[0]*8.725/32768;			// 500/180*PI=8.725   单位： rad/s
	
	requestCVPValueFast(pSCA_ID1,Block);
	bikebot.phi = (double)(pSCA_ID1->Position_Real)*(-0.174533);//rad。10/180*PI=0.174533
	bikebot.dphi = (double)(pSCA_ID1->Velocity_Real)*(-0.167);//rad/s
	bikebot.curr = (double)(pSCA_ID1->Current_Real);
	
#if	UPDATE_V		//要不要更新速度
	double vr = (double)read_encoder() * bikebot.perimeter/16;// ÷(0.02*800)。0.02表示中断时间；200P/R，四分频后800P/R
	if(vr > 10) vr = VR;
//	if(vr < 2 && vr > 0.2){
////		bikebot.vr = vr * cos(bikebot.phi);
//		bikebot.vr = vr;
//	}
	bikebot.vr = vr * cos(bikebot.phi);
#endif
/*	IMU读取  */
//	bikebot.dpsi = (double)stcGyro.w[2]*8.725/32768;
	bikebot.psi = (double)stcAngle.Angle[2]/32768*PI - bikebot.psi_offset;//IMU读取的偏航角
	
/*	计算得到  */
	bikebot.dpsi = 1.01*bikebot.vr*tan(bikebot.phi)/cos(bikebot.varphi);//cos（epsilon）/l = 1.01
//	bikebot.psi += bikebot.dpsi * 0.01;//累加得到的偏航角

	bikebot.x += bikebot.vr * cos(bikebot.psi) * 0.02;
	bikebot.y += bikebot.vr * sin(bikebot.psi) * 0.02;
	
	if(fabs(bikebot.varphi) > MAX_Roll) bikebotStop();
}

volatile int i = 0;		int flag = 0;
int cirTime = -1;		int cosTime = -1;

double x_offset = 0;	double y_offset = 0;
double xe = 0; 			double ye = 0;
double dxe = 0;   	 	double dye = 0;
double ddxe = 0;   	 	double ddye = 0;
double dddxe = 0;   	double dddye = 0;
double ddddxe = 0;  	double ddddye = 0;
double dddddxe = 0; 	double dddddye = 0;
double cr = 2.3; 		double new_Wt = - PI/2;

void balanceCalc(void){	
	/*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	
	/*期望轨迹*/
	i = i+1;
	if(i == cirTime) {
		flag = 1;	new_Wt = - PI/2;
		x_offset = x;
		y_offset = y + cr;
	}else if(i == cosTime) {
		flag = 2;
		x_offset = x;
		y_offset = y;
	}
	
	double dt =0.02;	double ve = vr; 
	
	if(flag == 0){//直线轨迹  
		xe += ve*dt;    ye = 0;
		dxe = ve;   	dye = 0;
	}else if(flag == 1){//切换到圆形轨迹
//		cr += 0.0002;
		double w=ve/cr; 
		new_Wt += w*dt;
		xe = x_offset + cr * cos(new_Wt); 	ye =  y_offset + cr*sin(new_Wt);
		dxe = -cr * sin(new_Wt) * w;		dye = cr * cos(new_Wt) * w;
//		ddxe = -dye * w;					ddye = dxe * w;
//		dddxe = -ddye * w;					dddye = ddxe * w;
//		ddddxe = -dddye * w;				ddddye = dddxe * w;
//		dddddxe = -ddddye * w;				dddddye = ddddxe * w;
	}else if(flag == 2){//切换到余弦轨迹
		xe = x;		dxe = dx;
		ye = y_offset - 1.5 + 1.5*cos((x-x_offset)/3.5);	dye = -1.5/3.5*sin((x-x_offset)/3.5)*dx;
	}
	//8字形
//	  double elx = 6; double wx= PI/16; 	double ely = 3; double wy = PI/8;
//    double xe = elx*sin(wx*i*dt);  		double ye = ely*sin(wy*i*dt); 
//    double dxe = elx*cos(wx*i*dt)*wx;  	double dye = ely*cos(wy*i*dt)*wy; 
//    double ddxe = -xe*wx*wx;  			double ddye = -ye*wy*wy; 
//    double dddxe = -dxe*wx*wx; 			double dddye = -dye*wy*wy; 
//    double ddddxe = -ddxe*wx*wx; 		double ddddye = -ddye*wy*wy;
//    double dddddxe = -dddxe*wx*wx;  	double dddddye = -dddye*wy*wy;
	
	bikebot.xe = xe;		bikebot.ye = ye;//记录期望轨迹
	/*External*/
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);	
	
//	double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	double ur = 0;
	
	double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2;
	double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
            + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2;
    double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1
            - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	/*Internal*/
	double varphie = solution(vr, dpsi, u_psi);//牛顿迭代法求方程根
	bikebot.varphie = varphie;
	double cosvare = cos(varphie);
	double sinvare = sin(varphie);
	
	double M1 = 0.4*cosvare*dpsi*dpsi+9.8/cosvare/cosvare;//h=0.4, g=9.8
	//2h=0.8, g*lt*lb*cos(epsilon)/h=0.6318, lb=0.42
    double M2 = (u_psi*vr+dpsi*dvr+0.8*dpsi*u_psi*sinvare+0.6318*(u_psi*vr-dpsi*dvr)/vr/vr+0.42*du_psi);
	
	double dvarphie = -M2/M1;
	
	double dM1 = (0.8*dpsi*u_psi*cosvare-0.4*dpsi*dpsi*sinvare+19.6/cosvare/cosvare*sinvare/cosvare)*dvarphie;//h=0.4, 2g=19.6
	double dM2 = du_psi*vr+2*u_psi*dvr+dpsi*ur+0.8*(u_psi*u_psi*sinvare+dpsi*du_psi*sinvare+dpsi*u_psi*cosvare*dvarphie)
            + 0.6318 * (du_psi*vr-2*u_psi*dvr-dpsi*ur+2*dpsi*dvr*dvr/vr)/vr/vr + 0.42*ddu_psi;
    
	double ddvarphie = (dM1*M2/M1 - dM2)/M1;
	
	
	double v_psi_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
	double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);
	double f_varphi = (8*vr + 3.2*sinvar*dpsi)*cosvar*dpsi //mh = 8, mh*h=3.2
				+78.4*sinvar + 5.0543*dpsi*cosvar/vr; //mb*hb*g=78.4, mb*g*lt*lb*cos(epsilon)=5.0543
	double g_psi = 3.36*cosvar; //mb*hb*lb=3.36
	double u_psi_int = (-f_varphi + 4.0*v_psi_int)/g_psi;//Jt = mh^2+Jb = 4.0
	
	/*计算转向角*/
	bikebot.new_phi = atan((dpsi+u_psi_int*dt)*0.9098*cosvar/vr)*bikebot.k;//dt=0.02， l/cos(epsilon)=0.9098
	double target = -bikebot.new_phi*5.73;//5.73=180/PI/360*36
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
}

void bikeMotion(void)
{
	double limit = 0.1;
	double target = bikebot.target;
	double last = bikebot.last_target;
	double phi_cmd = 0;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	setPositionFast(pSCA_ID1, (float)phi_cmd);
	bikebot.last_target = phi_cmd;
}

void dataRecord(void){
//	printf("%.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f\r\n",bikebot.varphi, bikebot.dvarphi, 
//			bikebot.varphie, bikebot.new_phi, bikebot.phi, bikebot.vr, bikebot.x, bikebot.y, bikebot.psi, bikebot.xe, bikebot.ye);
	printf("%.3f, %.3f, %.3f, %.3f\r\n",bikebot.x, bikebot.y,bikebot.xe, bikebot.ye);
}

void bikebotStop(void){
	HAL_TIM_Base_Stop_IT(&TIM3_Handler);
	wheelSet(0);
//	legDown();
	delay_ms(600);	
	disableAllActuators();
	printf("\r\n\r\n\r\n");
	while(1);
}

double solution(double vr, double dpsi, double u_psi){
	int cnt = 0;
	double init = 0;
	double res = 0;
	double fun, dfun;
	double tmp;
	double second = (vr + GRAVITY*BODY_lt*BODY_lb*cos(EPSILON)/BODY_hb/vr)*dpsi + BODY_lb * u_psi;
	do{
		init = res;
		fun = BODY_hb*dpsi*dpsi*sin(init)+GRAVITY*tan(init)+ second;
		tmp = cos(init);
		dfun = BODY_hb*dpsi*dpsi*tmp+GRAVITY/tmp/tmp;
		res = init - fun/dfun;
		if(++cnt > 10){//至多迭代10次
			break;
		}
	}while(fabs(res-init)>1e-3);
	return res;
}
