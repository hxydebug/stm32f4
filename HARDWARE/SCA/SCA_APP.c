/**
  ******************************************************************************
  * @文	件 ： SCA_APP.c
  * @作	者 ： 王增豪
  * @版	本 ： V1.0
  * @日	期 ： 2020.6.28
  * @摘	要 ： SCA 应用程序
  ******************************************************************************/ 
/* Update log --------------------------------------------------------------------*/
//V1.1.0 2020.6.28  完善SCA调试程序

/* Includes ----------------------------------------------------------------------*/
#include "can.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "SCA_APP.h"
#include "SCA_API.h"

/* Variable defines --------------------------------------------------------------*/
SCA_Handler_t* pSCA_ID1 = NULL;		//读写指针，可用于获取执行器参数或调用FAST型函数

/* CAN端口信息定义，用于绑定SCA句柄，实现多端口控制。移植时根据实际数量定义 */
CAN_Handler_t CAN_Port1;

/* Funcation defines -------------------------------------------------------------*/

/**
  * @功	能	控制器初始化
  * @参	数	无
  * @返	回	无
  */
void SCA_Init()
{
	/* 初始化CAN端口参数 */
	CAN_Port1.CanPort = 1;			//标记端口号
	CAN_Port1.Retry = 2;			//失败重发次数
	CAN_Port1.Send = CAN1_Send_Msg;	//CAN1端口发送函数
	
	
	/* 装载执行器的ID与所使用的CAN端口号 */
	setupActuators(SCA_ID, &CAN_Port1);	//ID1 绑定CAN1
	
	/* 获取ID1和2的参数句柄 */
	pSCA_ID1 = getInstance(SCA_ID);
	
	/* 启动所有执行器 */
	enableAllActuators();
}

/**
  * @功	能	电机参数设置
  * @参	数	无
  * @返	回	无
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
  * @功	能	位置归零
  * @参	数	无
  * @返	回	无
  */
void SCA_Homing()
{
	/* 未开机直接退出 */
	if(pSCA_ID1->Power_State == Actr_Disable)	{
		//printf("\r\n未开机\r\n");
		return;
	}
	
	/* 切换执行器操作模式到梯形位置模式 */
	activateActuatorMode(SCA_ID,SCA_Profile_Position_Mode,Block);
//	activateActuatorMode(SCA_ID,SCA_Position_Mode,Block);
	/* 归零 执行器 */
	setPosition(SCA_ID,0);
	
	/* 等待归零成功 */
	do
	{
		getPosition(SCA_ID,Unblock);
		delay_ms(100);
	}
	while((pSCA_ID1->Position_Real > 0.1f)||(pSCA_ID1->Position_Real < -0.1f));
}

#if DEMO
/**
  * @功	能	在CAN总线上查找存在的SCA，并打印找到的ID
  * @参	数	无
  * @返	回	无
  * @注	意	每台执行器都有自己的ID，若初次使用不知道
  *			对应的ID，可用此函数查找。此功能需要开SCA_DEBUGER
  */
void SCA_Lookup()
{
	/* 初始化CAN端口参数 */
	CAN_Port1.CanPort = 1;			//标记端口号
	CAN_Port1.Retry = 2;			//失败重发次数
	CAN_Port1.Send = CAN1_Send_Msg;	//CAN1端口发送函数
	
	
	/* 调用函数查找对应总线上存在的ID */
	lookupActuators(&CAN_Port1);	//轮询CAN1总线
}

/**
  * @功	能	正反转切换两次
  * @参	数	无
  * @返	回	无
  */
void SCA_Exp1()
{
	/* 未开机直接退出 */
	if(pSCA_ID1->Power_State == Actr_Disable)	return;
	
	/* 归零 */
	SCA_Homing();
	
	/* 开启正反转 */
	setPosition(SCA_ID,3);			//普通函数以ID调用执行器
	delay_ms(1000);
	
	setPosition(SCA_ID,-3);
	delay_ms(1000);
	
	setPosition(SCA_ID,3);
	delay_ms(1000);
	
	setPosition(SCA_ID,-3);
	delay_ms(1000);
} 

/**
  * @功	能	高低速切换
  * @参	数	无
  * @返	回	无
  */
void SCA_Exp2()
{
	/* 未开机直接退出 */
	if(pSCA_ID1->Power_State == Actr_Disable)	return;
	
	/* 切换执行器操作模式到梯形速度模式 */
	activateActuatorMode(SCA_ID,SCA_Profile_Velocity_Mode,Block);
	
	/* 高低速切换 */
	setVelocity(SCA_ID,300);
	delay_ms(1000);
	
	setVelocity(SCA_ID,600);
	delay_ms(1000);
	
	setVelocity(SCA_ID,300);
	delay_ms(1000);
	
	setVelocity(SCA_ID,600);
	delay_ms(1000);
	
	/* 停止 */
	setVelocity(SCA_ID,0);
}

/**
  * @功	能	零位设置
  * @参	数	无
  * @返	回	无
  */
void homePositionReset()
{
	getPosition(SCA_ID, Block);
	setHomingPosition(SCA_ID, pSCA_ID1->Position_Real, Block);
}


/**
  * @功	能	复位控制器
  * @参	数	无
  * @返	回	无
  */
void resetAll()
{
	resetController(SCA_ID);
}

////////////////////////////////////////////////////////////////////////////////////

/**
  * @功	能	串口打印提示信息
  * @参	数	无
  * @返	回	无
  */
void Log()
{
	printf("\r\n欢迎使用 INNFOS SCA 驱动测试！ \r\n");
	printf("详细通信协议参见 INNFOS WIKI！ \r\n");
	printf("发送 1 轮询总线上的执行器ID！ \r\n");
	printf("发送 2 使用默认ID初始化SCA控制器！ \r\n");
	printf("发送 3 进入位置归零测试程序！ \r\n");
	printf("发送 4 进入正反转测试程序！ \r\n");
	printf("发送 5 进入高低速测试程序！ \r\n");
	printf("发送 6 将执行器关机！ \r\n");
	printf("发送 7 设置当前位置为零位！ \r\n");
	printf("发送 8 复位！\r\n");
}

uint8_t cmd = 0;					//外部控制命令

void CMD_Usart(void)
{
	if(cmd == 0)	
		cmd = USART1->DR;
	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
}
/**
  * @功	能	串口命令处理函数
  * @参	数	cmd：接收到的指令
  * @返	回	无
  */
void CMD_Handler(uint8_t cmd)
{
	switch(cmd)
	{
		case 1:
			printf("\r\n执行轮询程序！\r\n");
		
			/* 调用轮询程序 */
			SCA_Lookup();
		
			printf("轮询结束！\r\n");
		break;
		
		case 2:
			printf("\r\nSCA初始化！\r\n");
		
			/* 调用初始化程序 */
			SCA_Init();
		
			/* 等待执行器稳定 */
			delay_ms(500);
		
			printf("SCA初始化结束！\r\n");
			break;
		
		case 3:
			printf("\r\n进入位置归零测试！\r\n");
		
			/* 调用测试程序 位置归零 */
			SCA_Homing();
		
			printf("位置归零测试结束！\r\n");
			break;
			
		case 4:
			printf("\r\n进入正反转切换测试！\r\n");
		
			/* 调用测试程序 正反转切换 */
			SCA_Exp1();
		
			printf("正反转切换测试结束！\r\n");
			break;
		
		case 5:
			printf("\r\n进入高低速切换测试！\r\n");
			
			/* 调用测试程序 高低速切换 */
			SCA_Exp2();
		
			printf("高低速切换测试结束！\r\n");
			break;
		
		case 6:
			printf("\r\n执行器关机！\r\n");
			
			/* 关闭所有执行器 */
			disableAllActuators();
		
			printf("执行器关机结束！\r\n");
			break;
		
		case 7:
			printf("\r\n执行器零位设置！\r\n");
			
			/* 所有执行器设置当前位置为零位 */
			homePositionReset();;
		
			printf("执行器零位设置结束！\r\n");
			break;
				
		case 8:
			printf("\r\n复位\r\n");
			resetAll();
			printf("\r\n复位结束\r\n");
		
		default:
			Log();
			break;
	}
}
#endif
