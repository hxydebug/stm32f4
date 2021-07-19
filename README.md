# unmanned bicycle

#### 介绍
无人自行车实验STM32程序版本存放

#### 软件架构
软件架构说明

#### 使用说明

1.  实验器材:
    	阿波罗STM32F429开发板
2.  程序功能:
    	1、串口1/2/3的接收、发送；
    	2、CAN1实现，移植SCA转向电机
    	3、JY901数据读取
    	4、EIC模型控制
3.  引脚说明：
    串口1（PC或SD模块）	PA9（TX）	PA10（RX）
    串口2（JY901）		PA2（TX）	PA3（RX）
    串口3（F103）		PB10（TX）	PB11（RX）
    CAN1（SCA）		PA11（RX）	PA12（TX）
