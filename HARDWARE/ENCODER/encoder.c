#include "encoder.h"
#include "wheel.h"

void Encoder_Init(void){
	Encoder_TIM_Init(65535-1, 1-1);
}

TIM_HandleTypeDef TIM4_Handler; 
TIM_Encoder_InitTypeDef sEncoderConfig;
//TIM4用于编码器计数
//arr：自动重装值。
//psc：时钟预分频数
void Encoder_TIM_Init(u32 arr,u32 psc)
{  
	TIM4_Handler.Instance = TIM4;
	TIM4_Handler.Init.Prescaler = psc;
	TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM4_Handler.Init.Period = arr;
	TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	//HAL_TIM_Base_Init(&TIM4_Handler);

	sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;  

	sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;   
	sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;  
	sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1; 
	sEncoderConfig.IC1Filter          = 0;

	sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;   
	sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;  
	sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1; 
	sEncoderConfig.IC2Filter          = 0;
	HAL_TIM_Encoder_Init(&TIM4_Handler, &sEncoderConfig);
	
	HAL_TIM_Encoder_Start(&TIM4_Handler, TIM_CHANNEL_ALL);
 }

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(htim->Instance==TIM4)
	{
		__HAL_RCC_TIM4_CLK_ENABLE();            //使能TIM4时钟
		__HAL_RCC_GPIOD_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate=GPIO_AF2_TIM4; 
		
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}
}

u32 read_encoder(void){
	u32 num;
	num = TIM4->CNT;
	TIM4->CNT = 0;
	return num;
}
