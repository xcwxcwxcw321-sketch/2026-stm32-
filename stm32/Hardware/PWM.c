#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	// 开启 TIM2 和 GPIOA 的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// 配置 PA2(PWMA) 和 PA3(PWMB) 为复用推挽输出
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 配置 TIM2 内部时钟
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1 ;          // ARR (100的占空比调节范围)
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;         // PSC (频率 = 72M/36/100 = 20kHz)
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	// 配置输出比较参数
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);                
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0 ;                    
	
	// 初始化 TIM2 的通道 3 和通道 4
	TIM_OC3Init(TIM2, &TIM_OCInitStructure); // 对应 PA2
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); // 对应 PA3

	TIM_Cmd(TIM2, ENABLE);
}

void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare3(TIM2, Compare);
}

void PWM_SetCompare4(uint16_t Compare)
{
	TIM_SetCompare4(TIM2, Compare);
}
