#include "stm32f10x.h"

void Encoder_Init(void)
{
    // 开启 TIM3, TIM4 和 GPIO 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 左编码器: PA6(CH1), PA7(CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 右编码器: PB6(CH1), PB7(CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置定时器为编码器模式
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// 获取左轮速度并清零
int16_t Encoder_GetLeft(void) {
    int16_t temp = (int16_t)TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    return -temp; // 实测左轮前进时为负计数，取反后统一为正速度
}

// 获取右轮速度并清零
int16_t Encoder_GetRight(void) {
    int16_t temp = (int16_t)TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    return -temp; // 注意：右电机通常反向安装，需取反保证前进为正
}
