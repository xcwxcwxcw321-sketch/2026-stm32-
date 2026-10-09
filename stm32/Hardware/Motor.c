#include "stm32f10x.h"
#include "PWM.h"
 
void Motor_Init(void)
{
    // 开启 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // 使用 PB12~PB15 控制方向
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    PWM_Init(); // 初始化原本 PA2, PA3 的 PWM 输出
}

void Motor_SetSpeed_A(int16_t Speed) // 左电机
{
    if(Speed >= 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompare3(Speed);
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompare3(-Speed);
    }
}

void Motor_SetSpeed_B(int16_t Speed) // 右电机
{
    if(Speed >= 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompare4(Speed);
    } else {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompare4(-Speed);
    }
}
