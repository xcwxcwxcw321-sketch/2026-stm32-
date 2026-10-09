#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare3(uint16_t Compare); // 控制电机A的PWM
void PWM_SetCompare4(uint16_t Compare); // 控制电机B的PWM

#endif
