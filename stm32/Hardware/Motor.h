#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void Motor_SetSpeed_A(int16_t Speed); // 控制电机A (例如左轮)
void Motor_SetSpeed_B(int16_t Speed); // 控制电机B (例如右轮)

#endif
