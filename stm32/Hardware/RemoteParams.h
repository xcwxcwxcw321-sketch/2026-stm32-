#ifndef __REMOTE_PARAMS_H
#define __REMOTE_PARAMS_H

#include "stm32f10x.h"

void RemoteParams_Init(void);
void RemoteParams_Process(void);
void RemoteParams_SendConfig(void);
void RemoteParams_SendStatus(void);
void RemoteParams_UpdateTelemetry(int16_t actual_left, int16_t actual_right,
                                  int16_t pwm_left, int16_t pwm_right);

#endif
