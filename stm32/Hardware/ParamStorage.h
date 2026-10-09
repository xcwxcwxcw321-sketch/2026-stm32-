#ifndef __PARAM_STORAGE_H
#define __PARAM_STORAGE_H

#include "stm32f10x.h"

#define PARAM_FLASH_ADDRESS  ((uint32_t)0x0800FC00)
#define PARAM_MAGIC          ((uint32_t)0x524F424F)
#define PARAM_VERSION        ((uint16_t)1)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int16_t velocity_kp_x100;
    int16_t velocity_ki_x100;
    int16_t track_kp_x100;
    int16_t track_kd_x100;
    int16_t problem1_max_speed;
    int16_t problem2_max_speed;
    int16_t ab_max_speed;
    int16_t track_center;
    int16_t track_dead_zone;
    int16_t max_turn;
    uint32_t problem1_stop_count;
    uint32_t problem2_stop_count;
    uint32_t ab_stop_count;
    uint32_t crc32;
} RobotParamBlock;

uint8_t ParamStorage_Load(RobotParamBlock *params);
uint8_t ParamStorage_Save(const RobotParamBlock *params);

#endif
