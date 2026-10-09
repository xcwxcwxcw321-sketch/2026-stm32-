#include "stm32f10x.h"
#include "UART.h"
#include "ParamStorage.h"
#include "RemoteParams.h"
#include <string.h>

/* main.c 中的实时参数，按键和串口共用同一份变量 */
extern uint8_t car_state;
extern uint8_t active_problem;
extern uint32_t encoder_total_count;
extern uint32_t run_elapsed_ticks;
extern float V_Kp;
extern float V_Ki;
extern float T_Kp;
extern float T_Kd;
extern int Problem1_Max_Speed;
extern int Problem2_Max_Speed;
extern int AB_Max_Speed;
extern uint32_t Problem1_Stop_Count;
extern uint32_t Problem2_Stop_Count;
extern uint32_t AB_Stop_Count;
extern int Current_Base_Speed;
extern int Track_Center;
extern int Track_Dead_Zone;
extern int Max_Turn;

#define REMOTE_STATE_STOP  0
#define REMOTE_LINE_SIZE   96

static int16_t Remote_ActualLeft = 0;
static int16_t Remote_ActualRight = 0;
static int16_t Remote_PwmLeft = 0;
static int16_t Remote_PwmRight = 0;

static int16_t Remote_Scale100(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value * 100.0f + 0.5f);
    }
    return (int16_t)(value * 100.0f - 0.5f);
}

static void Remote_ResetDefaults(void)
{
    V_Kp = 1.50f;
    V_Ki = 0.30f;
    T_Kp = 0.50f;
    T_Kd = 5.50f;
    Problem1_Max_Speed = 30;
    Problem2_Max_Speed = 25;
    AB_Max_Speed = 20;
    Problem1_Stop_Count = 43000UL;
    Problem2_Stop_Count = 43000UL;
    AB_Stop_Count = 43000UL;
    Track_Center = 72;
    Track_Dead_Zone = 2;
    Max_Turn = 12;
}

static void Remote_BuildBlock(RobotParamBlock *params)
{
    params->magic = PARAM_MAGIC;
    params->version = PARAM_VERSION;
    params->size = sizeof(RobotParamBlock);
    params->velocity_kp_x100 = Remote_Scale100(V_Kp);
    params->velocity_ki_x100 = Remote_Scale100(V_Ki);
    params->track_kp_x100 = Remote_Scale100(T_Kp);
    params->track_kd_x100 = Remote_Scale100(T_Kd);
    params->problem1_max_speed = (int16_t)Problem1_Max_Speed;
    params->problem2_max_speed = (int16_t)Problem2_Max_Speed;
    params->ab_max_speed = (int16_t)AB_Max_Speed;
    params->track_center = (int16_t)Track_Center;
    params->track_dead_zone = (int16_t)Track_Dead_Zone;
    params->max_turn = (int16_t)Max_Turn;
    params->problem1_stop_count = Problem1_Stop_Count;
    params->problem2_stop_count = Problem2_Stop_Count;
    params->ab_stop_count = AB_Stop_Count;
    params->crc32 = 0;
}

static uint8_t Remote_BlockValuesValid(const RobotParamBlock *params)
{
    if (params->velocity_kp_x100 < 0 || params->velocity_kp_x100 > 1000) return 0;
    if (params->velocity_ki_x100 < 0 || params->velocity_ki_x100 > 1000) return 0;
    if (params->track_kp_x100 < 0 || params->track_kp_x100 > 300) return 0;
    if (params->track_kd_x100 < 0 || params->track_kd_x100 > 900) return 0;
    if (params->problem1_max_speed < 10 || params->problem1_max_speed > 40) return 0;
    if (params->problem2_max_speed < 8 || params->problem2_max_speed > 35) return 0;
    if (params->ab_max_speed < 6 || params->ab_max_speed > 30) return 0;
    if (params->problem1_stop_count > 999999UL) return 0;
    if (params->problem2_stop_count > 999999UL) return 0;
    if (params->ab_stop_count > 999999UL) return 0;
    if (params->track_center < 10 || params->track_center > 90) return 0;
    if (params->track_dead_zone < 0 || params->track_dead_zone > 20) return 0;
    if (params->max_turn < 1 || params->max_turn > 30) return 0;
    return 1;
}

static void Remote_ApplyBlock(const RobotParamBlock *params)
{
    V_Kp = params->velocity_kp_x100 / 100.0f;
    V_Ki = params->velocity_ki_x100 / 100.0f;
    T_Kp = params->track_kp_x100 / 100.0f;
    T_Kd = params->track_kd_x100 / 100.0f;
    Problem1_Max_Speed = params->problem1_max_speed;
    Problem2_Max_Speed = params->problem2_max_speed;
    AB_Max_Speed = params->ab_max_speed;
    Problem1_Stop_Count = params->problem1_stop_count;
    Problem2_Stop_Count = params->problem2_stop_count;
    AB_Stop_Count = params->ab_stop_count;
    Track_Center = params->track_center;
    Track_Dead_Zone = params->track_dead_zone;
    Max_Turn = params->max_turn;
}

static uint8_t Remote_ParseInt(const char *text, int32_t *value)
{
    uint32_t result = 0;
    uint8_t negative = 0;

    if (text == 0 || value == 0 || *text == '\0') return 0;
    if (*text == '-')
    {
        negative = 1;
        text++;
    }
    if (*text == '\0') return 0;

    while (*text != '\0')
    {
        if (*text < '0' || *text > '9') return 0;
        if (result > 214748364U) return 0;
        result = result * 10U + (uint32_t)(*text - '0');
        text++;
    }

    if (result > 2147483647U) return 0;
    *value = negative ? -(int32_t)result : (int32_t)result;
    return 1;
}

static uint8_t Remote_ParseFixed100(const char *text, int32_t *value_x100)
{
    uint32_t whole = 0;
    uint32_t fraction = 0;
    uint8_t fraction_digits = 0;
    uint8_t negative = 0;
    uint8_t has_digit = 0;

    if (text == 0 || value_x100 == 0 || *text == '\0') return 0;
    if (*text == '-')
    {
        negative = 1;
        text++;
    }

    while (*text >= '0' && *text <= '9')
    {
        has_digit = 1;
        whole = whole * 10U + (uint32_t)(*text - '0');
        if (whole > 100000U) return 0;
        text++;
    }

    if (*text == '.')
    {
        text++;
        while (*text >= '0' && *text <= '9')
        {
            if (fraction_digits >= 2) return 0;
            fraction = fraction * 10U + (uint32_t)(*text - '0');
            fraction_digits++;
            has_digit = 1;
            text++;
        }
    }

    if (!has_digit || *text != '\0') return 0;
    if (fraction_digits == 0) fraction = 0;
    if (fraction_digits == 1) fraction *= 10U;
    *value_x100 = (int32_t)(whole * 100U + fraction);
    if (negative) *value_x100 = -*value_x100;
    return 1;
}

static uint8_t Remote_NameEquals(const char *name, uint16_t name_length, const char *expected)
{
    uint16_t expected_length = (uint16_t)strlen(expected);
    return (name_length == expected_length && strncmp(name, expected, name_length) == 0);
}

static uint8_t Remote_SetParameter(const char *name, uint16_t name_length, const char *value_text)
{
    int32_t value;

    if (Remote_NameEquals(name, name_length, "VKP"))
    {
        if (!Remote_ParseFixed100(value_text, &value) || value < 0 || value > 1000) return 0;
        V_Kp = value / 100.0f;
    }
    else if (Remote_NameEquals(name, name_length, "VKI"))
    {
        if (!Remote_ParseFixed100(value_text, &value) || value < 0 || value > 1000) return 0;
        V_Ki = value / 100.0f;
    }
    else if (Remote_NameEquals(name, name_length, "TKP"))
    {
        if (!Remote_ParseFixed100(value_text, &value) || value < 0 || value > 300) return 0;
        T_Kp = value / 100.0f;
    }
    else if (Remote_NameEquals(name, name_length, "TKD"))
    {
        if (!Remote_ParseFixed100(value_text, &value) || value < 0 || value > 900) return 0;
        T_Kd = value / 100.0f;
    }
    else
    {
        if (!Remote_ParseInt(value_text, &value)) return 0;

        if (Remote_NameEquals(name, name_length, "P1_MAX"))
        {
            if (value < 10 || value > 40) return 0;
            Problem1_Max_Speed = (int)value;
        }
        else if (Remote_NameEquals(name, name_length, "P2_MAX"))
        {
            if (value < 8 || value > 35) return 0;
            Problem2_Max_Speed = (int)value;
        }
        else if (Remote_NameEquals(name, name_length, "AB_MAX"))
        {
            if (value < 6 || value > 30) return 0;
            AB_Max_Speed = (int)value;
        }
        else if (Remote_NameEquals(name, name_length, "P1_STOP"))
        {
            if (value < 0 || value > 999999) return 0;
            Problem1_Stop_Count = (uint32_t)value;
        }
        else if (Remote_NameEquals(name, name_length, "P2_STOP"))
        {
            if (value < 0 || value > 999999) return 0;
            Problem2_Stop_Count = (uint32_t)value;
        }
        else if (Remote_NameEquals(name, name_length, "AB_STOP"))
        {
            if (value < 0 || value > 999999) return 0;
            AB_Stop_Count = (uint32_t)value;
        }
        else if (Remote_NameEquals(name, name_length, "CENTER"))
        {
            if (value < 10 || value > 90) return 0;
            Track_Center = (int)value;
        }
        else if (Remote_NameEquals(name, name_length, "DEAD"))
        {
            if (value < 0 || value > 20) return 0;
            Track_Dead_Zone = (int)value;
        }
        else if (Remote_NameEquals(name, name_length, "MAX_TURN"))
        {
            if (value < 1 || value > 30) return 0;
            Max_Turn = (int)value;
        }
        else
        {
            return 0;
        }
    }

    /* 运行中下调当前模式的最高速度时，立即限制目标速度 */
    if (active_problem == 1 && Current_Base_Speed > Problem1_Max_Speed)
        Current_Base_Speed = Problem1_Max_Speed;
    else if (active_problem == 2 && Current_Base_Speed > Problem2_Max_Speed)
        Current_Base_Speed = Problem2_Max_Speed;
    else if (active_problem == 3 && Current_Base_Speed > AB_Max_Speed)
        Current_Base_Speed = AB_Max_Speed;

    return 1;
}

void RemoteParams_SendConfig(void)
{
    UART_SendString("CFG,VKP=");
    UART_SendFixed100(Remote_Scale100(V_Kp));
    UART_SendString(",VKI=");
    UART_SendFixed100(Remote_Scale100(V_Ki));
    UART_SendString(",TKP=");
    UART_SendFixed100(Remote_Scale100(T_Kp));
    UART_SendString(",TKD=");
    UART_SendFixed100(Remote_Scale100(T_Kd));
    UART_SendString(",P1_MAX=");
    UART_SendInt32(Problem1_Max_Speed);
    UART_SendString(",P2_MAX=");
    UART_SendInt32(Problem2_Max_Speed);
    UART_SendString(",AB_MAX=");
    UART_SendInt32(AB_Max_Speed);
    UART_SendString(",P1_STOP=");
    UART_SendUInt32(Problem1_Stop_Count);
    UART_SendString(",P2_STOP=");
    UART_SendUInt32(Problem2_Stop_Count);
    UART_SendString(",AB_STOP=");
    UART_SendUInt32(AB_Stop_Count);
    UART_SendString(",CENTER=");
    UART_SendInt32(Track_Center);
    UART_SendString(",DEAD=");
    UART_SendInt32(Track_Dead_Zone);
    UART_SendString(",MAX_TURN=");
    UART_SendInt32(Max_Turn);
    UART_SendString("\n");
}

void RemoteParams_SendStatus(void)
{
    UART_SendString("STAT,STATE=");
    UART_SendString(car_state == REMOTE_STATE_STOP ? "STOP" : "RUN");
    UART_SendString(",MODE=");
    if (active_problem == 1) UART_SendString("P1");
    else if (active_problem == 2) UART_SendString("P2");
    else UART_SendString("AB");
    UART_SendString(",BASE=");
    UART_SendInt32(Current_Base_Speed);
    UART_SendString(",ACT_L=");
    UART_SendInt32(Remote_ActualLeft);
    UART_SendString(",ACT_R=");
    UART_SendInt32(Remote_ActualRight);
    UART_SendString(",PWM_L=");
    UART_SendInt32(Remote_PwmLeft);
    UART_SendString(",PWM_R=");
    UART_SendInt32(Remote_PwmRight);
    UART_SendString(",COUNT=");
    UART_SendUInt32(encoder_total_count);
    UART_SendString(",TIME_TICKS=");
    UART_SendUInt32(run_elapsed_ticks);
    UART_SendString("\n");
}

void RemoteParams_UpdateTelemetry(int16_t actual_left, int16_t actual_right,
                                  int16_t pwm_left, int16_t pwm_right)
{
    Remote_ActualLeft = actual_left;
    Remote_ActualRight = actual_right;
    Remote_PwmLeft = pwm_left;
    Remote_PwmRight = pwm_right;
}

void RemoteParams_Init(void)
{
    RobotParamBlock params;
    uint8_t loaded = 0;

    UART_Init();
    if (ParamStorage_Load(&params) && Remote_BlockValuesValid(&params))
    {
        Remote_ApplyBlock(&params);
        loaded = 1;
    }

    UART_SendString("READY,LOAD=");
    UART_SendString(loaded ? "FLASH\n" : "DEFAULT\n");
    RemoteParams_SendConfig();
}

void RemoteParams_Process(void)
{
    char line[REMOTE_LINE_SIZE];
    char *separator;
    RobotParamBlock params;

    if (!UART_ReadLine(line, sizeof(line))) return;

    if (strcmp(line, "GET") == 0)
    {
        RemoteParams_SendConfig();
    }
    else if (strcmp(line, "STATUS") == 0)
    {
        RemoteParams_SendStatus();
    }
    else if (strcmp(line, "SAVE") == 0)
    {
        if (car_state != REMOTE_STATE_STOP)
        {
            UART_SendString("ERR,BUSY\n");
        }
        else
        {
            Remote_BuildBlock(&params);
            UART_SendString(ParamStorage_Save(&params) ? "OK,SAVED\n" : "ERR,FLASH\n");
        }
    }
    else if (strcmp(line, "LOAD") == 0)
    {
        if (car_state != REMOTE_STATE_STOP)
        {
            UART_SendString("ERR,BUSY\n");
        }
        else if (ParamStorage_Load(&params) && Remote_BlockValuesValid(&params))
        {
            Remote_ApplyBlock(&params);
            UART_SendString("OK,LOADED\n");
            RemoteParams_SendConfig();
        }
        else
        {
            UART_SendString("ERR,NO_SAVED_CONFIG\n");
        }
    }
    else if (strcmp(line, "DEFAULT") == 0)
    {
        if (car_state != REMOTE_STATE_STOP)
        {
            UART_SendString("ERR,BUSY\n");
        }
        else
        {
            Remote_ResetDefaults();
            UART_SendString("OK,DEFAULT\n");
            RemoteParams_SendConfig();
        }
    }
    else if (strcmp(line, "HELP") == 0)
    {
        UART_SendString("HELP,GET|STATUS|SET,NAME,VALUE|SAVE|LOAD|DEFAULT\n");
    }
    else if (strncmp(line, "SET,", 4) == 0)
    {
        separator = strchr(line + 4, ',');
        if (separator == 0 || separator[1] == '\0')
        {
            UART_SendString("ERR,FORMAT\n");
        }
        else if (Remote_SetParameter(line + 4, (uint16_t)(separator - (line + 4)), separator + 1))
        {
            UART_SendString("OK,SET\n");
            RemoteParams_SendConfig();
        }
        else
        {
            UART_SendString("ERR,NAME_OR_RANGE\n");
        }
    }
    else
    {
        UART_SendString("ERR,UNKNOWN\n");
    }
}
