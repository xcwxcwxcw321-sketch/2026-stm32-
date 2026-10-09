#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "LineTracker.h"
#include "Encoder.h"  // 确保你已经创建了这个硬件驱动
#include "RemoteParams.h"

// ================== 按键控制相关 ==================
#define KEY_GPIO_PORT    GPIOC
#define KEY_GPIO_PIN     GPIO_Pin_13
#define COUNT_KEY_PIN    GPIO_Pin_14
#define KEY_GPIO_CLK     RCC_APB2Periph_GPIOC
#define UP_KEY_PORT       GPIOA
#define UP_KEY_PIN        GPIO_Pin_1
#define UP_KEY_CLK        RCC_APB2Periph_GPIOA
#define DOWN_KEY_PORT     GPIOB
#define DOWN_KEY_PIN      GPIO_Pin_1
#define DOWN_KEY_CLK      RCC_APB2Periph_GPIOB
#define PREV_KEY_PORT     GPIOA
#define PREV_KEY_PIN      GPIO_Pin_12
#define NEXT_KEY_PORT     GPIOB
#define NEXT_KEY_PIN      GPIO_Pin_3

// 发车状态信号：行驶时 PA0 为高电平，停车时为低电平
#define RUN_SIGNAL_PORT    GPIOA
#define RUN_SIGNAL_PIN     GPIO_Pin_0
#define RUN_SIGNAL_CLK     RCC_APB2Periph_GPIOA

// 状态定义
#define STATE_STOP       0   // 停止状态
#define STATE_RUNNING    1   // 运行状态
#define UI_MAIN_MENU     0
#define UI_TUNE_MENU     1
#define UI_AB_MEASURE    2
#define PROBLEM_1        1
#define PROBLEM_2        2
#define PROBLEM_AB       3

// 全局状态变量
uint8_t car_state = STATE_STOP;
uint16_t control_tick = 0;    // 控制周期计数，用于 OLED 刷新
uint16_t speed_ramp_tick = 0; // 目标速度提升间隔计数
uint16_t run_timer_last_count = 0;
uint32_t run_elapsed_ticks = 0; // TIM1 以 10kHz 计时，1000 tick = 0.1s
uint8_t key_last_state = 1;  // 上次按键状态（高电平=未按下）
uint8_t key_current_state = 1;
uint8_t count_key_last_state = 1;
uint8_t count_key_current_state = 1;
uint8_t up_key_last_state = 1;
uint8_t up_key_current_state = 1;
uint8_t down_key_last_state = 1;
uint8_t down_key_current_state = 1;
uint8_t prev_key_last_state = 1;
uint8_t prev_key_current_state = 1;
uint8_t next_key_last_state = 0;
uint8_t next_key_current_state = 0;
uint8_t ui_mode = UI_MAIN_MENU;
uint8_t main_menu_index = 0;
uint8_t tune_index = 0;
uint8_t active_problem = PROBLEM_1;
uint8_t finish_target_reached = 0;
uint32_t encoder_left_total = 0;
uint32_t encoder_right_total = 0;
uint32_t encoder_total_count = 0;

// ================== 参数调节区 ==================
// --- 速度 PID 参数 (内环) ---
float V_Kp = 1.5;  // 降低比例系数，减少速度波动
float V_Ki = 0.3;  // 降低积分系数，减少超调

// --- 循迹 PD 参数 (外环) ---
float T_Kp =0.5; // 恢复足够的纠偏力度，同时略低于原来的 0.6
float T_Kd = 5.5;  // 恢复阻尼，避免高速时纠偏滞后形成大摆动

// --- 三种运行模式的速度参数 ---
#define PROBLEM1_START_SPEED    10
#define PROBLEM2_START_SPEED     8
#define AB_START_SPEED           6
#define P1_RAMP_UP_INTERVAL     20  // 每 200ms 增加 1
#define P2_RAMP_UP_INTERVAL     20  // 每 200ms 增加 1
#define AB_TRACK_MAX_TURN        2  // AB直线模式只允许循迹作微小修正
#define AB_RAMP_START_INTERVAL  50  // AB 初始每 500ms 增加 1
#define AB_RAMP_MIN_INTERVAL     8  // AB 最快每 80ms 增加 1
#define AB_RAMP_INTERVAL_STEP    3  // 每升一级速度，等待时间减少 30ms
#define FINISH_WINDOW_BEFORE_PERCENT 10 // 题1/题2从编码器设定值的 90% 开始识别光电横线
#define FINISH_FORCE_STOP_PERCENT 102  // 到编码器设定值的 102% 强制停车
#define FINISH_MIN_ACTIVE           5  // 屏蔽第六路后，只要 5 路探头检测到黑线就停车

int Problem1_Max_Speed = 30;
int Problem2_Max_Speed = 25;
int AB_Max_Speed = 20;
uint32_t Problem1_Stop_Count = 43000UL;
uint32_t Problem2_Stop_Count = 43000UL;
uint32_t AB_Stop_Count = 43000UL;
int Current_Base_Speed = PROBLEM1_START_SPEED;

// --- 共用循迹参数 ---
int Track_Center = 72;
int Track_Dead_Zone = 2;
int Max_Turn = 12;

#define MAIN_MENU_COUNT  4
#define TUNE_ITEM_COUNT  11

// 全局变量区
int Last_V_Err_L = 0, Last_V_Err_R = 0;
int V_I_Sum_L = 0, V_I_Sum_R = 0;
int Last_Track_Error = 0;

// =======================================================================
// 以下是你原本写的传感器数据处理函数 

// 第1~8路依次对应 bit7~bit0，第六路对应 bit2
#define SENSOR_6_MASK  0x04

/**
 * @brief  软件滤波函数：处理传感器杂波
 */
uint8_t Filter_Noise(uint8_t raw_sensor) {
    uint8_t filtered = raw_sensor & (uint8_t)(~SENSOR_6_MASK);
    if ((filtered & 0x3C) != 0) { 
        filtered &= 0x7E;         
    }
    return filtered;
}

/**
 * @brief  计算小车当前在黑线上的质心位置
 */
int Calculate_Position(uint8_t sensor) {
    uint32_t sum = 0;
    uint32_t active_count = 0;
    for(int i = 0; i < 8; i++) {
        if((sensor >> (7 - i)) & 0x01) { 
            sum += (i + 1) * 10;         
            active_count++;
        }
    }
    if (active_count == 0) return -1; 
    return sum / active_count;
}

/**
 * @brief  判断光电阵列是否压到与跑道垂直的黑色停车线
 * @note   使用未经过质心滤波的原始黑线位，第六路仍然被屏蔽
 */
uint8_t Is_Finish_Line(uint8_t sensor) {
    uint8_t active_count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if ((sensor >> (7 - i)) & 0x01) {
            active_count++;
        }
    }

    return (active_count >= FINISH_MIN_ACTIVE);
}

// =======================================================================
// 左右轮独立的速度闭环 PI 控制器

/**
 * @brief  速度 PI 计算：目标速度与编码器实际速度作差
 */
int Velocity_PID_Control(int Target, int Actual, int *I_Sum, int *Last_Err) {
    int Error = Target - Actual;
    int New_I_Sum = *I_Sum + Error;
    
    // 1. 积分限幅
    if (New_I_Sum > 300) New_I_Sum = 300;
    if (New_I_Sum < -300) New_I_Sum = -300;
    
    // 2. 预估输出；输出已饱和且误差继续推向饱和区时，暂停积分
    int Output = (int)(V_Kp * Error + V_Ki * New_I_Sum);
    if (!((Output > 100 && Error > 0) || (Output < 0 && Error < 0))) {
        *I_Sum = New_I_Sum;
    }
    
    // 3. 使用有效积分重新计算并限制 PWM
    Output = (int)(V_Kp * Error + V_Ki * (*I_Sum));
    if (Output > 100) Output = 100;
    if (Output < 0) Output = 0;  // 运行时只允许车轮向前
    
    *Last_Err = Error;
    return Output;
}

// =======================================================================
// C13、C14 按键初始化函数
void Key13_Init(void) {
    // 开启 PC13、PC14、PA1、PB1、PA12、PB3 和 AFIO 时钟
    RCC_APB2PeriphClockCmd(KEY_GPIO_CLK | UP_KEY_CLK | DOWN_KEY_CLK |
                          RCC_APB2Periph_AFIO, ENABLE);

    // PB3 默认属于 JTAG，关闭 JTAG 并保留 SWD 下载调试
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入（低电平有效）
    GPIO_InitStructure.GPIO_Pin = KEY_GPIO_PIN | COUNT_KEY_PIN;
    GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = UP_KEY_PIN;
    GPIO_Init(UP_KEY_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DOWN_KEY_PIN;
    GPIO_Init(DOWN_KEY_PORT, &GPIO_InitStructure);

    // PA12 是 USB D+，常见开发板带外部上拉，改为按键接 GND、低电平有效
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = PREV_KEY_PIN;
    GPIO_Init(PREV_KEY_PORT, &GPIO_InitStructure);

    // PB3 仍使用下拉输入，按键接 3.3V，高电平有效
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Pin = NEXT_KEY_PIN;
    GPIO_Init(NEXT_KEY_PORT, &GPIO_InitStructure);
}

/**
 * @brief  按键扫描函数（带消抖）
 * @retval 1=按键按下，0=未按下
 */
uint8_t Key13_Scan(void) {
    uint8_t key_val = 0;
    key_current_state = GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY_GPIO_PIN);
    
    // 检测下降沿（从未按下到按下）
    if (key_last_state == 1 && key_current_state == 0) {
        Delay_ms(20);  // 消抖延时
        if (GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY_GPIO_PIN) == 0) {
            key_val = 1;  // 确认按下
        }
    }
    key_last_state = key_current_state;
    return key_val;
}

/**
 * @brief  C14 按键扫描函数（带消抖）
 * @retval 1=按键按下，0=未按下
 */
uint8_t Key14_Scan(void) {
    uint8_t key_val = 0;
    count_key_current_state = GPIO_ReadInputDataBit(KEY_GPIO_PORT, COUNT_KEY_PIN);

    if (count_key_last_state == 1 && count_key_current_state == 0) {
        Delay_ms(20);
        if (GPIO_ReadInputDataBit(KEY_GPIO_PORT, COUNT_KEY_PIN) == 0) {
            key_val = 1;
        }
    }
    count_key_last_state = count_key_current_state;
    return key_val;
}

/**
 * @brief  PA1 增加按键扫描
 */
uint8_t UpKey_Scan(void) {
    uint8_t key_val = 0;
    up_key_current_state = GPIO_ReadInputDataBit(UP_KEY_PORT, UP_KEY_PIN);

    if (up_key_last_state == 1 && up_key_current_state == 0) {
        Delay_ms(20);
        if (GPIO_ReadInputDataBit(UP_KEY_PORT, UP_KEY_PIN) == 0) {
            key_val = 1;
        }
    }
    up_key_last_state = up_key_current_state;
    return key_val;
}

/**
 * @brief  PB1 减少按键扫描
 */
uint8_t DownKey_Scan(void) {
    uint8_t key_val = 0;
    down_key_current_state = GPIO_ReadInputDataBit(DOWN_KEY_PORT, DOWN_KEY_PIN);

    if (down_key_last_state == 1 && down_key_current_state == 0) {
        Delay_ms(20);
        if (GPIO_ReadInputDataBit(DOWN_KEY_PORT, DOWN_KEY_PIN) == 0) {
            key_val = 1;
        }
    }
    down_key_last_state = down_key_current_state;
    return key_val;
}

/**
 * @brief  PA12 上一项按键扫描（低电平有效）
 */
uint8_t PrevKey_Scan(void) {
    uint8_t key_val = 0;
    prev_key_current_state = GPIO_ReadInputDataBit(PREV_KEY_PORT, PREV_KEY_PIN);

    if (prev_key_last_state == 1 && prev_key_current_state == 0) {
        Delay_ms(20);
        if (GPIO_ReadInputDataBit(PREV_KEY_PORT, PREV_KEY_PIN) == 0) {
            key_val = 1;
        }
    }
    prev_key_last_state = prev_key_current_state;
    return key_val;
}

/**
 * @brief  PB3 下一项按键扫描（高电平有效）
 */
uint8_t NextKey_Scan(void) {
    uint8_t key_val = 0;
    next_key_current_state = GPIO_ReadInputDataBit(NEXT_KEY_PORT, NEXT_KEY_PIN);

    if (next_key_last_state == 0 && next_key_current_state == 1) {
        Delay_ms(20);
        if (GPIO_ReadInputDataBit(NEXT_KEY_PORT, NEXT_KEY_PIN) == 1) {
            key_val = 1;
        }
    }
    next_key_last_state = next_key_current_state;
    return key_val;
}

/**
 * @brief  初始化 PA0 发车状态输出，上电默认低电平
 */
void RunSignal_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RUN_SIGNAL_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = RUN_SIGNAL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RUN_SIGNAL_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(RUN_SIGNAL_PORT, RUN_SIGNAL_PIN);
}

void RunSignal_Set(uint8_t running) {
    if (running) {
        GPIO_SetBits(RUN_SIGNAL_PORT, RUN_SIGNAL_PIN);
    } else {
        GPIO_ResetBits(RUN_SIGNAL_PORT, RUN_SIGNAL_PIN);
    }
}

/**
 * @brief  电机停止函数
 */
void Motor_Stop(void) {
    Motor_SetSpeed_A(0);
    Motor_SetSpeed_B(0);
    RunSignal_Set(0);
    RemoteParams_UpdateTelemetry(0, 0, 0, 0);
}

/**
 * @brief  清空硬件编码器和累计里程计数
 */
void Encoder_Count_Reset(void) {
    Encoder_GetLeft();
    Encoder_GetRight();
    encoder_left_total = 0;
    encoder_right_total = 0;
    encoder_total_count = 0;
}

/**
 * @brief  累加左右编码器脉冲，使用两轮绝对计数的平均值表示行驶距离
 */
void Encoder_Count_Add(int16_t left_count, int16_t right_count) {
    int32_t left_abs = left_count;
    int32_t right_abs = right_count;

    if (left_abs < 0) left_abs = -left_abs;
    if (right_abs < 0) right_abs = -right_abs;

    encoder_left_total += (uint32_t)left_abs;
    encoder_right_total += (uint32_t)right_abs;
    encoder_total_count = (encoder_left_total + encoder_right_total) / 2;
}

/**
 * @brief  初始化独立运行计时器 TIM1，计数频率为 10kHz
 */
void RunTimer_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * @brief  从按键启动时刻重新开始计时
 */
void RunTimer_Start(void) {
    TIM_SetCounter(TIM1, 0);
    run_timer_last_count = 0;
    run_elapsed_ticks = 0;
}

/**
 * @brief  累加 TIM1 计数差值，支持 16 位计数器回绕
 */
void RunTimer_Update(void) {
    uint16_t now = (uint16_t)TIM_GetCounter(TIM1);
    run_elapsed_ticks += (uint16_t)(now - run_timer_last_count);
    run_timer_last_count = now;
}

uint32_t Get_Active_Stop_Count(void) {
    if (active_problem == PROBLEM_1) return Problem1_Stop_Count;
    if (active_problem == PROBLEM_2) return Problem2_Stop_Count;
    return AB_Stop_Count;
}

int Get_Active_Start_Speed(void) {
    if (active_problem == PROBLEM_1) return PROBLEM1_START_SPEED;
    if (active_problem == PROBLEM_2) return PROBLEM2_START_SPEED;
    return AB_START_SPEED;
}

int Get_Active_Max_Speed(void) {
    if (active_problem == PROBLEM_1) return Problem1_Max_Speed;
    if (active_problem == PROBLEM_2) return Problem2_Max_Speed;
    return AB_Max_Speed;
}

uint16_t Get_Active_Ramp_Interval(void) {
    int32_t interval;
    int speed_step;

    if (active_problem == PROBLEM_1) return P1_RAMP_UP_INTERVAL;
    if (active_problem == PROBLEM_2) return P2_RAMP_UP_INTERVAL;

    // AB 速度越高，下一次升速等待越短，使目标加速度逐步增大
    speed_step = Current_Base_Speed - AB_START_SPEED;
    if (speed_step < 0) speed_step = 0;
    interval = AB_RAMP_START_INTERVAL - speed_step * AB_RAMP_INTERVAL_STEP;
    if (interval < AB_RAMP_MIN_INTERVAL) interval = AB_RAMP_MIN_INTERVAL;
    return (uint16_t)interval;
}

/**
 * @brief  清空 OLED 四行
 */
void OLED_ClearAllLines(void) {
    OLED_ShowString(1, 1, "                ");
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(4, 1, "                ");
}

/**
 * @brief  显示运行速度、累计计数和时间
 */
void OLED_ShowRunInfo(void) {
    uint32_t display_count = encoder_total_count;
    uint32_t display_target = Get_Active_Stop_Count();
    uint32_t deciseconds = run_elapsed_ticks / 1000;

    if (display_count > 999999) display_count = 999999;
    if (display_target > 999999) display_target = 999999;
    if (deciseconds > 999) deciseconds = 999;

    if (active_problem == PROBLEM_1) {
        OLED_ShowString(1, 1, "P1 RUN SPD:000  ");
    } else if (active_problem == PROBLEM_2) {
        OLED_ShowString(1, 1, "P2 RUN SPD:000  ");
    } else {
        OLED_ShowString(1, 1, "AB ACC SPD:000  ");
    }
    OLED_ShowNum(1, 12, Current_Base_Speed, 3);

    OLED_ShowString(2, 1, "C:000000 T:00.0 ");
    OLED_ShowNum(2, 3, display_count, 6);
    OLED_ShowNum(2, 12, deciseconds / 10, 2);
    OLED_ShowNum(2, 15, deciseconds % 10, 1);

    OLED_ShowString(3, 1, "Stop:000000     ");
    OLED_ShowNum(3, 6, display_target, 6);
    OLED_ShowString(4, 1, "C13/C14 Stop   ");
}

/**
 * @brief  显示四项主菜单
 */
void OLED_ShowMainMenu(void) {
    OLED_ClearAllLines();

    if (main_menu_index == 0) {
        OLED_ShowString(1, 1, "MENU 1/4       ");
        OLED_ShowString(2, 1, "Problem1 <20s  ");
        OLED_ShowString(4, 1, "C13 Start      ");
    } else if (main_menu_index == 1) {
        OLED_ShowString(1, 1, "MENU 2/4       ");
        OLED_ShowString(2, 1, "Problem2 <30s  ");
        OLED_ShowString(4, 1, "C13 Start      ");
    } else if (main_menu_index == 2) {
        OLED_ShowString(1, 1, "MENU 3/4       ");
        OLED_ShowString(2, 1, "AB Accel Run    ");
        OLED_ShowString(4, 1, "C13 Start      ");
    } else {
        OLED_ShowString(1, 1, "MENU 4/4       ");
        OLED_ShowString(2, 1, "Parameter Setup");
        OLED_ShowString(4, 1, "C13 Enter      ");
    }
    OLED_ShowString(3, 1, "PA12<      >PB3 ");
}

/**
 * @brief  显示调参菜单
 */
void OLED_ShowTuneMenu(void) {
    uint16_t value_scaled;

    OLED_ClearAllLines();

    switch (tune_index) {
        case 0:
            OLED_ShowString(1, 1, "1 P1 MAX SPEED ");
            OLED_ShowString(2, 1, "Value:000      ");
            OLED_ShowNum(2, 7, Problem1_Max_Speed, 3);
            break;
        case 1:
            OLED_ShowString(1, 1, "2 P2 MAX SPEED ");
            OLED_ShowString(2, 1, "Value:000      ");
            OLED_ShowNum(2, 7, Problem2_Max_Speed, 3);
            break;
        case 2:
            OLED_ShowString(1, 1, "3 AB MAX SPEED ");
            OLED_ShowString(2, 1, "Value:000      ");
            OLED_ShowNum(2, 7, AB_Max_Speed, 3);
            break;
        case 3:
            OLED_ShowString(1, 1, "4 P1 STOP CNT  ");
            OLED_ShowString(2, 1, "Value:000000   ");
            OLED_ShowNum(2, 7, Problem1_Stop_Count, 6);
            break;
        case 4:
            OLED_ShowString(1, 1, "5 P2 STOP CNT  ");
            OLED_ShowString(2, 1, "Value:000000   ");
            OLED_ShowNum(2, 7, Problem2_Stop_Count, 6);
            break;
        case 5:
            OLED_ShowString(1, 1, "6 AB STOP CNT  ");
            OLED_ShowString(2, 1, "Value:000000   ");
            OLED_ShowNum(2, 7, AB_Stop_Count, 6);
            break;
        case 6:
            OLED_ShowString(1, 1, "7 TRACK CENTER ");
            OLED_ShowString(2, 1, "Value:000      ");
            OLED_ShowNum(2, 7, Track_Center, 3);
            break;
        case 7:
            OLED_ShowString(1, 1, "8 TRACK KP     ");
            OLED_ShowString(2, 1, "Value:0.00     ");
            value_scaled = (uint16_t)(T_Kp * 100 + 0.5);
            OLED_ShowNum(2, 7, value_scaled / 100, 1);
            OLED_ShowNum(2, 9, value_scaled % 100, 2);
            break;
        case 8:
            OLED_ShowString(1, 1, "9 TRACK KD     ");
            OLED_ShowString(2, 1, "Value:0.00     ");
            value_scaled = (uint16_t)(T_Kd * 100 + 0.5);
            OLED_ShowNum(2, 7, value_scaled / 100, 1);
            OLED_ShowNum(2, 9, value_scaled % 100, 2);
            break;
        case 9:
            OLED_ShowString(1, 1, "10 MAX TURN    ");
            OLED_ShowString(2, 1, "Value:000      ");
            OLED_ShowNum(2, 7, Max_Turn, 3);
            break;
        default:
            OLED_ShowString(1, 1, "11 AB MEASURE  ");
            OLED_ShowString(2, 1, "C13 Enter Test ");
            break;
    }

    OLED_ShowString(3, 1, "PA1 +   PB1 -  ");
    OLED_ShowString(4, 1, "A12< PB3> C14 BK");
}

/**
 * @brief  调整调参菜单中的当前参数
 */
void Tune_Adjust(int8_t direction) {
    switch (tune_index) {
        case 0:
            Problem1_Max_Speed += direction;
            if (Problem1_Max_Speed > 40) Problem1_Max_Speed = 40;
            if (Problem1_Max_Speed < PROBLEM1_START_SPEED) Problem1_Max_Speed = PROBLEM1_START_SPEED;
            break;
        case 1:
            Problem2_Max_Speed += direction;
            if (Problem2_Max_Speed > 35) Problem2_Max_Speed = 35;
            if (Problem2_Max_Speed < PROBLEM2_START_SPEED) Problem2_Max_Speed = PROBLEM2_START_SPEED;
            break;
        case 2:
            AB_Max_Speed += direction;
            if (AB_Max_Speed > 30) AB_Max_Speed = 30;
            if (AB_Max_Speed < AB_START_SPEED) AB_Max_Speed = AB_START_SPEED;
            break;
        case 3:
            if (direction > 0 && Problem1_Stop_Count <= 999899UL) Problem1_Stop_Count += 100UL;
            if (direction < 0 && Problem1_Stop_Count >= 100UL) Problem1_Stop_Count -= 100UL;
            break;
        case 4:
            if (direction > 0 && Problem2_Stop_Count <= 999899UL) Problem2_Stop_Count += 100UL;
            if (direction < 0 && Problem2_Stop_Count >= 100UL) Problem2_Stop_Count -= 100UL;
            break;
        case 5:
            if (direction > 0 && AB_Stop_Count <= 999899UL) AB_Stop_Count += 100UL;
            if (direction < 0 && AB_Stop_Count >= 100UL) AB_Stop_Count -= 100UL;
            break;
        case 6:
            Track_Center += direction;
            if (Track_Center > 90) Track_Center = 90;
            if (Track_Center < 10) Track_Center = 10;
            break;
        case 7:
            T_Kp += 0.05 * direction;
            if (T_Kp > 3.0) T_Kp = 3.0;
            if (T_Kp < 0.0) T_Kp = 0.0;
            break;
        case 8:
            T_Kd += 0.10 * direction;
            if (T_Kd > 9.0) T_Kd = 9.0;
            if (T_Kd < 0.0) T_Kd = 0.0;
            break;
        case 9:
            Max_Turn += direction;
            if (Max_Turn > 30) Max_Turn = 30;
            if (Max_Turn < 1) Max_Turn = 1;
            break;
        default:
            break;
    }
}

/**
 * @brief  开始题1、题2或 AB 直线模式
 */
void Start_Problem(uint8_t problem) {
    active_problem = problem;
    car_state = STATE_RUNNING;
    finish_target_reached = 0;
    control_tick = 0;
    speed_ramp_tick = 0;
    Current_Base_Speed = Get_Active_Start_Speed();
    RunTimer_Start();
    Encoder_Count_Reset();
    V_I_Sum_L = 0;
    V_I_Sum_R = 0;
    Last_V_Err_L = 0;
    Last_V_Err_R = 0;
    Last_Track_Error = 0;
    RunSignal_Set(1);
    OLED_ShowRunInfo();
    RemoteParams_SendStatus();
}

/**
 * @brief  AB 模式手推测距界面
 */
void OLED_ShowABMeasure(void) {
    uint32_t display_count = encoder_total_count;

    if (display_count > 999999UL) display_count = 999999UL;
    OLED_ClearAllLines();
    OLED_ShowString(1, 1, "AB ENCODER TEST ");
    OLED_ShowString(2, 1, "Count:000000    ");
    OLED_ShowNum(2, 7, display_count, 6);
    OLED_ShowString(3, 1, "C13 Save C14 BK ");
    OLED_ShowString(4, 1, "PA1 Reset       ");
}

/**
 * @brief  停车后显示本次最终时间和计数
 */
void OLED_ShowResult(void) {
    uint32_t deciseconds = run_elapsed_ticks / 1000;
    uint32_t display_count = encoder_total_count;

    if (deciseconds > 9999) deciseconds = 9999;
    if (display_count > 999999) display_count = 999999;

    OLED_ClearAllLines();
    if (!finish_target_reached) {
        OLED_ShowString(1, 1, "MANUAL STOP     ");
    } else if (active_problem == PROBLEM_1) {
        if (run_elapsed_ticks <= 200000UL) {
            OLED_ShowString(1, 1, "P1 DONE PASS    ");
        } else {
            OLED_ShowString(1, 1, "P1 OVER 20s     ");
        }
    } else if (active_problem == PROBLEM_2) {
        if (run_elapsed_ticks <= 300000UL) {
            OLED_ShowString(1, 1, "P2 DONE PASS    ");
        } else {
            OLED_ShowString(1, 1, "P2 OVER 30s     ");
        }
    } else {
        OLED_ShowString(1, 1, "AB ACCEL DONE   ");
    }
    OLED_ShowString(2, 1, "Time:000.0s     ");
    OLED_ShowNum(2, 6, deciseconds / 10, 3);
    OLED_ShowNum(2, 10, deciseconds % 10, 1);
    OLED_ShowString(3, 1, "Count:000000    ");
    OLED_ShowNum(3, 7, display_count, 6);
    OLED_ShowString(4, 1, "A12/PB3 Menu   ");
}

// =======================================================================
// 主函数
int main(void) {
    // 初始化所有外设
    RunSignal_Init();
    RemoteParams_Init();
    OLED_Init();
    Motor_Init();
    LineTracker_Init();
    Encoder_Init();
    Key13_Init();  // 初始化按键
    RunTimer_Init();

    OLED_ShowMainMenu();

    while (1) {
        // 串口中断只缓存命令，所有参数修改在主循环串行执行
        RemoteParams_Process();

        // ==================== 扫描六个按键 ====================
        {
            uint8_t start_pressed = Key13_Scan();
            uint8_t back_pressed = Key14_Scan();
            uint8_t up_pressed = UpKey_Scan();
            uint8_t down_pressed = DownKey_Scan();
            uint8_t prev_pressed = PrevKey_Scan();
            uint8_t next_pressed = NextKey_Scan();

            if (car_state == STATE_STOP) {
                if (ui_mode == UI_MAIN_MENU) {
                    if (prev_pressed) {
                        if (main_menu_index == 0) main_menu_index = MAIN_MENU_COUNT - 1;
                        else main_menu_index--;
                        OLED_ShowMainMenu();
                    }
                    if (next_pressed) {
                        main_menu_index++;
                        if (main_menu_index >= MAIN_MENU_COUNT) main_menu_index = 0;
                        OLED_ShowMainMenu();
                    }
                    if (back_pressed) {
                        OLED_ShowMainMenu();
                    }
                    if (start_pressed) {
                        if (main_menu_index == 0) {
                            Start_Problem(PROBLEM_1);
                        } else if (main_menu_index == 1) {
                            Start_Problem(PROBLEM_2);
                        } else if (main_menu_index == 2) {
                            Start_Problem(PROBLEM_AB);
                        } else {
                            ui_mode = UI_TUNE_MENU;
                            OLED_ShowTuneMenu();
                        }
                    }
                } else if (ui_mode == UI_TUNE_MENU) {
                    if (prev_pressed) {
                        if (tune_index == 0) tune_index = TUNE_ITEM_COUNT - 1;
                        else tune_index--;
                        OLED_ShowTuneMenu();
                    }
                    if (next_pressed) {
                        tune_index++;
                        if (tune_index >= TUNE_ITEM_COUNT) tune_index = 0;
                        OLED_ShowTuneMenu();
                    }
                    if (up_pressed) {
                        Tune_Adjust(1);
                        OLED_ShowTuneMenu();
                        RemoteParams_SendConfig();
                    }
                    if (down_pressed) {
                        Tune_Adjust(-1);
                        OLED_ShowTuneMenu();
                        RemoteParams_SendConfig();
                    }
                    if (start_pressed && tune_index == 10) {
                        ui_mode = UI_AB_MEASURE;
                        control_tick = 0;
                        Encoder_Count_Reset();
                        Motor_Stop();
                        OLED_ShowABMeasure();
                    }
                    if (back_pressed) {
                        ui_mode = UI_MAIN_MENU;
                        OLED_ShowMainMenu();
                    }
                } else {
                    // AB 手推测距：实时累计两轮编码器绝对计数的平均值
                    Encoder_Count_Add((int16_t)Encoder_GetLeft(),
                                      (int16_t)Encoder_GetRight());
                    control_tick++;

                    if (up_pressed) {
                        Encoder_Count_Reset();
                        OLED_ShowABMeasure();
                    } else if (start_pressed) {
                        if (encoder_total_count > 0UL) {
                            AB_Stop_Count = encoder_total_count;
                            RemoteParams_SendConfig();
                        }
                        ui_mode = UI_TUNE_MENU;
                        OLED_ShowTuneMenu();
                    } else if (back_pressed) {
                        ui_mode = UI_TUNE_MENU;
                        OLED_ShowTuneMenu();
                    } else if ((control_tick % 10) == 0) {
                        OLED_ShowABMeasure();
                    }
                }
            } else if (start_pressed || back_pressed) {
                // 运行中手动停止，不再缓停
                Encoder_Count_Add((int16_t)Encoder_GetLeft(),
                                  (int16_t)Encoder_GetRight());
                RunTimer_Update();
                finish_target_reached = 0;
                Motor_Stop();
                car_state = STATE_STOP;
                ui_mode = UI_MAIN_MENU;
                OLED_ShowResult();
                RemoteParams_SendStatus();
            }
        }
        
        // ==================== 运行状态处理 ====================
        if (car_state == STATE_RUNNING) {
            uint32_t stop_count;
            uint32_t finish_window_margin;
            uint32_t finish_window_min;
            uint32_t finish_window_max;
            int active_max_speed;
            int turn = 0;
            uint8_t sensor_raw = 0;
            uint8_t should_stop = 0;

            control_tick++;
            RunTimer_Update();
            
            // 1. 所有模式都读取循迹传感器；AB模式稍后只保留很小的修正量
            {
                uint8_t raw_data = LineTracker_ReadData();
                uint8_t sensor;
                int current_pos;
                int track_error;

                sensor_raw = (uint8_t)(~raw_data) & (uint8_t)(~SENSOR_6_MASK);
                sensor = Filter_Noise(sensor_raw);

                // 2. 获取循迹位置
                current_pos = Calculate_Position(sensor);

                // 3. 原来的丢线保护逻辑
                if (current_pos == -1) {
                    if (Last_Track_Error > 0) {
                        current_pos = 0;
                    } else {
                        current_pos = 90;
                    }
                }

                // 4. 原来的外环循迹 PD 计算
                track_error = Track_Center - current_pos;
                if (track_error >= -Track_Dead_Zone && track_error <= Track_Dead_Zone) {
                    track_error = 0;
                }
                turn = (int)(T_Kp * track_error + T_Kd * (track_error - Last_Track_Error));
                Last_Track_Error = track_error;

                if (active_problem == PROBLEM_AB) {
                    if (turn > AB_TRACK_MAX_TURN) turn = AB_TRACK_MAX_TURN;
                    if (turn < -AB_TRACK_MAX_TURN) turn = -AB_TRACK_MAX_TURN;
                }
            }

            // 5. 读取并累计编码器
            int actual_L = Encoder_GetLeft();
            int actual_R = Encoder_GetRight();
            Encoder_Count_Add((int16_t)actual_L, (int16_t)actual_R);

            // 6. AB 直接按编码器停车；题1/题2在 90%~102% 区间识别横向黑线
            stop_count = Get_Active_Stop_Count();
            active_max_speed = Get_Active_Max_Speed();

            if (active_problem == PROBLEM_AB) {
                if (stop_count > 0UL && encoder_total_count >= stop_count) {
                    should_stop = 1;
                }
            } else if (stop_count > 0UL) {
                finish_window_margin = stop_count * FINISH_WINDOW_BEFORE_PERCENT / 100UL;
                finish_window_min = stop_count - finish_window_margin;
                finish_window_max = stop_count * FINISH_FORCE_STOP_PERCENT / 100UL;

                if (encoder_total_count >= finish_window_max) {
                    // 达到 102% 不再等待光电信号，直接停车
                    should_stop = 1;
                } else if (encoder_total_count >= finish_window_min) {
                    if (Is_Finish_Line(sensor_raw)) {
                        should_stop = 1;
                    }
                }
            }

            if (should_stop) {
                finish_target_reached = 1;
                RunTimer_Update();
                Motor_Stop();
                car_state = STATE_STOP;
                ui_mode = UI_MAIN_MENU;
                OLED_ShowResult();
                RemoteParams_SendStatus();
                continue;
            }

            // AB 的升速间隔会随速度提高而缩短，形成逐步增大的目标加速度
            if (Current_Base_Speed < active_max_speed) {
                speed_ramp_tick++;
                if (speed_ramp_tick >= Get_Active_Ramp_Interval()) {
                    Current_Base_Speed++;
                    speed_ramp_tick = 0;
                }
            }

            // 差速不能超过当前基础速度，防止单轮反转
            if (turn > Max_Turn) turn = Max_Turn;
            if (turn < -Max_Turn) turn = -Max_Turn;
            if (turn > Current_Base_Speed) turn = Current_Base_Speed;
            if (turn < -Current_Base_Speed) turn = -Current_Base_Speed;

            // 7. 计算左右轮目标速度和 PI 输出；AB只使用限幅后的微小循迹修正
            int target_L = Current_Base_Speed - turn;
            int target_R = Current_Base_Speed + turn;
            int pwm_L = Velocity_PID_Control(target_L, actual_L, &V_I_Sum_L, &Last_V_Err_L);
            int pwm_R = Velocity_PID_Control(target_R, actual_R, &V_I_Sum_R, &Last_V_Err_R);

            // 8. 输出控制给电机
            Motor_SetSpeed_A(pwm_L);
            Motor_SetSpeed_B(pwm_R);
            RemoteParams_UpdateTelemetry((int16_t)actual_L, (int16_t)actual_R,
                                         (int16_t)pwm_L, (int16_t)pwm_R);

            // 每 100ms 刷新速度、计数和时间
            if ((control_tick % 10) == 0) {
                OLED_ShowRunInfo();
            }
        } else {
            // 停止状态：确保电机停止
            Motor_Stop();
        }

        Delay_ms(10); // 严格维持 10ms 控制周期
    }
}
