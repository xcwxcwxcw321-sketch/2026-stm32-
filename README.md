# STM32 + Raspberry Pi 钢球水管平衡系统

本仓库整理本次竞赛使用的 STM32 电机控制工程、树莓派视觉与步进控制网页、部署服务和调试脚本。

## 目录

- `stm32/`：STM32F103 电机控制 Keil 工程，包含 USART1 远程参数协议、PID、编码器、电机和 OLED 模块。
- `raspberry-pi/`：树莓派视觉识别、D36A 步进轴控制网页、参数文件和调试脚本。
- `deployment/`：部署辅助文件和运行说明。

## STM32 串口参数协议

```text
GET
STATUS
SET,NAME,VALUE
SAVE
LOAD
DEFAULT
HELP
```

参数名包括 `VKP`、`VKI`、`TKP`、`TKD`、`P1_MAX`、`P2_MAX`、`AB_MAX`、`P1_STOP`、`P2_STOP`、`AB_STOP`、`CENTER`、`DEAD`、`MAX_TURN`。

## 安全说明

- 树莓派部署脚本通过环境变量 `D36A_PI_PASSWORD` 获取 SSH 密码，不在仓库中保存密码。
- 运行前确认 D36A 使用独立 12 V 供电，树莓派与驱动器共地，并确认步进轴最低点。
- `Objects/`、`Listings/`、Python 缓存、图片和压缩包等生成物没有纳入仓库。
