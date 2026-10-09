#include "stm32f10x.h"
#include "MyI2C.h"

// 模块地址与寄存器
#define TRACKER_ADDR    0x24   // 7位地址0x12左移一位
#define REG_DATA        0x30

void LineTracker_Init(void) {
	MyI2C_Init();
	// 上电需要等待20s稳定，建议在main函数最初或手动校准，这里不做强制延时
}

// 读取8个探头的数据
uint8_t LineTracker_ReadData(void) {
	uint8_t data;
	
	MyI2C_Start();
	MyI2C_SendByte(TRACKER_ADDR);       // 发送写地址
	MyI2C_ReceiveAck();
	MyI2C_SendByte(REG_DATA);           // 发送要读取的寄存器 0x30
	MyI2C_ReceiveAck();
	
	MyI2C_Start();                      // Restart
	MyI2C_SendByte(TRACKER_ADDR | 0x01);// 发送读地址 (0x25)
	MyI2C_ReceiveAck();
	data = MyI2C_ReceiveByte();         // 接收数据
	MyI2C_SendAck(1);                   // 产生非应答NACK
	MyI2C_Stop();
	
	return data;
}
