#include "stm32f10x.h"
#include "Delay.h"

// 引脚操作宏定义
#define SCL_PORT    GPIOB
#define SCL_PIN     GPIO_Pin_10
#define SDA_PORT    GPIOB
#define SDA_PIN     GPIO_Pin_11

void MyI2C_W_SCL(uint8_t BitValue) {
	GPIO_WriteBit(SCL_PORT, SCL_PIN, (BitAction)BitValue);
	Delay_us(5);
}

void MyI2C_W_SDA(uint8_t BitValue) {
	GPIO_WriteBit(SDA_PORT, SDA_PIN, (BitAction)BitValue);
	Delay_us(5);
}

uint8_t MyI2C_R_SDA(void) {
	uint8_t BitValue = GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN);
	Delay_us(5);
	return BitValue;
}

void MyI2C_Init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出
	GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, SCL_PIN | SDA_PIN); // 默认高电平
}

void MyI2C_Start(void) {
	MyI2C_W_SDA(1); MyI2C_W_SCL(1);
	MyI2C_W_SDA(0); MyI2C_W_SCL(0);
}

void MyI2C_Stop(void) {
	MyI2C_W_SDA(0); MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte) {
	for (uint8_t i = 0; i < 8; i++) {
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1); MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void) {
	uint8_t Byte = 0x00;
	MyI2C_W_SDA(1); // 释放SDA
	for (uint8_t i = 0; i < 8; i++) {
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA()) { Byte |= (0x80 >> i); }
		MyI2C_W_SCL(0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit) {
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1); MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void) {
	MyI2C_W_SDA(1); MyI2C_W_SCL(1);
	uint8_t AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}
