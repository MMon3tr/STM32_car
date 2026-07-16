#include "ml_i2c.h"

#define SDA_Output(x)  gpio_set(I2C_GPIO, I2C_SDA_GPIO_Pin, x)
#define SCL_Output(x)  gpio_set(I2C_GPIO, I2C_SCL_GPIO_Pin, x)
#define SDA_Input()    gpio_get(I2C_GPIO, I2C_SDA_GPIO_Pin)

/* I2C时序延时: 约250kHz (Fast-mode兼容), 72MHz下delay_us(2) ≈ 2us */
#define I2C_DELAY      delay_us(2)

void I2C_Init()
{
	gpio_init(I2C_GPIO, I2C_SCL_GPIO_Pin, OUT_OD);
	gpio_init(I2C_GPIO, I2C_SDA_GPIO_Pin, OUT_OD);

	SDA_Output(1);
	SCL_Output(1);
}

// 起始信号
void I2C_Start()
{
	SDA_Output(1);
	SCL_Output(1);
	I2C_DELAY;
	SDA_Output(0);
	I2C_DELAY;
	SCL_Output(0);
}

// 停止信号
void I2C_Stop()
{
	SDA_Output(0);
	I2C_DELAY;
	SCL_Output(1);
	I2C_DELAY;
	SDA_Output(1);
}

// 发送一个字节
void I2C_SendByte(uint8_t byte)
{
	for(int i = 0; i < 8; i++)
	{
		if(byte & (0x80>>i))
			SDA_Output(1);
		else
			SDA_Output(0);
		I2C_DELAY;
		SCL_Output(1);
		I2C_DELAY;
		SCL_Output(0);
	}
}

// 接收一个字节
uint8_t I2C_ReceiveByte()
{
	uint8_t byte = 0;
	SDA_Output(1);
	for(int i = 0; i < 8; i++)
	{
		SCL_Output(1);
		I2C_DELAY;
		if(SDA_Input())
			byte |= (0x80>>i);
		SCL_Output(0);
		I2C_DELAY;
	}

	return byte;
}

// 发送应答
void I2C_SendAck()
{
	SDA_Output(0);
	I2C_DELAY;
	SCL_Output(1);
	I2C_DELAY;
	SCL_Output(0);
}

// 不发送应答
void I2C_NotSendAck()
{
	SDA_Output(1);
	I2C_DELAY;
	SCL_Output(1);
	I2C_DELAY;
	SCL_Output(0);
}

// 等待从机应答
uint8_t I2C_WaitAck()
{
	uint8_t byte = 0;
	SDA_Output(1);
	I2C_DELAY;
	SCL_Output(1);
	I2C_DELAY;
	byte = SDA_Input();
	SCL_Output(0);
	I2C_DELAY;

	return byte;
}

