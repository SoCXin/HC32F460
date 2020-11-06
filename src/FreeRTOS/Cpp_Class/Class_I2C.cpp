#include "Class_I2C.h"

I2C_Class::I2C_Class(M4_I2C_TypeDef *dev,rt_device_ops_t *ops)
{
	I2C_DEV = dev;
	Op = ops;
}
I2C_Class::~I2C_Class()
{
	Op->close(I2C_DEV);
	Op = NULL;
}
uint8_t I2C_Class::I2C_init(uint32_t baudrate)
{
	Op->init(I2C_DEV,baudrate);
	return 0;
}
uint8_t I2C_Class::I2C_Read(uint8_t DeviceAddr,uint8_t addr,uint8_t *buffer, uint16_t len)
{
	Op->read(I2C_DEV,DeviceAddr,addr,buffer,len);
	return 0;
}
uint8_t I2C_Class::I2C_write(uint8_t DeviceAddr,uint8_t addr, const uint8_t *buffer, uint16_t len)
{
	Op->write(I2C_DEV,DeviceAddr,addr,buffer,len);
	return 0;
}

