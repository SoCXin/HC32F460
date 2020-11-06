#ifndef CLASS_I2C_H
#define CLASS_I2C_H

#include "Hw_I2C.h"
typedef void (*p_func_t)(void);
typedef struct rt_device_ops
{
    /* common device interface */
	uint8_t  (*init)   (M4_I2C_TypeDef *dev,uint32_t baudrate);
    uint8_t  (*close)  (M4_I2C_TypeDef *dev);
    uint8_t (*read)   (M4_I2C_TypeDef *dev, uint8_t DeviceAddr,uint8_t addr, uint8_t *buffer, uint16_t len);
    uint8_t (*write)  (M4_I2C_TypeDef *dev, uint8_t DeviceAddr,uint8_t addr, const uint8_t *buffer, uint16_t len);
} rt_device_ops_t;
class I2C_Class
{
public:
	I2C_Class(M4_I2C_TypeDef *dev,rt_device_ops_t *ops);
	~I2C_Class();
	uint8_t I2C_init(uint32_t baudrate);
	uint8_t I2C_write(uint8_t DeviceAddr,uint8_t addr, const uint8_t *buffer, uint16_t len);
	uint8_t I2C_Read(uint8_t DeviceAddr,uint8_t addr, uint8_t *buffer, uint16_t len);
private:
	M4_I2C_TypeDef *I2C_DEV;
	rt_device_ops_t *Op;
};
#endif
