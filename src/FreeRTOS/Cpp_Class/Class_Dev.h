#ifndef CLASS_DEV_H
#define CLASS_DEV_H
#include "hc32_ddl.h"
typedef uint8_t status_device_t;
typedef uint16_t rt_size_t;
typedef uint16_t rt_off_t;
typedef struct rt_device_ops
{
    /* common device interface */
	uint8_t  (*init)   (M4_I2C_TypeDef *dev,uint32_t baudrate);
    uint8_t  (*close)  (M4_I2C_TypeDef *dev);
    uint8_t (*read)   (M4_I2C_TypeDef *dev, uint8_t DeviceAddr,uint8_t addr, uint8_t *buffer, uint16_t len);
    uint8_t (*write)  (M4_I2C_TypeDef *dev, uint8_t DeviceAddr,uint8_t addr, const uint8_t *buffer, uint16_t len);//Device Address addr针对I2C，其他接口无此参数可设置为0
	uint8_t (*setio)  (en_port_t port,uint16_t pin);
	uint8_t (*clrio)  (en_port_t port,uint16_t pin);
	uint8_t (*togleio) (en_port_t port,uint16_t pin);
} rt_device_ops_t;
class IOPort
{
public:
	//IO只实现以下实例
	virtual uint8_t IO_init(en_port_t port,uint16_t pin,en_pin_mode_t mode);
	virtual uint8_t Setio(en_port_t port,uint16_t pin);
	virtual uint8_t Clrio(en_port_t port,uint16_t pin);
	virtual uint8_t Togleio(en_port_t port,uint16_t pin);
};
class SerialInterface
{
public:
	virtual uint8_t SerialInterface_init(uint32_t baudrate);
	virtual uint8_t SerialRead(uint8_t DeviceAddr,uint8_t addr, uint8_t *buffer, uint16_t len);
	virtual uint8_t SerialWrite(uint8_t DeviceAddr,uint8_t addr, const uint8_t *buffer, uint16_t len);
	virtual uint8_t SerialClose();
};
//class hcDevice : public  SerialInterface, public IOPort
//{
//public:
//    hcDevice(void *device_base,rt_device_ops_t *ops);
//	~hcDevice();
//	//接口	
//private:  
//	const struct rt_device_ops *ops;
//	void *Device_Base;
//};
class hcDeviceIO : public  IOPort
{
	public:
    hcDeviceIO();
	~hcDeviceIO();
	//接口	
	private:  
	const struct rt_device_ops *ops;
	void *Device_Base;
};
class hcDeviceSerial : public  SerialInterface
{
	public:
    hcDeviceSerial();
	~hcDeviceSerial();
	//接口	
	private:  
	const struct rt_device_ops *ops;
	void *Device_Base;
};

#endif
