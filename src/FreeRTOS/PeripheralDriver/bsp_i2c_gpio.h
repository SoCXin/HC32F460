#ifndef BSP_I2C_GPIO_H
#define BSP_I2C_GPIO_H
#include <hc32_ddl.h>

#define SCL_DIR bM4_PORT_PCRC4_POUTE
#define SDA_DIR bM4_PORT_PCRC5_POUTE
#define SCL_Pin_out bM4_PORT_PODRC_POUT04
#define SDA_Pin_out bM4_PORT_PODRC_POUT05
#define SDA_Pin_in bM4_PORT_PIDRC_PIN05

//Íâ²¿º¯Êý
void Set_SCL_DIR(void);
void Set_SDA_DIR(unsigned char dir);
void Set_SCL_pin(bool value);
void Set_SDA_pin(bool value);
unsigned char Get_SDA_pin(void);
void Gpio_I2C_Init(void);
void I2C_Start(void);
unsigned char  I2C_Write(unsigned char);
unsigned char I2C_Read(unsigned char);
unsigned char  I2C_GetAck(void);
void I2C_PutAck(unsigned char);
void I2C_Stop(void);

unsigned char I2C_Send_Command(uint8_t DeviceAddr, uint8_t address, uint8_t *data, uint8_t len);
unsigned char I2C_Read_Command(uint8_t DeviceAddr, uint8_t address, uint8_t *rxbuf, uint8_t len);
#endif
