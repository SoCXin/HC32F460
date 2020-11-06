#ifndef HW_I2C_SLAVE_H
#define HW_I2C_SLAVE_H
#include "hc32_ddl.h"
#include "Hw_I2C.h"

#define I2C_SLAVE_SCL_PORT   PortC//PortA//PortC
#define I2C_SLAVE_SCL_Pin    Pin04//Pin01//Pin04
#define I2C_SLAVE_SDA_PORT   PortC//PortA//PortC
#define I2C_SLAVE_SDA_Pin    Pin05//Pin00//Pin05
#define I2C_SLAVE_SCL_FUNC		Func_I2c1_Scl
#define I2C_SLAVE_SDA_FUNC		Func_I2c1_Sda


#define SLAVE_DATA_LEN                   64u
#define SLAVE_ADDRESS                   0x06u

#define GENERATE_START                  0x00u
#define GENERATE_RESTART                0x01u

#define ADDRESS_W                       0x00u
#define ADDRESS_R                       0x01u



uint8_t Hw_I2C_Slave_Init(M4_I2C_TypeDef* I2Cx);

#endif
