#ifndef HW_I2C_H
#define HW_I2C_H
#include "hc32_ddl.h"
#define I2C1_UNIT M4_I2C1
#define I2C2_UNIT M4_I2C2
#define I2C3_UNIT M4_I2C3

#define I2C1_SCL_PORT   PortC//PortA//PortC
#define I2C1_SCL_Pin    Pin04//Pin01//Pin04
#define I2C1_SDA_PORT   PortC//PortA//PortC
#define I2C1_SDA_Pin    Pin05//Pin00//Pin05
#define I2C1_CLK			PWC_FCG1_PERIPH_I2C1
#define I2C2_CLK			PWC_FCG1_PERIPH_I2C2
#define I2C3_CLK			PWC_FCG1_PERIPH_I2C3

#define TIMEOUT                         ((uint32_t)0x10000)

#define I2C_RET_OK                      0
#define I2C_RET_ERROR                   1
#define I2C_BUSY						2
#define I2C_TIMEROUT					3
#define I2C_BADADDR						4
#define I2C_BADPARA                     5//²ÎÊý´íÎó

#ifdef __cplusplus
extern "C" {
#endif

void HW_I2C_Port_Init(void);
uint8_t HW_I2C_Init(M4_I2C_TypeDef* pstcI2Cx,uint32_t baudrate);
uint8_t I2C_Write_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, const uint8_t *data, uint16_t len);
uint8_t I2C_Read_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, uint8_t *data, uint16_t len);	
uint8_t I2C_Write_Buffer(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,const uint8_t *data, uint16_t len);	
uint8_t I2C_Read_Buffer(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr, uint8_t *data, uint16_t len);	
#ifdef __cplusplus
};
#endif

#endif
