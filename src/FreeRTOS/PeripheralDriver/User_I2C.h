#ifndef USER_I2C_H
#define USER_I2C_H
#include "hc32_ddl.h"
uint8_t E2_Initialize(void);
void User_I2C1_Init(void);
uint8_t User_I2C1_Master_Read(uint8_t DeviceAddr, uint8_t DataAddr,uint8_t *RxData, uint8_t Data_len);
uint8_t User_I2C1_Master_Write(uint8_t DeviceAddr, uint8_t DataAddr,uint8_t *TxData, uint8_t Data_len);
void TestEEPROM(void);
//void User_I2C1_Master_Read_OneByte(uint8_t DeviceAddr, uint8_t DataAddr,uint8_t *RecData);

#endif
