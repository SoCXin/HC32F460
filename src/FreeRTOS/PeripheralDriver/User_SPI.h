#ifndef USER_SPI_H
#define USER_SPI_H
#include "hc32_ddl.h"
void User_SPI_Init(void);
void USER_SPI_TEST(void);
void SPI_Writedata(uint8_t data);
uint8_t SPI_ReadData(void);
uint8_t SPIx_ReadWriteByte(uint8_t TxData);
#endif
