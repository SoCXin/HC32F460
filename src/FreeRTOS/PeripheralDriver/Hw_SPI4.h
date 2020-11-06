#ifndef HW_SPI4_H
#define HW_SPI4_H
#include "hc32_ddl.h"
void Hw_SPI4_Init(void);
void Hw_SPI4_TEST(void);
void Hw_SPI4_DMA_Init(void);
void Hw_SPI4_TX_DMA_Init(void);
void SPI4_WriteBuffer_DMA(uint8_t *pdata,uint8_t Len);
void SPI4_WriteBuffer(uint8_t *pdata,uint8_t Len);
void SPI4_WriteBuffer_INT(uint8_t *pdata,uint8_t Len);
#endif
