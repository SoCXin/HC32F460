#ifndef BSP_I2C_DMA_H
#define BSP_I2C_DMA_H
#include "hc32_ddl.h"
#include "bsp_dma.h"
#include "bsp_interrupt.h"
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

#define I2C_DMA_UNIT    M4_DMA1
#define DMA_RX_CH       (DmaCh0)
#define DMA_TX_CH       (DmaCh1)
#define DMA_RX_CH_TRIG  EVT_I2C1_RXI
#define DMA_TX_CH_TRIG  EVT_I2C1_TXI
#define DMA_TX_INT      INT_DMA1_TC1
#define DMA_RX_INT      INT_DMA1_TC0
#define DMA_Mode_RXsrc  AddressFix
#define DMA_Mode_RXdes  AddressIncrease
#define DMA_Mode_TXsrc  AddressIncrease
#define DMA_Mode_TXdes  AddressFix
#define DataWidth       Dma8Bit

#define DMA_CH0_IRQn       Int022_IRQn
#define DMA_CH1_IRQn       Int023_IRQn

#define TIMEOUT                         ((uint32_t)0x2000)

#define I2C_RET_OK                      0
#define I2C_RET_ERROR                   1
#define I2C_BUSY						2
#define I2C_TIMEROUT					3
#define I2C_BADADDR						4
#define I2C_BADPARA                     5//²ÎÊý´íÎó

#ifdef __cplusplus
extern "C" {
#endif
extern uint8_t  bsp_I2C_DMA_Init(M4_I2C_TypeDef* pstcI2Cx,uint32_t baudrate);
extern uint8_t I2C_DMA_Write_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, const uint8_t *data, uint16_t len);
extern uint8_t I2C_DMA_Read_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, uint8_t *data, uint16_t len);    
    
#ifdef __cplusplus
};
#endif



#endif

