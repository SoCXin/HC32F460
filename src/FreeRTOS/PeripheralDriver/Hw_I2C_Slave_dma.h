#ifndef HW_I2C_SLAVE_DMA_H
#define HW_I2C_SLAVE_DMA_H
#include "hc32_ddl.h"
#include "bsp_dma.h"
/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Define I2C unit used for the example */
#define I2C_SLAVE_CH                          (M4_I2C1)
/* Define slave device address for example */
#define SLAVE_ADDRESS                   0x06u
/* Define port and pin for SDA and SCL */
#define I2C1_SCL_PORT                   (PortC)
#define I2C1_SCL_PIN                    (Pin04)
#define I2C1_SDA_PORT                   (PortC)
#define I2C1_SDA_PIN                    (Pin05)

#define TIMEOUT                         ((uint32_t)0x10000)

#define I2C_RET_OK                      0u
#define I2C_RET_ERROR                   1u

#define GENERATE_START                  0x00u
#define GENERATE_RESTART                0x01u

#define ADDRESS_W                       0x00u
#define ADDRESS_R                       0x01u

/* Define Write and read data length for the example */
#define SLAVE_DATA_LEN                   64u
/* Define i2c baudrate */
#define I2C_BAUDRATE                    400000ul

#define SLAVE_I2C_DMA_UNIT    M4_DMA1
#define SLAVE_DMA_RX_CH       (DmaCh0)
#define SLAVE_DMA_TX_CH       (DmaCh1)
#define SLAVE_DMA_RX_CH_TRIG  EVT_I2C1_RXI
#define SLAVE_DMA_TX_CH_TRIG  EVT_I2C1_TXI
#define SLAVE_DMA_TX_INT      INT_DMA1_TC1
#define SLAVE_DMA_RX_INT      INT_DMA1_TC0
#define SLAVE_DMA_Mode_RXsrc  AddressFix
#define SLAVE_DMA_Mode_RXdes  AddressIncrease
#define SLAVE_DMA_Mode_TXsrc  AddressIncrease
#define SLAVE_DMA_Mode_TXdes  AddressFix
#define SLAVE_DataWidth       Dma8Bit

#define TIMERA_UNIT_CLOCK(x)            PWC_FCG2_PERIPH_TIMA##x
#define TIMERA_UNIT(x)                  M4_TMRA##x

uint8_t bsp_i2c_slave_init(M4_I2C_TypeDef* I2Cx);
void slave_prepareTXdata(uint8_t addr, uint8_t *data ,uint8_t len);
void I2C_slave_init(void);
void TimerA1_Init(void);
void TimerA2_Init(void);

#endif
