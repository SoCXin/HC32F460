#ifndef HW_SPI1_H
#define HW_SPI1_H
#include "hc32_ddl.h"
//unsigned char SPI_DATA;
/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE
/* SPI_SCK Port/Pin definition */
#ifdef SPI_MASTER_MODE
#define CLK_DIVISION        SpiClkDiv8
#define SPI3_TRNCNT              (50u)//传输次数
#endif
#ifdef SPI3_SLAVE_MODE
#define CLK_DIVISION        SpiClkDiv2
#define SPI_DMA_TRNCNT              (40*1024)//传输次数
#endif
#define SPI3_SCK_PORT                    PortB
#define SPI3_SCK_PIN                     Pin12
#define SPI3_SCK_FUNC                    Func_Spi3_Sck

/* SPI_NSS Port/Pin definition */
#define SPI3_NSS_PORT                    PortB
#define SPI3_NSS_PIN                     Pin13
#define SPI3_NSS_FUNC                    Func_Spi3_Nss0

/* SPI_MOSI Port/Pin definition */
#define SPI3_MOSI_PORT                   PortB
#define SPI3_MOSI_PIN                    Pin14
#define SPI3_MOSI_FUNC                   Func_Spi3_Mosi

/* SPI_MISO Port/Pin definition */
#define SPI3_MISO_PORT                   PortB
#define SPI3_MISO_PIN                    Pin15
#define SPI3_MISO_FUNC                   Func_Spi3_Miso

/* SPI unit and clock definition */
#define SPI3_UNIT                        M4_SPI3
#define SPI3_UNIT_CLOCK                  PWC_FCG1_PERIPH_SPI3
#define SPI3_TX_INT_SOURCE               INT_SPI3_SRTI
#define SPI3_RX_INT_SOURCE               INT_SPI3_SRRI
#define SPI3_ERR_INT_SOURCE              INT_SPI3_SPEI
#define SPI3_ERR_IDEL_SOURCE             INT_SPI3_SPII

#define SPI3_TX_IRQn            Int004_IRQn
#define SPI3_RX_IRQn            Int005_IRQn
#define SPI3_ERR_IRQn           Int006_IRQn
#define SPI3_IDEL_IRQn          Int007_IRQn

void Hw_SPI3_Init(void);
void Hw_SPI3_TEST(void);

#endif
