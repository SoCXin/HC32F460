#ifndef BSP_DMA_H
#define BSP_DMA_H
#include "hc32_ddl.h"

#ifdef __cplusplus
extern "C" {
#endif
void bsp_dma_init(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    en_dma_address_mode_t enSrcMode,\
                    en_dma_address_mode_t enDesMode,\
                    en_dma_transfer_width_t DataWidth
                );
void bsp_dma_SetSrcAddr(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    uint32_t addr
                    );
void bsp_dma_SetDesAddr(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    uint32_t addr
                    );
void bsp_dma_set_TrigSrc(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    en_event_src_t EventSrc
                    );   
void bsp_dma_set_count(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    uint16_t num);
void bsp_dma_ch_enable(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch, 
                    bool value);

#ifdef __cplusplus
};
#endif
#endif