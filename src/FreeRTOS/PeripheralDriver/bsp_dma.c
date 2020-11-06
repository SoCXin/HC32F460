#include "bsp_dma.h"

void bsp_dma_init(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    en_dma_address_mode_t enSrcMode,\
                    en_dma_address_mode_t enDesMode,\
                    en_dma_transfer_width_t DataWidth
                )
{
    stc_dma_config_t stcDmaCfg;
    MEM_ZERO_STRUCT(stcDmaCfg);
    
    stcDmaCfg.u16BlockSize = 1;//
    stcDmaCfg.u16TransferCnt = 1;//
    
   
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = 0;
    stcDmaCfg.u16DesRptSize = 0;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = enSrcMode;
    stcDmaCfg.stcDmaChCfg.enDesInc = enDesMode;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = DataWidth;
    //turn on DMA UNIT Clock
    if(pstcDmaReg == M4_DMA1)
    {
       PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1, Enable); 
    }
    else
    {
       PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable); 
    }
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
    /* Enable DMA UNIT. */
    DMA_Cmd(pstcDmaReg,Enable);   
    /* Initialize DMA channel. */
    DMA_InitChannel(pstcDmaReg, u8Ch, &stcDmaCfg);
    /* Enable DMA channel. */
    DMA_ChannelCmd(pstcDmaReg, u8Ch,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(pstcDmaReg, u8Ch,TrnCpltIrq);
}

void bsp_dma_SetDesAddr(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    uint32_t addr
                    )
{
    DMA_SetDesAddress(pstcDmaReg, u8Ch, addr);
}

void bsp_dma_SetSrcAddr(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    uint32_t addr
                    )
{
    DMA_SetSrcAddress(pstcDmaReg, u8Ch, addr);
}

void bsp_dma_set_TrigSrc(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    en_event_src_t EventSrc
                    )
{
    DMA_SetTriggerSrc(pstcDmaReg, u8Ch, EventSrc);
}
void bsp_dma_set_count(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch,\
                    uint16_t num)
{
    DMA_SetTransferCnt(pstcDmaReg, u8Ch, num);
}
void bsp_dma_ch_enable(M4_DMA_TypeDef* pstcDmaReg, \
                    uint8_t u8Ch, bool value)
{
    DMA_ChannelCmd(pstcDmaReg, u8Ch, value);
}

