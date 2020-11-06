#include "hc32_ddl.h"
#include "User_Uart.h"
#include "User_ADC.h"
#include "System_InterruptCFG_Def.h"

#define DMA_UNIT                (M4_DMA1)
#define DMA_CH                  (DmaCh1)
#define DMA_TRNCNT              (1000u)//传输次数
#define DMA_BLKSIZE             (1u)
#define DMA_RPT_SIZE            (1000u)
#define USART2_DR_ADDRESS        0x4001D404
uint32_t DMA0_Dre_Data[250], DMA0_Src_data[250];
uint16_t Dcu1_result;
uint16_t Get_DCU1_Result(void)
{
    return Dcu1_result;
}
void DMA1_CH0_Callback(void)
{
    //M4_DMA1->DTCTL0_f.CNT = DMA_TRNCNT;
    Set_ADC_Data(M4_DCU1->DATA0);
    Dcu1_result = M4_DCU1->DATA0;
    M4_DCU1->DATA0 = 1000;
}

void User_DMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;//
    stcDmaCfg.u16TransferCnt = 0;//
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&M4_DCU1->DATA1);//(&DMA0_Dre_Data[0]);//Target Address
    stcDmaCfg.u32SrcAddr = (uint32_t)(&(M4_ADC1->DR10));//USART2_DR_ADDRESS;//(uint32_t)(&DMA0_Src_data[0]);//Source Address
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = DMA_RPT_SIZE;
    stcDmaCfg.u16DesRptSize = DMA_RPT_SIZE;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Enable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressFix;//地址不变
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma16Bit;

//    M4_MSTP->FCG0PC = 0xA5A50001;
//    M4_MSTP->FCG0_f.DMA1 = Reset;
//    M4_MSTP->FCG0PC = 0xA5A50000;
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1, Enable);
    /* Enable DMA1. */
    DMA_Cmd(DMA_UNIT,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA1_BTC0;
    stcIrqRegiConf.enIRQn = DMA1_CH0_IRQn;
    stcIrqRegiConf.pfnCallback =  DMA1_CH0_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    
    DMA_SetTriggerSrc(DMA_UNIT,DMA_CH,EVT_ADC1_EOCA);
       
//    M4_AOS->INT_SFTTRG_f.STRG = 1;
//    
//    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
//    {
//        M4_AOS->INT_SFTTRG_f.STRG = 1;
//    }
}
void GetDMA_CNT(void)
{
    M4_DMA1->MONDNSEQCTL0_f.DNSCNT;
}
