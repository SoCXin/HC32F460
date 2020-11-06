#include "Hw_SPI4.h"

#define SPI4_TX_IRQn            Int008_IRQn
#define SPI4_RX_IRQn            Int009_IRQn
#define SPI4_ERR_IRQn           Int010_IRQn
#define SPI4_IDEL_IRQn          Int011_IRQn
#define DMA1_CH0_IRQn           Int012_IRQn
#define DMA1_CH1_IRQn           Int013_IRQn
//unsigned char SPI4_DATA;
/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE
/* SPI_SCK Port/Pin definition */
#define SPI4_SCK_PORT                    PortC
#define SPI4_SCK_PIN                     Pin13
#define SPI4_SCK_FUNC                    Func_Spi4_Sck

/* SPI_NSS Port/Pin definition */
#define SPI4_NSS_PORT                    PortH
#define SPI4_NSS_PIN                     Pin02
#define SPI4_NSS_FUNC                    Func_Spi4_Nss0

/* SPI_MOSI Port/Pin definition */
#define SPI4_MOSI_PORT                   PortB
#define SPI4_MOSI_PIN                    Pin09
#define SPI4_MOSI_FUNC                   Func_Spi4_Mosi

/* SPI_MISO Port/Pin definition */
#define SPI4_MISO_PORT                   PortB
#define SPI4_MISO_PIN                    Pin08
#define SPI4_MISO_FUNC                   Func_Spi4_Miso

/* SPI unit and clock definition */
#define SPI4_UNIT                        M4_SPI4
#define SPI4_UNIT_CLOCK                  PWC_FCG1_PERIPH_SPI4
#define SPI4_TX_INT_SOURCE               INT_SPI4_SRTI
#define SPI4_RX_INT_SOURCE               INT_SPI4_SRRI
#define SPI4_ERR_INT_SOURCE              INT_SPI4_SPEI
#define SPI4_ERR_IDEL_SOURCE             INT_SPI4_SPII

#define SPI4_DMA_UNIT2                (M4_DMA2)
#define SPI4_DMA_CH                  (DmaCh0)
#define SPI4_TX_DMA_CH                  (DmaCh1)
#define SPI4_DMA_CLK                 PWC_FCG0_PERIPH_DMA2
#define SPI4_DMA_TRNCNT              (50u)//传输次数
#define DMA_BLKSIZE             (1u)
#define SPI4_DMA_RPT_SIZE            (50u)
#define DMA_INT_SRC             INT_DMA2_BTC0
#define DMA_Trg_Src             EVT_SPI4_SRRI
#define DMA_TX_Trg_Src          EVT_SPI4_SRTI
uint8_t SPI4_RX_Data;
bool flag_SPI4_RX, flag_SPI4_TX;
uint8_t SPI4_RXbuff[SPI4_DMA_TRNCNT],spi4_rx_counter;
uint8_t *p_data = NULL,pos,tx_len;
void Hw_SPI4_TX_Callback(void)
{  
//    SPI_SendData8(SPI4_UNIT,0xFF);
//    flag_SPI4_TX = true;
	static uint8_t data;
	if(p_data == NULL)
	{
		return;
	}
	if(pos>=tx_len)
	{
		pos = 0;
		p_data = NULL;
//		SPI_IrqCmd(SPI4_UNIT, SpiIrqSend, Disable);
		return;
	}
	while(SPI4_UNIT->SR_f.TDEF == 0);
	SPI4_UNIT->DR = data= *(p_data+pos);
		pos++;
}
void Hw_SPI4_RX_Callback(void)
{   
    SPI4_RX_Data = SPI_ReceiveData8(SPI4_UNIT);
}
void Hw_SPI4_IDEL_Callback(void)
{
    spi4_rx_counter = 0;;
}
void Hw_SPI4_ERR_Callback(void)
{
    if(SPI4_UNIT->SR_f.MODFERF)//模式故障
    {
        SPI4_UNIT->SR_f.MODFERF = 0;
    }
    if(SPI4_UNIT->SR_f.UDRERF)//欠载错误
    {
        SPI4_UNIT->SR_f.UDRERF = 0;
    }
    if(SPI4_UNIT->SR_f.PERF)//奇偶校验错误
    {
        SPI4_UNIT->SR_f.PERF = 0;
    }
    if(SPI4_UNIT->SR_f.OVRERF)//过载错误
    {
        SPI4_UNIT->SR_f.OVRERF = 0;
    }
    SPI4_UNIT->CR1_f.SPE = 1;
}
void Hw_SPI4_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
//    stc_port_init_t Port_CFG;
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
//    MEM_ZERO_STRUCT(Port_CFG);
    
    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI4_UNIT_CLOCK, Enable);

//    Port_CFG.enPinMode = Pin_Mode_Out;
//	PORT_Init(SPI4_NSS_PORT, SPI4_NSS_PIN, &Port_CFG);
//    PORT_SetBits(SPI4_NSS_PORT,SPI4_NSS_PIN);
    /* Configuration SPI pin */
    PORT_SetFunc(SPI4_SCK_PORT, SPI4_SCK_PIN, SPI4_SCK_FUNC, Disable);
//    PORT_SetFunc(SPI4_NSS_PORT, SPI4_NSS_PIN, SPI4_NSS_FUNC, Disable);
    PORT_SetFunc(SPI4_MOSI_PORT, SPI4_MOSI_PIN, SPI4_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI4_MISO_PORT, SPI4_MISO_PIN, SPI4_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv8;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode3Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;

#ifdef SPI_MASTER_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayTypicalSck1;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayTypicalSck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalTypicalSck1PlusPck2;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck1PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif
    SPI_Init(SPI4_UNIT, &stcSpiInit);

    /* SPI3 tx interrupt */
    stcIrqRegiConf.enIntSrc = SPI4_TX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI4_TX_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI4_TX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);//使能发送中断
    /* Enable software trigger interrupt */
//    enIntEnable(Int5);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* SPI3 rx interrupt */
    stcIrqRegiConf.enIntSrc = SPI4_RX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI4_RX_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI4_RX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* SPI3 Error interrupt */
    stcIrqRegiConf.enIntSrc = SPI4_ERR_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI4_ERR_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI4_ERR_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* SPI3 idel interrupt */
    stcIrqRegiConf.enIntSrc = SPI4_ERR_IDEL_SOURCE;
    stcIrqRegiConf.enIRQn = SPI4_IDEL_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI4_IDEL_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* Enable SPI */
    SPI_Cmd(SPI4_UNIT, Enable);
    Ddl_Delay1ms(10);
//    SPI_IrqCmd(SPI4_UNIT, SpiIrqReceive, Enable);
    SPI_IrqCmd(SPI4_UNIT, SpiIrqSend, Enable);
    SPI_IrqCmd(SPI4_UNIT, SpiIrqError, Enable);
//    SPI_IrqCmd(SPI_UNIT, SpiIrqIdel, Enable);
//    SPI_Cmd(SPI_UNIT, Enable);
    Hw_SPI4_DMA_Init();//打开和配置DMA
//	Hw_SPI4_TX_DMA_Init();
}

void Hw_SPI4_TEST(void)
{	
    while(SPI4_UNIT->SR_f.TDEF == 0);
    SPI_SendData8(SPI4_UNIT,0x9A);	
}
void Hw_SPI4_DMA_Callback(void)
{
    DMA_ClearIrqFlag(SPI4_DMA_UNIT2,SPI4_DMA_CH, TrnCpltIrq);
    DMA_SetTransferCnt(SPI4_DMA_UNIT2,SPI4_DMA_CH,SPI4_DMA_TRNCNT);
    DMA_SetDesAddress(SPI4_DMA_UNIT2,SPI4_DMA_CH,(uint32_t)(SPI4_RXbuff));
    DMA_ChannelCmd(SPI4_DMA_UNIT2, SPI4_DMA_CH,Enable); 
}
void Hw_SPI4_TX_DMA_Callback(void)
{
    DMA_ClearIrqFlag(SPI4_DMA_UNIT2,SPI4_DMA_CH, TrnCpltIrq);
//    DMA_SetTransferCnt(SPI4_DMA_UNIT2,SPI4_DMA_CH,SPI4_DMA_TRNCNT);
//    DMA_SetDesAddress(SPI4_DMA_UNIT2,SPI4_DMA_CH,(uint32_t)(SPI4_RXbuff));
//    DMA_ChannelCmd(SPI4_DMA_UNIT2, SPI4_DMA_CH,Enable); 
}
void Hw_SPI4_DMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;//
    stcDmaCfg.u16TransferCnt = SPI4_DMA_TRNCNT;//
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&SPI4_RXbuff[0]);//(&DMA0_Dre_Data[0]);//Target Address
    stcDmaCfg.u32SrcAddr = (uint32_t)(&(SPI4_UNIT->DR));//USART2_DR_ADDRESS;//(uint32_t)(&DMA0_Src_data[0]);//Source Address
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = 1;
    stcDmaCfg.u16DesRptSize = SPI4_DMA_RPT_SIZE;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Enable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressFix;//地址不变
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

//    M4_MSTP->FCG0PC = 0xA5A50001;
//    M4_MSTP->FCG0_f.DMA1 = Reset;
//    M4_MSTP->FCG0PC = 0xA5A50000;
    PWC_Fcg0PeriphClockCmd(SPI4_DMA_CLK, Enable);
   
    /* Enable DMA1. */
    DMA_Cmd(SPI4_DMA_UNIT2,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(SPI4_DMA_UNIT2, SPI4_DMA_CH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(SPI4_DMA_UNIT2, SPI4_DMA_CH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(SPI4_DMA_UNIT2, SPI4_DMA_CH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA2_TC0;
    stcIrqRegiConf.enIRQn = DMA1_CH0_IRQn;
    stcIrqRegiConf.pfnCallback =  Hw_SPI4_DMA_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS,Enable);
    
    DMA_SetTriggerSrc(SPI4_DMA_UNIT2,SPI4_DMA_CH,DMA_Trg_Src);
       
//    M4_AOS->INT_SFTTRG_f.STRG = 1;
//    
//    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
//    {
//        M4_AOS->INT_SFTTRG_f.STRG = 1;
//    }
}

void Hw_SPI4_TX_DMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;//
    stcDmaCfg.u16TransferCnt = SPI4_DMA_TRNCNT;//
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&(SPI4_UNIT->DR));//(&DMA0_Dre_Data[0]);//Target Address
    stcDmaCfg.u32SrcAddr = (uint32_t)(&SPI4_RXbuff[0]);//USART2_DR_ADDRESS;//(uint32_t)(&DMA0_Src_data[0]);//Source Address
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = SPI4_DMA_RPT_SIZE;
    stcDmaCfg.u16DesRptSize = 1;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;//地址不变
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    PWC_Fcg0PeriphClockCmd(SPI4_DMA_CLK, Enable);
   
    /* Enable DMA1. */
    DMA_Cmd(SPI4_DMA_UNIT2,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(SPI4_DMA_UNIT2, SPI4_TX_DMA_CH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(SPI4_DMA_UNIT2, SPI4_TX_DMA_CH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(SPI4_DMA_UNIT2, SPI4_TX_DMA_CH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA2_TC1;
    stcIrqRegiConf.enIRQn = DMA1_CH1_IRQn;
    stcIrqRegiConf.pfnCallback =  Hw_SPI4_TX_DMA_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS,Enable);
    
    DMA_SetTriggerSrc(SPI4_DMA_UNIT2,SPI4_TX_DMA_CH,DMA_TX_Trg_Src);  
}
void SPI4_WriteBuffer_DMA(uint8_t *pdata,uint8_t Len)
{
    DMA_SetTransferCnt(SPI4_DMA_UNIT2,SPI4_TX_DMA_CH,Len);
    DMA_SetSrcAddress(SPI4_DMA_UNIT2,SPI4_TX_DMA_CH,(uint32_t)(pdata));
    DMA_ChannelCmd(SPI4_DMA_UNIT2, SPI4_TX_DMA_CH,Enable); 		
	SPI4_UNIT->CR1_f.SPE = 0;
	SPI4_UNIT->CR1_f.SPE = 1;
}
void SPI4_WriteBuffer(uint8_t *pdata,uint8_t Len)
{
	for(int i= 0; i<Len;i++)
	{
		while(SPI4_UNIT->SR_f.TDEF == 0);
		SPI_SendData8(SPI4_UNIT,pdata[i]);	
	}   
}
void SPI4_WriteBuffer_INT(uint8_t *pdata,uint8_t Len)
{
	p_data = pdata;
	tx_len = Len;
	pos = 1;
	SPI4_UNIT->DR= *pdata;
}