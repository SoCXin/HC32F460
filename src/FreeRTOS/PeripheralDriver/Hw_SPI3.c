#include "Hw_SPI3.h"

uint8_t SPI3_RX_Data;
bool flag_SPI3_RX, flag_SPI3_TX;
uint8_t SPI3_RXbuff[SPI3_DMA_TRNCNT],SPI3_TXbuff[SPI3_DMA_TRNCNT],spi3_rx_counter;
void Hw_SPI3_TX_Callback(void)
{  
    SPI_SendData8(SPI3_UNIT,0xFF);
    flag_SPI3_TX = true;
}
void Hw_SPI3_RX_Callback(void)
{   
//    SPI_SetReadDataRegObject(SPI_UNIT,SpiReadReceiverBuffer);
    SPI3_RX_Data = SPI_ReceiveData8(SPI3_UNIT);
    if(SPI3_RX_Data == 0x55 && spi3_rx_counter > 1)
    {
        if(SPI3_RXbuff[1] != 0x22)
        {
            spi3_rx_counter = 0;
        }       
    }
    SPI3_RXbuff[spi3_rx_counter++] = SPI3_RX_Data;
    if(spi3_rx_counter>=50)
    {
        spi3_rx_counter = 0;
    }
    flag_SPI3_RX = true;
}
void Hw_SPI3_IDEL_Callback(void)
{
//    SPI_UNIT->SR_f.IDLNF = 1;
    flag_SPI3_RX = 0;;
}
void Hw_SPI3_ERR_Callback(void)
{
    if(SPI3_UNIT->SR_f.MODFERF)//模式故障
    {
        SPI3_UNIT->SR_f.MODFERF = 0;
    }
    if(SPI3_UNIT->SR_f.UDRERF)//欠载错误
    {
        SPI3_UNIT->SR_f.UDRERF = 0;
    }
    if(SPI3_UNIT->SR_f.PERF)//奇偶校验错误
    {
        SPI3_UNIT->SR_f.PERF = 0;
    }
    if(SPI3_UNIT->SR_f.OVRERF)//过载错误
    {
        SPI3_UNIT->SR_f.OVRERF = 0;
    }
    SPI3_UNIT->CR1_f.SPE = 1;
}
void Hw_SPI3_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
	stc_port_init_t Port_CFG;
    /* configuration structure initialization */
	MEM_ZERO_STRUCT(Port_CFG);
    MEM_ZERO_STRUCT(stcSpiInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
	Port_CFG.enPinMode = Pin_Mode_Out;
    Port_CFG.enPullUp = Enable;
    Port_CFG.enPinOType = Pin_OType_Od;
	PORT_Init(SPI3_NSS_PORT, SPI3_NSS_PIN, &Port_CFG);
    Port_CFG.enPinOType = Pin_OType_Cmos;
    PORT_Init(SPI3_MOSI_PORT, SPI3_MOSI_PIN, &Port_CFG);
    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI3_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI3_SCK_PORT, SPI3_SCK_PIN, SPI3_SCK_FUNC, Disable);
//    PORT_SetFunc(SPI3_NSS_PORT, SPI3_NSS_PIN, SPI3_NSS_FUNC, Disable);
    PORT_SetFunc(SPI3_MOSI_PORT, SPI3_MOSI_PIN, SPI3_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI3_MISO_PORT, SPI3_MISO_PIN, SPI3_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv128;//SPI datarate,PCLK1/128
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode4Line;
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
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck1PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif
    SPI_Init(SPI3_UNIT, &stcSpiInit);

    /* SPI3 tx interrupt */
    stcIrqRegiConf.enIntSrc = SPI3_TX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_TX_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI3_TX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Enable software trigger interrupt */
//    enIntEnable(Int5);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* SPI3 rx interrupt */
    stcIrqRegiConf.enIntSrc = SPI3_RX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_RX_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI3_RX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* SPI3 Error interrupt */
    stcIrqRegiConf.enIntSrc = SPI3_ERR_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_ERR_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI3_ERR_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* SPI3 idel interrupt */
    stcIrqRegiConf.enIntSrc = SPI3_ERR_IDEL_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_IDEL_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI3_IDEL_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* Enable SPI */
    SPI_Cmd(SPI3_UNIT, Enable);
//    SPI_UNIT->CR1_f.SPE = 1;
    Ddl_Delay1ms(10);
    
    SPI_IrqCmd(SPI3_UNIT, SpiIrqReceive, Enable);
    SPI_IrqCmd(SPI3_UNIT, SpiIrqSend, Enable);
    SPI_IrqCmd(SPI3_UNIT, SpiIrqError, Enable);
//    SPI_IrqCmd(SPI_UNIT, SpiIrqIdel, Enable);
//    SPI_Cmd(SPI_UNIT, Enable);
//    Hw_SPI3_RxDMA_Init();
//	Hw_SPI3_TxDMA_Init();
}
void Hw_SPI3_Init1(void)
{
    PORT_SetFunc(SPI3_SCK_PORT, SPI3_SCK_PIN, SPI3_MOSI_FUNC, Disable);//SPI3_SCK_FUNC
    PORT_SetFunc(SPI3_NSS_PORT, SPI3_NSS_PIN, SPI3_MISO_FUNC, Disable);
    PORT_SetFunc(SPI3_MOSI_PORT, SPI3_MOSI_PIN, Func_Gpio, Disable);//SPI3_MOSI_FUNC//CS
    PORT_SetFunc(SPI3_MISO_PORT, SPI3_MISO_PIN, SPI3_SCK_FUNC, Disable);//SPI3_MISO_FUNC
}
void Hw_SPI3_Init2(void)
{
    PORT_SetFunc(SPI3_SCK_PORT, SPI3_SCK_PIN, SPI3_SCK_FUNC, Disable);
    PORT_SetFunc(SPI3_NSS_PORT, SPI3_NSS_PIN, Func_Gpio, Disable);
    PORT_SetFunc(SPI3_MOSI_PORT, SPI3_MOSI_PIN, SPI3_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI3_MISO_PORT, SPI3_MISO_PIN, SPI3_MISO_FUNC, Disable);
}
void Hw_SPI3_TEST(void)
{	
    while(SPI3_UNIT->SR_f.TDEF == 0);
    SPI_SendData8(SPI3_UNIT,0x55);	
}
void SPI3_DMA_RX_Callback(void)
{
	while(SPI3_UNIT->SR_f.IDLNF ==1);
	PORT_SetBits(SPI3_NSS_PORT,SPI3_NSS_PIN);
	flag_SPI3_RX = true;
//    DMA_ClearIrqFlag(SPI3_DMA_UNIT2,SPI3_DMA_RxCH, TrnCpltIrq);
//    DMA_SetTransferCnt(SPI3_DMA_UNIT2,SPI3_DMA_RxCH,SPI3_DMA_TRNCNT);
//    DMA_SetDesAddress(SPI3_DMA_UNIT2,SPI3_DMA_RxCH,(uint32_t)(SPI3_RXbuff));
//    DMA_ChannelCmd(SPI3_DMA_UNIT2, SPI3_DMA_RxCH,Enable); 
}
void SPI3_DMA_TX_Callback(void)
{
//    DMA_ClearIrqFlag(SPI3_DMA_UNIT2,SPI3_DMA_TxCH, TrnCpltIrq);
//    DMA_SetTransferCnt(SPI3_DMA_UNIT2,SPI3_DMA_TxCH,SPI3_DMA_TRNCNT);
//    DMA_SetSrcAddress(SPI3_DMA_UNIT2,SPI3_DMA_TxCH,(uint32_t)(SPI3_TXbuff));
//    DMA_ChannelCmd(SPI3_DMA_UNIT2, SPI3_DMA_TxCH,Enable); 
}
void Hw_SPI3_RxDMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;//
    stcDmaCfg.u16TransferCnt = SPI3_DMA_TRNCNT;//
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&SPI3_RXbuff[0]);//
    stcDmaCfg.u32SrcAddr = (uint32_t)(&(SPI3_UNIT->DR));//
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = 1;
    stcDmaCfg.u16DesRptSize = SPI3_DMA_RPT_SIZE;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressFix;//地址不变
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    PWC_Fcg0PeriphClockCmd(SPI3_DMA_CLK, Enable);
   
    /* Enable DMA1. */
    DMA_Cmd(SPI3_DMA_UNIT2,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(SPI3_DMA_UNIT2, SPI3_DMA_RxCH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
//    DMA_ChannelCmd(SPI3_DMA_UNIT2, SPI3_DMA_RxCH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(SPI3_DMA_UNIT2, SPI3_DMA_RxCH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = DMA_INT_RXSRC;
    stcIrqRegiConf.enIRQn = DMA2_CH0_IRQn;
    stcIrqRegiConf.pfnCallback =  SPI3_DMA_RX_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    
    DMA_SetTriggerSrc(SPI3_DMA_UNIT2,SPI3_DMA_RxCH,DMA_RXTrg_Src);
       
}
void Hw_SPI3_TxDMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;//
    stcDmaCfg.u16TransferCnt = SPI3_DMA_TRNCNT;//
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&(SPI3_UNIT->DR));//
    stcDmaCfg.u32SrcAddr = (uint32_t)(&SPI3_TXbuff[0]);//
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = 1;
    stcDmaCfg.u16DesRptSize = SPI3_DMA_RPT_SIZE;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;//地址不变
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    PWC_Fcg0PeriphClockCmd(SPI3_DMA_CLK, Enable);
   
    /* Enable DMA1. */
    DMA_Cmd(SPI3_DMA_UNIT2,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(SPI3_DMA_UNIT2, SPI3_DMA_TxCH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
//    DMA_ChannelCmd(SPI3_DMA_UNIT2, SPI3_DMA_TxCH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(SPI3_DMA_UNIT2, SPI3_DMA_TxCH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = DMA_INT_TXSRC;
    stcIrqRegiConf.enIRQn = DMA2_CH1_IRQn;
    stcIrqRegiConf.pfnCallback =  SPI3_DMA_TX_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    
    DMA_SetTriggerSrc(SPI3_DMA_UNIT2,SPI3_DMA_TxCH,DMA_TXTrg_Src);
       
}
void SPI_TX_8bit(uint8_t *rxdata,uint8_t *data,uint8_t len)
{
	PORT_ResetBits(SPI3_NSS_PORT,SPI3_NSS_PIN);
	for(int i= 0;i<len;i++)
	{
		while(SPI3_UNIT->SR_f.TDEF == 0);
		SPI_SendData8(SPI3_UNIT,data[i]);
		while(SPI3_UNIT->SR_f.RDFF == 0);
		rxdata[i] = SPI_ReceiveData8(SPI3_UNIT);
		
	}
	while(SPI3_UNIT->SR_f.IDLNF ==1);
	PORT_SetBits(SPI3_NSS_PORT,SPI3_NSS_PIN);
}
void Hw_SPI3_DMA_START(void)
{
	SPI_Cmd(SPI3_UNIT, Disable);
	SPI_Cmd(SPI3_UNIT, Enable);
}