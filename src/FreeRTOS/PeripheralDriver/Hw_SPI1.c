#include "Hw_SPI1.h"
//#include "System_InterruptCFG_Def.h"
uint8_t SPI1_RX_Data;
bool flag_SPI3_RX, flag_SPI3_TX;
uint8_t RXbuff[SPI3_TRNCNT];
uint8_t TXbuff[SPI3_TRNCNT];
uint8_t DummyByte = 0x55;
void Prepare_data(void)
{
    uint32_t i;
    for(i=0;i<SPI3_TRNCNT;i++)
    {
        TXbuff[i] = i;
    }
}
void Hw_SPI3_TX_Callback(void)
{  
#ifdef SPI_SLAVE_MODE    
    M4_SPI3->DR = 0xFF;
#endif
}
void Hw_SPI3_RX_Callback(void)
{   
    //SPI_IrqCmd(SPI1_UNIT, SpiIrqReceive, Disable);//¹Ø±ÕÖÐ¶Ï
    DummyByte = SPI3_UNIT->DR;
}
void Hw_SPI3_IDEL_Callback(void)
{
    SPI_IrqCmd(SPI3_UNIT, SpiIrqIdel, Disable);
}
void Hw_SPI3_ERR_Callback(void)
{
    if(SPI3_UNIT->SR_f.MODFERF)//Ä£Ê½¹ÊÕÏ
    {
        SPI3_UNIT->SR_f.MODFERF = 0;
    }
    if(SPI3_UNIT->SR_f.UDRERF)//Ç·ÔØ´íÎó
    {
        SPI3_UNIT->SR_f.UDRERF = 0;
    }
    if(SPI3_UNIT->SR_f.PERF)//ÆæÅ¼Ð£Ñé´íÎó
    {
        SPI3_UNIT->SR_f.PERF = 0;
    }
    if(SPI3_UNIT->SR_f.OVRERF)//¹ýÔØ´íÎó
    {
        SPI3_UNIT->SR_f.OVRERF = 0;
    }
    SPI3_UNIT->CR1_f.SPE = 1;
}
void Hw_SPI3_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    Prepare_data();
    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI3_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI3_SCK_PORT, SPI3_SCK_PIN, SPI3_SCK_FUNC, Disable);
    PORT_SetFunc(SPI3_NSS_PORT, SPI3_NSS_PIN, SPI3_NSS_FUNC, Disable);
    PORT_SetFunc(SPI3_MOSI_PORT, SPI3_MOSI_PIN, SPI3_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI3_MISO_PORT, SPI3_MISO_PIN, SPI3_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = CLK_DIVISION;
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
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif
    SPI_Init(SPI3_UNIT, &stcSpiInit);
    
#ifdef SPI_SLAVE_MODE
    /* SPI3 tx interrupt */
    stcIrqRegiConf.enIntSrc = SPI1_TX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI1_TX_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI1_TX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Enable software trigger interrupt */
//    enIntEnable(Int5);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
#endif
    /* SPI3 rx interrupt */
    stcIrqRegiConf.enIntSrc = SPI3_RX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_RX_IRQn;
    stcIrqRegiConf.pfnCallback = Hw_SPI3_RX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
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
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* Enable SPI */
    SPI_IrqCmd(SPI3_UNIT, SpiIrqReceive, Enable);
    SPI_IrqCmd(SPI3_UNIT, SpiIrqSend, Enable);
    SPI_IrqCmd(SPI3_UNIT, SpiIrqError, Enable);
    SPI_IrqCmd(SPI3_UNIT, SpiIrqIdel, Enable);
    SPI_Cmd(SPI3_UNIT, Enable);
    
}

void Hw_SPI3_TEST(void)
{
    while(SPI3_UNIT->SR_f.TDEF == 0);
    SPI_SendData8(SPI3_UNIT,0x55);	
}



