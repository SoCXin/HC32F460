#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

unsigned char SPI_DATA;
/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE
/* SPI_SCK Port/Pin definition */
#define SPI_SCK_PORT                    PortE
#define SPI_SCK_PIN                     Pin00
#define SPI_SCK_FUNC                    Func_Spi3_Sck

/* SPI_NSS Port/Pin definition */
#define SPI_NSS_PORT                    PortE
#define SPI_NSS_PIN                     Pin01
#define SPI_NSS_FUNC                    Func_Spi3_Nss0

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   PortE
#define SPI_MOSI_PIN                    Pin02
#define SPI_MOSI_FUNC                   Func_Spi3_Mosi

/* SPI_MISO Port/Pin definition */
#define SPI_MISO_PORT                   PortE
#define SPI_MISO_PIN                    Pin03
#define SPI_MISO_FUNC                   Func_Spi3_Miso

/* SPI unit and clock definition */
#define SPI_UNIT                        M4_SPI3
#define SPI_UNIT_CLOCK                  PWC_FCG1_PERIPH_SPI3
#define SPI_TX_INT_SOURCE               INT_SPI3_SRTI
#define SPI_RX_INT_SOURCE               INT_SPI3_SRRI
#define SPI_ERR_INT_SOURCE              INT_SPI3_SPEI
#define SPI_ERR_IDEL_SOURCE             INT_SPI3_SPII
uint8_t SPI1_RX_Data;
bool flag_SPI3_RX, flag_SPI3_TX;
uint8_t RXbuff[50],counter;
void USER_SPI3_TX_Callback(void)
{  
    SPI_SendData8(SPI_UNIT,0xFF);
    flag_SPI3_TX = true;
}
void USER_SPI3_RX_Callback(void)
{   
//    SPI_SetReadDataRegObject(SPI_UNIT,SpiReadReceiverBuffer);
    SPI1_RX_Data = SPI_ReceiveData8(SPI_UNIT);
    if(SPI1_RX_Data == 0x55 && counter > 1)
    {
        if(RXbuff[1] != 0x22)
        {
            counter = 0;
        }       
    }
    RXbuff[counter++] = SPI1_RX_Data;
    if(counter>=50)
    {
        counter = 0;
    }
    flag_SPI3_RX = true;
}
void USER_SPI3_IDEL_Callback(void)
{
//    SPI_UNIT->SR_f.IDLNF = 1;
    counter = 0;;
}
void USER_SPI3_ERR_Callback(void)
{
    if(SPI_UNIT->SR_f.MODFERF)//Ä£Ê½¹ÊÕÏ
    {
        SPI_UNIT->SR_f.MODFERF = 0;
    }
    if(SPI_UNIT->SR_f.UDRERF)//Ç·ÔØ´íÎó
    {
        SPI_UNIT->SR_f.UDRERF = 0;
    }
    if(SPI_UNIT->SR_f.PERF)//ÆæÅ¼Ð£Ñé´íÎó
    {
        SPI_UNIT->SR_f.PERF = 0;
    }
    if(SPI_UNIT->SR_f.OVRERF)//¹ýÔØ´íÎó
    {
        SPI_UNIT->SR_f.OVRERF = 0;
    }
    SPI_UNIT->CR1_f.SPE = 1;
}
void User_SPI_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(SPI_NSS_PORT, SPI_NSS_PIN, SPI_NSS_FUNC, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);
//    PORT_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv4;
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
    SPI_Init(SPI_UNIT, &stcSpiInit);

    /* SPI3 tx interrupt */
    stcIrqRegiConf.enIntSrc = SPI_TX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_TX_IRQn;
    stcIrqRegiConf.pfnCallback = USER_SPI3_TX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Enable software trigger interrupt */
//    enIntEnable(Int5);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* SPI3 rx interrupt */
    stcIrqRegiConf.enIntSrc = SPI_RX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_RX_IRQn;
    stcIrqRegiConf.pfnCallback = USER_SPI3_RX_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* SPI3 Error interrupt */
    stcIrqRegiConf.enIntSrc = SPI_ERR_INT_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_ERR_IRQn;
    stcIrqRegiConf.pfnCallback = USER_SPI3_ERR_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* SPI3 idel interrupt */
    stcIrqRegiConf.enIntSrc = SPI_ERR_IDEL_SOURCE;
    stcIrqRegiConf.enIRQn = SPI3_IDEL_IRQn;
    stcIrqRegiConf.pfnCallback = USER_SPI3_IDEL_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
	
//    SPI_GeneralLoopbackCmd(SPI_UNIT,Enable);
    /* Enable SPI */
//    SPI_Cmd(SPI_UNIT, Enable);
//    SPI_UNIT->CR1_f.SPE = 1;
    Ddl_Delay1ms(10);
//    SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Enable);
//    SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Enable);
    SPI_IrqCmd(SPI_UNIT, SpiIrqError, Enable);
	
//    SPI_IrqCmd(SPI_UNIT, SpiIrqIdel, Enable);
    SPI_Cmd(SPI_UNIT, Enable);
    
}

void USER_SPI_TEST(void)
{	
    SPI_SendData8(SPI_UNIT,0x55);	
}
void SPI_Writedata(uint8_t data)
{	
	SPI_SendData8(SPI_UNIT,data);
	while(PORT_GetBit(SPI_NSS_PORT,SPI_NSS_PIN) ==  0);
//	while(SPI_UNIT->SR_f.TDEF == 0);
//	Ddl_Delay1us(2);
}
uint8_t SPI_ReadData(void)
{
	return SPI_ReceiveData8(SPI_UNIT);
}
uint8_t SPIx_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;
	while(SPI_UNIT->SR_f.TDEF == 0)
	{
		retry++;
		if(retry>200)
			return 0;
	}
	SPI_SendData8(SPI_UNIT,TxData);
	while(SPI_UNIT->SR_f.TDEF == 0)
	{
		;
	}
	retry = 0;
	while(SPI_UNIT->SR_f.RDFF == 0)
	{
		retry++;
		if(retry>200)
			return 0;
	}	
	return SPI_ReceiveData8(SPI_UNIT);
}
