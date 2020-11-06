#include "Hw_I2C_Slave_dma.h"
uint8_t SlaveRxBuf[SLAVE_DATA_LEN];
uint8_t SlaveTxBuf[133][SLAVE_DATA_LEN];
uint16_t rec_len;
uint32_t u32DataInOffset = 0ul;
uint32_t u32DataOutOffset = 0ul;
uint8_t u8FinishFlag = 0u;
static uint8_t slave_regaddr;

/**
 *******************************************************************************
 ** \brief static function for buffer write.
 **
 ** \param  u8Data the data to be write.
 **
 ** \retval None
 **
 ******************************************************************************/
static void BufWrite(uint8_t u8Data)//接收主机发过来的数据
{
    if(u32DataInOffset>SLAVE_DATA_LEN)
	{
		u32DataInOffset = 0;
	}
    SlaveRxBuf[u32DataInOffset] = u8Data;
//    if(u32DataInOffset == 0)
//    {
//        if(SlaveRxBuf[0] <=64)
//            rec_len = SlaveRxBuf[0];
//    }
//    if(u32DataInOffset == 1)
//    {
//        if(SlaveRxBuf[1] <133)
//            slave_regaddr = SlaveRxBuf[1];
//    }
    u32DataInOffset++;	
}

/**
 *******************************************************************************
 ** \brief static function for buffer read.
 **
 ** \param  void.
 **
 ** \retval uint8_t the data read from the buffer.
 **
 ******************************************************************************/
static uint8_t BufRead(void)//读取当前要往主机写的数据
{
    uint8_t temp;
    if(u32DataOutOffset>SLAVE_DATA_LEN)
	{
		u32DataOutOffset = 0;
	}
    temp = SlaveTxBuf[slave_regaddr][u32DataOutOffset++];	
    return temp;
}
/**
 *******************************************************************************
 ** \brief I2C EEI(communication error or event) interrupt callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void I2C_EEI_Callback(void)
{
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_STARTF))
    {
        M4_TMRA2->CNTER = 0;
        M4_TMRA2->BCSTR_f.START = 1;
    }
    /* If start interrupt occurred */
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_SLADDR0F))
    {
        I2C_ClearStatus(I2C_SLAVE_CH, I2C_CLR_SLADDR0FCLR | I2C_CLR_STOPFCLR| I2C_CLR_NACKFCLR);
        /* Enable Tx or Rx function*/
        if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_TRA))
        {
            /* Enable tx buffer empty interrupt function*/
            I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_TEMPTYIE, Enable);
            /* Write the first data to DTR immediately */
            //I2C_SendData(I2C_SLAVE_CH, BufRead());           
        }
        else
        {
            /* Config rx buffer full interrupt function*/
            I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_RFULLIE, Enable);
        }
        M4_TMRA1->BCSTR_f.START = 0;
        M4_TMRA1->CNTER = 0;
        M4_TMRA1->BCSTR_f.START = 1;        
        printf("s\r\n");
        /* Enable stop and NACK interrupt */
        I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_STOPIE | I2C_CR2_NACKIE | I2C_CR2_TMOURIE, Enable);
    }

    /* If NACK interrupt occurred */
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_NACKSENDF))
    {
        /* clear NACK flag*/
        I2C_ClearStatus(I2C_SLAVE_CH, I2C_CLR_NACKFCLR);
        /* Stop tx or rx process*/
        if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_TRA))
        {
            /* Config tx buffer empty interrupt function disable*/
            I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_TEMPTYIE, Disable);
            /* Read DRR register to release SCL*/
            I2C_ReadData(I2C_SLAVE_CH);
            I2C_SLAVE_CH->CR1_f.SWRST = 1;
            I2C_SLAVE_CH->CR1_f.SWRST = 0;
        }
        else
        {
            /* Config rx buffer full interrupt function disable */
            I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_RFULLIE, Disable);
        }
    }

    /* If stop interrupt occurred */
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_STOPF))
    {
        /* Disable all interrupt enable flag except SLADDR0IE*/
        I2C_IntCmd(I2C_SLAVE_CH,                                                     \
                   I2C_CR2_TEMPTYIE | I2C_CR2_RFULLIE |         \
                   I2C_CR2_STOPIE | I2C_CR2_NACKIE | I2C_CR2_TMOURIE,                         \
                   Disable);
        /* Clear STOPF flag */
        I2C_ClearStatus(I2C_SLAVE_CH, I2C_CLR_STOPFCLR);
        u8FinishFlag = 1u;
        u32DataInOffset = 0;
        u32DataOutOffset = 0;
        bsp_dma_SetDesAddr(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_RX_CH, (uint32_t)SlaveRxBuf);
        bsp_dma_SetSrcAddr(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_TX_CH, (uint32_t)&SlaveTxBuf[0][0]);
        rec_len = TIMERA_GetCurrCount(M4_TMRA1); 
        M4_TMRA2->BCSTR_f.START = 0;
        printf("%d\r\n",rec_len);
//        do
//        {
//            M4_TMRA1->CNTER = 0;
//        }while(M4_TMRA1->CNTER != 0);
    }
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_TMOUTF))
    {
        printf("timeout\r\n");
    }
}


/**
 *******************************************************************************
 ** \brief I2C TEI(transfer buffer empty) interrupt callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void I2C_TEI_Callback(void)
{
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_TENDF))
    {
        /* Dummy read for release SCL */
        (void)I2C_ReadData(I2C_SLAVE_CH);
    }
}
/**
 * @brief   I2C TXI(transfer buffer empty) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_TXI_Callback(void)
{
     if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_TEMPTYF))
    {
//        if(u32DataOutOffset < rec_len)
//        {
          I2C_SendData(I2C_SLAVE_CH, BufRead());
//        }
//        else
//        {
//            I2C_SendData(I2C_SLAVE_CH, 0xFF);
//        }
    }
}
void Update_TxBuffer(uint8_t *data)
{
	for (uint32_t i = 0; i < SLAVE_DATA_LEN; i++)
	{
		SlaveTxBuf[slave_regaddr][i] = data[i];
	}		
}

/**
 *******************************************************************************
 ** \brief I2C RXI(receive buffer full) interrupt callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void I2C_RXI_Callback(void)
{
    if(Set == I2C_GetStatus(I2C_SLAVE_CH, I2C_SR_RFULLF))
    {
        BufWrite(I2C_ReadData(I2C_SLAVE_CH));
    }
}

/**
 ******************************************************************************
 ** \brief  Initialize the I2C peripheral for slave
 **
 ** \param  None
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Initialize failed
 **         - I2C_RET_OK     Initialize success
 ******************************************************************************/
uint8_t bsp_i2c_slave_init(M4_I2C_TypeDef* I2Cx)
{
    stc_i2c_init_t stcI2cInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_clk_freq_t stcClkFreq;
//    stc_clock_timeout_init_t i2c_timerout_cfg;
    
    I2C_DeInit(I2C_SLAVE_CH);

    /* Get system clock frequency */
    CLK_GetClockFreq(&stcClkFreq);

    MEM_ZERO_STRUCT(stcI2cInit);
//    MEM_ZERO_STRUCT(i2c_timerout_cfg);
    stcI2cInit.enI2cMode = I2cSlave;
    stcI2cInit.u32Pclk3 = stcClkFreq.pclk3Freq;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    I2C_Init(I2C_SLAVE_CH, &stcI2cInit);

    I2C_Cmd(I2C_SLAVE_CH, Enable);

    /* Set slave address*/
#ifdef I2C_10BITS_ADDRESS
    I2C_SlaveAdr0Config(I2C_SLAVE_CH, Enable, Adr10bit, SLAVE_ADDRESS);
#else
    I2C_SlaveAdr0Config(I2C_SLAVE_CH, Enable, Adr7bit, SLAVE_ADDRESS);
#endif
//    i2c_timerout_cfg.enClkTimeOutSwitch = LowTimerOutOn;//开启I2C时钟低电平超时检测
//    i2c_timerout_cfg.u16TimeOutHigh = 0x00FF;
//    i2c_timerout_cfg.u16TimeOutLow = 0x00FF;//超时时间65535个I2C时钟。
//    I2C_ClkTimeOutConfig(I2C_SLAVE_CH,&i2c_timerout_cfg);
//    I2C_SLAVE_CH->CR3_f.TMOUTEN = 1;//使能超时功能
//    I2C_SLAVE_CH->CR1_f.SMBUS = 1;//SMBUS模式
    I2C_IntCmd(I2C_SLAVE_CH,I2C_CR2_TMOURIE,Enable);
    /* Register EEI Int to Vect.No.001 */
    stcIrqRegiConf.enIRQn = Int001_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = INT_I2C1_EE1;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &I2C_EEI_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Register RXI Int to Vect.No.003 */
//    stcIrqRegiConf.enIRQn = Int003_IRQn;
//    /* Select I2C receive full interrupt function */
//    stcIrqRegiConf.enIntSrc = INT_I2C1_RXI;
//    /* Callback function */
//    stcIrqRegiConf.pfnCallback = &I2C_RXI_Callback;
//    /* Registration IRQ */
//    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
//    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    /* Set priority */
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Register TEI Int to Vect.No.002 */
//    stcIrqRegiConf.enIRQn = Int002_IRQn;
//    /* Select I2C TX buffer empty interrupt function */
//    stcIrqRegiConf.enIntSrc = INT_I2C1_TEI;
//    /* Callback function */
//    stcIrqRegiConf.pfnCallback = &I2C_TEI_Callback;
//    /* Registration IRQ */
//    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
//    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    /* Set priority */
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Register TEI Int to Vect.No.002 */
//    stcIrqRegiConf.enIRQn = Int004_IRQn;
//    /* Select I2C TX buffer empty interrupt function */
//    stcIrqRegiConf.enIntSrc = INT_I2C1_TXI;
//    /* Callback function */
//    stcIrqRegiConf.pfnCallback = &I2C_TXI_Callback;
//    /* Registration IRQ */
//    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
//    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    /* Set priority */
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* Config slave address match interrupt function*/
    I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_SLADDR0EN | I2C_CR2_STARTIE, Enable);
    I2C_SLAVE_CH->CR3_f.FACKEN = 0;
    return I2C_RET_OK;
}

void I2C_slave_init(void)
{
		stc_port_init_t stcPortInit;
		/*initiallize LED port*/
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;


    /* Initialize I2C port*/
    PORT_SetFunc(I2C1_SCL_PORT, I2C1_SCL_PIN, Func_I2c1_Scl, Disable);
    PORT_SetFunc(I2C1_SDA_PORT, I2C1_SDA_PIN, Func_I2c1_Sda, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_I2C1, Enable);
    /* Initialize I2C peripheral and enable function*/
    bsp_i2c_slave_init(I2C_SLAVE_CH);
    bsp_dma_init(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_RX_CH, SLAVE_DMA_Mode_RXsrc, SLAVE_DMA_Mode_RXdes, SLAVE_DataWidth);
    bsp_dma_init(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_TX_CH, SLAVE_DMA_Mode_TXsrc, SLAVE_DMA_Mode_TXdes, SLAVE_DataWidth);
    
    bsp_dma_SetDesAddr(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_RX_CH, (uint32_t)SlaveRxBuf);
    bsp_dma_SetSrcAddr(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_RX_CH, (uint32_t)&(I2C_SLAVE_CH->DRR));
    bsp_dma_set_count(SLAVE_I2C_DMA_UNIT,SLAVE_DMA_RX_CH,0);
    bsp_dma_ch_enable(SLAVE_I2C_DMA_UNIT,SLAVE_DMA_RX_CH,Enable);
    bsp_dma_set_TrigSrc(SLAVE_I2C_DMA_UNIT,SLAVE_DMA_RX_CH,SLAVE_DMA_RX_CH_TRIG);
    SLAVE_I2C_DMA_UNIT->CH0CTL_f.DRPTEN = 1;
    SLAVE_I2C_DMA_UNIT->RPT0_f.DRPT = SLAVE_DATA_LEN;
    
    bsp_dma_SetDesAddr(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_TX_CH, (uint32_t)&(I2C_SLAVE_CH->DTR));
    bsp_dma_SetSrcAddr(SLAVE_I2C_DMA_UNIT, SLAVE_DMA_TX_CH, (uint32_t)&SlaveTxBuf[0][0]);
    bsp_dma_set_count(SLAVE_I2C_DMA_UNIT,SLAVE_DMA_TX_CH,0);//不停发送
    bsp_dma_ch_enable(SLAVE_I2C_DMA_UNIT,SLAVE_DMA_TX_CH,Enable);
    bsp_dma_set_TrigSrc(SLAVE_I2C_DMA_UNIT,SLAVE_DMA_TX_CH,SLAVE_DMA_TX_CH_TRIG);
    SLAVE_I2C_DMA_UNIT->CH1CTL_f.SRTPEN = 1;
    SLAVE_I2C_DMA_UNIT->RPT1_f.SRPT = SLAVE_DATA_LEN;
    TimerA1_Init();
    TimerA2_Init();
}
void slave_prepareTXdata(uint8_t addr, uint8_t *data ,uint8_t len)
{
    if(len>SLAVE_DATA_LEN)
    {
        return;
    }
    if(addr>=133)
    {
        return;
    }
    for(uint8_t i = 0;i<len;i++)
    {
        SlaveTxBuf[addr][i] = data[i];
    }
}
void set_slave_Data_addr(uint8_t addr)
{
    slave_regaddr = addr;
}
void get_slave_rx_data(uint8_t *data,uint8_t pos)
{
    *data = SlaveRxBuf[pos];
}
void slave_soft_reset(void)
{
    I2C_SLAVE_CH->CR1_f.SWRST = 1;
    I2C_SLAVE_CH->CR1_f.SWRST = 0;
}
void TimerA1_Init(void)
{
    stc_timera_base_init_t stcTimeraInit;
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT_CLOCK(1), Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);//(PWC_FCG0_PERIPH_PTDIS | PWC_FCG0_PERIPH_DCU1, Enable);
    
    stcTimeraInit.enClkDiv = TimeraPclkDiv512;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 0xFFFF;        //周期数
    TIMERA_BaseInit(M4_TMRA1, &stcTimeraInit);
    /* Enable period count interrupt */
    TIMERA_IrqCmd(M4_TMRA1, TimeraIrqOverflow, Enable);

    /* Configure timera unit 1 Event triger counting up */
    M4_TMRA1->HCUPR_f.HCUP10 = Enable;
    TIMERA_SetCountTriggerSrc(EVT_I2C1_RXI);//Triger source I2C1 RX
    
    /* Set external Int Ch.4 trigger timera startup */
    TIMERA_Cmd(M4_TMRA1,Enable);//启动TIMERA            
}
static void TimerA2_OVF_Callback(void)
{
    printf("i2c timer out\r\n");
    I2C_SLAVE_CH->CR1_f.SWRST = 1;
    I2C_SLAVE_CH->CR1_f.SWRST = 0;
}
void TimerA2_Init(void)
{
    stc_timera_base_init_t stcTimeraInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
//    stc_dcu_init_t stcDcuInit;
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
//    MEM_ZERO_STRUCT(stcDcuInit);
    
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT_CLOCK(2), Enable);
//    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);//(PWC_FCG0_PERIPH_PTDIS | PWC_FCG0_PERIPH_DCU1, Enable);
    
    stcTimeraInit.enClkDiv = TimeraPclkDiv256;//84000000/256 约为328K的时钟，接近400K
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 1000;        //1000个时钟后超时中断
    TIMERA_BaseInit(M4_TMRA2, &stcTimeraInit);
    /* Enable period count interrupt */
    TIMERA_IrqCmd(M4_TMRA2, TimeraIrqOverflow, Enable);

    stcIrqRegiConf.enIRQn = Int003_IRQn;
    /* Select I2C receive full interrupt function */
    stcIrqRegiConf.enIntSrc = INT_TMRA2_OVF;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &TimerA2_OVF_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    /* Set external Int Ch.4 trigger timera startup */
//    TIMERA_Cmd(M4_TMRA2,Enable);//启动TIMERA            
}