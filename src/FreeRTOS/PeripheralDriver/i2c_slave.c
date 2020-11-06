#include "hc32_ddl.h"
#include "i2c_slave.h"

uint8_t SlaveRxBuf[SLAVE_DATA_LEN];
uint8_t SlaveTxBuf[133][SLAVE_DATA_LEN];
uint8_t rec_len;
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
    if(u32DataInOffset == 0)
    {
        if(SlaveRxBuf[0] <=64)
            rec_len = SlaveRxBuf[0];
    }
    if(u32DataInOffset == 1)
    {
        if(SlaveRxBuf[1] <133)
            slave_regaddr = SlaveRxBuf[1];
    }
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
            I2C_SendData(I2C_SLAVE_CH, BufRead());
        }
        else
        {
            /* Config rx buffer full interrupt function*/
            I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_RFULLIE, Enable);
        }

        /* Enable stop and NACK interrupt */
        I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_STOPIE | I2C_CR2_NACKIE, Enable);
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
                   I2C_CR2_STOPIE | I2C_CR2_NACKIE,                         \
                   Disable);
        /* Clear STOPF flag */
        I2C_ClearStatus(I2C_SLAVE_CH, I2C_CLR_STOPFCLR);
        u8FinishFlag = 1u;
        u32DataInOffset = 0;
        u32DataOutOffset = 0;
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
        if(u32DataOutOffset < rec_len)
        {
            I2C_SendData(I2C_SLAVE_CH, BufRead());
        }
        else
        {
            I2C_SendData(I2C_SLAVE_CH, 0xFF);
        }
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
uint8_t Slave_Initialize(void)
{
    stc_i2c_init_t stcI2cInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_clk_freq_t stcClkFreq;

    I2C_DeInit(I2C_SLAVE_CH);

    /* Get system clock frequency */
    CLK_GetClockFreq(&stcClkFreq);

    MEM_ZERO_STRUCT(stcI2cInit);
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
    stcIrqRegiConf.enIRQn = Int003_IRQn;
    /* Select I2C receive full interrupt function */
    stcIrqRegiConf.enIntSrc = INT_I2C1_RXI;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &I2C_RXI_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Register TEI Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    /* Select I2C TX buffer empty interrupt function */
    stcIrqRegiConf.enIntSrc = INT_I2C1_TEI;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &I2C_TEI_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Register TEI Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int004_IRQn;
    /* Select I2C TX buffer empty interrupt function */
    stcIrqRegiConf.enIntSrc = INT_I2C1_TXI;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &I2C_TXI_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* Config slave address match interrupt function*/
    I2C_IntCmd(I2C_SLAVE_CH, I2C_CR2_SLADDR0EN, Enable);
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
    Slave_Initialize();
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