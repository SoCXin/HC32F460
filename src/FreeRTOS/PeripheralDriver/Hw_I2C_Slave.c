#include "Hw_I2C_Slave.h"
#include "SEGGER_RTT.h"
M4_I2C_TypeDef* pstcI2Cx = I2C1_UNIT;
uint32_t u32DataInOffset = 0ul;
uint32_t u32DataOutOffset = 0ul;
uint8_t u8TxBuf[SLAVE_DATA_LEN];
uint8_t u8RxBuf[SLAVE_DATA_LEN];
/**
 *******************************************************************************
 ** \brief static function for buffer write.
 **
 ** \param  u8Data the data to be write.
 **
 ** \retval None
 **
 ******************************************************************************/
static void BufWrite(uint8_t u8Data)
{
    if(u32DataInOffset >= SLAVE_DATA_LEN)
    {
       u32DataInOffset = 0;
    }
    u8RxBuf[u32DataInOffset] = u8Data;
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
static uint8_t BufRead(void)
{
    uint8_t temp;
    if(u32DataOutOffset >= SLAVE_DATA_LEN)
    {
        u32DataOutOffset = 0;
    }
    temp = u8TxBuf[u32DataOutOffset];
    u32DataOutOffset++;

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
    printf("%x\r\n" pstcI2Cx->)j;ajdj
    if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_SLADDR0F))
    {
        I2C_ClearStatus(pstcI2Cx, I2C_CLR_SLADDR0FCLR|I2C_CLR_STOPFCLR|I2C_CLR_NACKFCLR);
        /* Enable Tx or Rx function*/
        if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_TRA))
        {
            /* Config tx buffer empty interrupt function*/
            I2C_IntCmd(pstcI2Cx, I2C_CR2_TEMPTYIE | I2C_CR2_TENDIE, Enable);
			u32DataOutOffset = 0;//将数据指向首地址。
            /* Write the first data to DTR immediately */
            I2C_SendData(pstcI2Cx, BufRead());
        }
        else
        {
            /* Config rx buffer full interrupt function*/
            I2C_IntCmd(pstcI2Cx, I2C_CR2_RFULLIE, Enable);
        }

        /* Enable stop and NACK interrupt */
        I2C_IntCmd(pstcI2Cx, I2C_CR2_STOPIE | I2C_CR2_NACKIE, Enable);
    }
    /* If NACK interrupt occurred */
    else if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))
    {
        /* clear NACK flag*/
        I2C_ClearStatus(pstcI2Cx, I2C_CLR_NACKFCLR);
        /* Stop tx or rx process*/
        if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_TRA))
        {
            /* Config tx buffer empty interrupt function disable*/
            I2C_IntCmd(pstcI2Cx, I2C_CR2_TEMPTYIE | I2C_CR2_TENDIE, Disable);

            /* Read DRR register to release SCL*/
            I2C_ReadData(pstcI2Cx);

        }
        else
        {
            /* Config rx buffer full interrupt function disable */
            I2C_IntCmd(pstcI2Cx, I2C_CR2_RFULLIE, Disable);
        }
    }

    /* If stop interrupt occurred */
    if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_STOPF))
    {
        /* Disable all interrupt enable flag except SLADDR0IE*/
        I2C_IntCmd(pstcI2Cx,                                                  \
                      I2C_CR2_TEMPTYIE | I2C_CR2_TENDIE |  I2C_CR2_RFULLIE |   \
                      I2C_CR2_STOPIE | I2C_CR2_NACKIE,                         \
                      Disable);
        /* Clear STOPF flag */
        I2C_ClearStatus(pstcI2Cx, I2C_CLR_STOPFCLR | I2C_CLR_NACKFCLR);
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
    if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))
    {
        /* Dummy read for release SCL */
        I2C_ReadData(pstcI2Cx);
    }
}
/**
 * @brief   I2C TXI(transfer buffer empty) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_TXI_Callback(void)
{
    if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_TEMPTYF))
    {
        if(u32DataOutOffset < SLAVE_DATA_LEN)
        {
            I2C_SendData(pstcI2Cx, BufRead());//写数据到I2C数据寄存器
        }
        else
        {
//            u8FinishFlag = 1U;
            /* Disable TXI interrupt */
            I2C_IntCmd(pstcI2Cx, I2C_CR2_TEMPTYIE, Disable);
            /* Enable TEI interrupt */
            I2C_IntCmd(pstcI2Cx, I2C_CR2_TENDIE, Enable);
        }
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
    if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_RFULLF))
    {
        BufWrite(I2C_ReadData(pstcI2Cx));
    }
}
void TXbuffer_Test_init(void)
{
	uint32_t i;
	for(i=0;i<SLAVE_DATA_LEN;i++)
	{
		u8TxBuf[i] = i;
	}
}
void Hw_I2C_Slave_Port_Init(void)
{
	/* Initialize I2C port*/
    PORT_SetFunc(I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_Pin, I2C_SLAVE_SCL_FUNC, Disable);
    PORT_SetFunc(I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_Pin, I2C_SLAVE_SDA_FUNC, Disable);
}
uint8_t Hw_I2C_Slave_Init(M4_I2C_TypeDef* I2Cx)
{
	stc_i2c_init_t stcI2cInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_clk_freq_t stcClkFreq;
	PWC_Fcg1PeriphClockCmd(I2C1_CLK, Enable);
	pstcI2Cx = I2Cx;
	TXbuffer_Test_init();
	Hw_I2C_Slave_Port_Init();
    I2C_DeInit(pstcI2Cx);

    /* Get system clock frequency */
    CLK_GetClockFreq(&stcClkFreq);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.enI2cMode = I2cSlave;
    stcI2cInit.u32Pclk3 = stcClkFreq.pclk3Freq;
    stcI2cInit.u32Baudrate = 400000;
    I2C_Init(pstcI2Cx, &stcI2cInit);

    I2C_Cmd(pstcI2Cx, Enable);

    /* Set slave address*/
#ifdef I2C_10BITS_ADDRESS
    I2C_SlaveAdr0Config(I2C_CH, Enable, Adr10bit, SLAVE_ADDRESS);
#else
    I2C_SlaveAdr0Config(pstcI2Cx, Enable, Adr7bit, SLAVE_ADDRESS);
#endif
    /* Register EEI Int to Vect.No.001 */
    stcIrqRegiConf.enIRQn = Int004_IRQn;
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
    stcIrqRegiConf.pfnCallback = I2C_RXI_Callback;
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
    stcIrqRegiConf.pfnCallback = I2C_TEI_Callback;
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
    I2C_IntCmd(pstcI2Cx, I2C_CR2_SLADDR0EN, Enable);

	return 0;
}





