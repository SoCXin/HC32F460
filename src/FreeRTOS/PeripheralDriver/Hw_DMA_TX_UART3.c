#include "Hw_DMA_TX_UART3.h"
#define MAX_len 256
uint8_t UART3_RXbuff[MAX_len];
uint8_t UART3_TXbuff[MAX_len];
static uint8_t UART3_TX_CNT,UART3_RX_CNT = 0;
uint8_t * p_txdata;
void UART3_TX_DMA_Init(void);
void Usart3RxEnd_IrqHandler(void)
{
	UART3_RXbuff[UART3_RX_CNT] = USART3_UNIT->DR_f.RDR;
	UART3_RX_CNT++;
}
static void USART1_RX_Callback(void)
{
//    UART3_RXbuff[UART3_RX_CNT] = USART3_UNIT->DR_f.RDR;
//	UART3_RX_CNT++;
}
static void USART3_RX_ERROR_Callback(void)
{
    if(USART3_UNIT->SR_f.ORE == 1)
    {
        USART3_UNIT->CR1_f.CORE = 1;       
    }  
    if(USART3_UNIT->SR_f.FE == 1)//帧错误
    {
        USART3_UNIT->CR1_f.CFE = 1;
    }
    if(USART3_UNIT->SR_f.PE == 1)//校验错误
    {
        USART3_UNIT->CR1_f.CPE = 1;
    }
}
/**
 *******************************************************************************
 ** \brief USART TX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTxIrqCallback(void)
{
	if(UART3_TX_CNT != 0)
	{
		USART_SendData(USART3_UNIT, *p_txdata);
		p_txdata++;
		UART3_TX_CNT--;
	}
	if(UART3_TX_CNT == 0)
	{
		USART_FuncCmd(USART3_UNIT, UsartTxEmptyInt, Disable);
		USART_FuncCmd(USART3_UNIT, UsartTxCmpltInt, Enable);
	}
}

/**
 *******************************************************************************
 ** \brief USART TX complete irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTxCmpltIrqCallback(void)
{
	USART_FuncCmd(USART3_UNIT, UsartTx, Disable);
	USART_FuncCmd(USART3_UNIT, UsartTxCmpltInt, Disable);
}

void Hw_Uart3_Init(void)
{
  	stc_usart_uart_init_t stcUsartConf;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t Port_CFG;
  	MEM_ZERO_STRUCT(stcUsartConf);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(Port_CFG);
        
	  stcUsartConf.enClkMode = UsartIntClkCkNoOutput;//internal Clk
	  stcUsartConf.enDataLength = UsartDataBits8;//8 bit data
	  stcUsartConf.enDirection = UsartDataLsbFirst;//小端数据
    stcUsartConf.enParity = UsartParityNone;//无奇偶校验
    stcUsartConf.enStopBit = UsartOneStopBit;//1个停止位
	  stcUsartConf.enSampleMode = UsartSamleBit8;
    stcUsartConf.enDetectMode = UsartStartBitFallEdge;//RX起始位为下降沿

    PWC_Fcg1PeriphClockCmd(USART3_CLK,Enable);
    Port_CFG.enPinMode = Pin_Mode_Out;
	PORT_Init(USART3_TX_PORT, USART3_TX_PIN, &Port_CFG);
    Port_CFG.enPinMode = Pin_Mode_In;
    PORT_Init(USART3_TX_PORT, USART3_TX_PIN, &Port_CFG);
    
    PORT_SetFunc(USART3_RX_PORT, USART3_RX_PIN, USART3_RX_FUNC, Disable);
    PORT_SetFunc(USART3_TX_PORT, USART3_TX_PIN, USART3_TX_FUNC, Disable);
    
    USART_UART_Init(USART3_UNIT,&stcUsartConf); //初始化串口 
		
    USART_SetBaudrate(USART3_UNIT, USART3_BAUDRATE);
		
    USART_FuncCmd(USART3_UNIT, UsartTx, Disable);
	USART_FuncCmd(USART3_UNIT, UsartRx, Enable);
    USART_FuncCmd(USART3_UNIT, UsartRxInt, Enable);
    
//    stcIrqRegiConf.enIntSrc = USART3_RI_NUM;
//    stcIrqRegiConf.enIRQn = USART3_RX_IRQn;
//    stcIrqRegiConf.pfnCallback = USART1_RX_Callback;
//    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_02);
//	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    enShareIrqEnable(INT_USART3_RI);
	NVIC_EnableIRQ(Int137_IRQn);
	
    stcIrqRegiConf.enIntSrc = USART3_EI_NUM;
    stcIrqRegiConf.enIRQn = USART3_ER_IRQn;
    stcIrqRegiConf.pfnCallback = USART3_RX_ERROR_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
	
	/* Set USART TX IRQ */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    stcIrqRegiConf.pfnCallback = &UsartTxIrqCallback;
    stcIrqRegiConf.enIntSrc = USART3_TI_NUM;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Set USART TX complete IRQ */
    stcIrqRegiConf.enIRQn = Int003_IRQn;
    stcIrqRegiConf.pfnCallback = &UsartTxCmpltIrqCallback;
    stcIrqRegiConf.enIntSrc = USART3_TCI_NUM;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    UART3_TX_DMA_Init();
}
void Test_UART3_TX(void)
{
    while(USART3_UNIT->SR_f.TC == 0);
    USART_SendData(USART3_UNIT, 0x55);
}
void DMA_CH0_TC_Callback(void)
{
    ;
}
void UART3_TX_DMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = 1;//
    stcDmaCfg.u16TransferCnt = 4;//传4个字节
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&USART3_UNIT->DR);//(&DMA0_Dre_Data[0]);//Target Address
    stcDmaCfg.u32SrcAddr = ((uint32_t)(UART3_TXbuff));//USART2_DR_ADDRESS;//(uint32_t)(&DMA0_Src_data[0]);//Source Address
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = 4;
    stcDmaCfg.u16DesRptSize = 0;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;     
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;   
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;//源地址自增
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;//目标地址不变
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1, Enable);
    
    
    /* Enable DMA1. */
    DMA_Cmd(M4_DMA1,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(M4_DMA1, DmaCh0, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(M4_DMA1, DmaCh0,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(M4_DMA1, DmaCh0,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA1_TC0;
    stcIrqRegiConf.enIRQn = DMA1_CH0_IRQn;
    stcIrqRegiConf.pfnCallback =  DMA_CH0_TC_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);//打开AOS时钟
    
    DMA_SetTriggerSrc(M4_DMA1,DmaCh0,DMA1_CH0_TRGSRC);
}
void UART3_DMA_TX_Write_Buffer(uint8_t *data, uint8_t len)
{
    DMA_SetSrcAddress(M4_DMA1,DmaCh0,(uint32_t)data);
    DMA_SetTransferCnt(M4_DMA1,DmaCh0,len);
    DMA_ClearIrqFlag(M4_DMA1,DmaCh0, TrnCpltIrq);
    DMA_ChannelCmd(M4_DMA1, DmaCh0,Enable);
    USART3_UNIT->CR1_f.TE = 0;
    USART3_UNIT->CR1_f.TE = 1;//触发发送数据
}
void UART_INT_Write_Buffer(uint8_t *data, uint8_t len)
{
	UART3_TX_CNT = len;
	p_txdata = data;
	USART_FuncCmd(USART3_UNIT, UsartTxAndTxEmptyInt, Enable);
}