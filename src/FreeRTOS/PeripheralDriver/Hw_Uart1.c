#include "Hw_Uart1.h"
uint8_t UART1_RXbuff[256];
uint8_t UART1_TXbuff[256];
static void USART1_RX_Callback(void)
{
    uint8_t data;
    data = USART1_UNIT->DR_f.RDR;
    while(USART1_UNIT->SR_f.TC == 0){;}
    USART_SendData(USART1_UNIT, data);
}
static void USART1_RX_ERROR_Callback(void)
{
    if(USART1_UNIT->SR_f.ORE == 1)
    {
        USART1_UNIT->CR1_f.CORE = 1;       
    }  
    if(USART1_UNIT->SR_f.FE == 1)//帧错误
    {
        USART1_UNIT->CR1_f.CFE = 1;
    }
    if(USART1_UNIT->SR_f.PE == 1)//校验错误
    {
        USART1_UNIT->CR1_f.CPE = 1;
    }
}
void Hw_Uart1_Init(void)
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

    PWC_Fcg1PeriphClockCmd(USART1_CLK,Enable);
    Port_CFG.enPinMode = Pin_Mode_Out;
	PORT_Init(USART1_TX_PORT, USART1_TX_PIN, &Port_CFG);
    Port_CFG.enPinMode = Pin_Mode_In;
    PORT_Init(USART1_TX_PORT, USART1_TX_PIN, &Port_CFG);
    
    PORT_SetFunc(USART1_RX_PORT, USART1_RX_PIN, USART1_RX_FUNC, Disable);
    PORT_SetFunc(USART1_TX_PORT, USART1_TX_PIN, USART1_TX_FUNC, Disable);
    
    USART_UART_Init(USART1_UNIT,&stcUsartConf); //初始化串口 
		
    USART_SetBaudrate(USART1_UNIT, USART1_BAUDRATE);
		
    USART_FuncCmd(USART1_UNIT, UsartTx, Enable);
	USART_FuncCmd(USART1_UNIT, UsartRx, Enable);
    USART_FuncCmd(USART1_UNIT, UsartRxInt, Enable);
    
    stcIrqRegiConf.enIntSrc = USART1_RI_NUM;
    stcIrqRegiConf.enIRQn = USART1_RX_IRQn;
    stcIrqRegiConf.pfnCallback = USART1_RX_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_02);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = USART1_EI_NUM;
    stcIrqRegiConf.enIRQn = USART1_ER_IRQn;
    stcIrqRegiConf.pfnCallback = USART1_RX_ERROR_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
}
void Test_UART1_TX(void)
{
    while(USART1_UNIT->SR_f.TC == 0);
    USART_SendData(USART1_UNIT, 0x55);
}
void DMA_CH0_TC_Callback(void)
{
    ;//
}
void UART1_TX_DMA_Init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = 1;//
    stcDmaCfg.u16TransferCnt = 4;//传4个字节
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&USART1_UNIT->DR);//(&DMA0_Dre_Data[0]);//Target Address
    stcDmaCfg.u32SrcAddr = ((uint32_t)(UART1_TXbuff));//USART2_DR_ADDRESS;//(uint32_t)(&DMA0_Src_data[0]);//Source Address
    
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
    DMA_InitChannel(M4_DMA1, DmaCh1, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(M4_DMA1, DmaCh1,Disable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(M4_DMA1, DmaCh1,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA1_TC1;
    stcIrqRegiConf.enIRQn = DMA1_CH0_IRQn;
    stcIrqRegiConf.pfnCallback =  DMA_CH0_TC_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);//打开AOS时钟
    
    DMA_SetTriggerSrc(M4_DMA1,DmaCh1,EVT_USART1_TI);
}
void UART1_DMA_TX_Write_Buffer(uint8_t *data, uint8_t len)
{
    DMA_SetSrcAddress(M4_DMA1,DmaCh1,(uint32_t)data);
    DMA_SetTransferCnt(M4_DMA1,DmaCh1,len);
    DMA_ClearIrqFlag(M4_DMA1,DmaCh1, TrnCpltIrq);
    DMA_ChannelCmd(M4_DMA1, DmaCh1,Enable);
	while(USART1_UNIT->SR_f.TC == 0){;}
    USART1_UNIT->CR1_f.TE = 0;
    USART1_UNIT->CR1_f.TE = 1;
}
