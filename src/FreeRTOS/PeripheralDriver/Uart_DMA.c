#include "hc32_ddl.h"
#include "Uart_DMA.h"
uint8_t UART_RXbuff[UART3_DMA_TRNCNT];
uint8_t UART3_TXbuff[UART3_DMA_TRNCNT];
volatile uint8_t flag_Uart_status;
void USART_RX_TimerOut_Callback(void)//串口超时处理
{
    USART_CH->CR1_f.CRTOF = 1;
    TIMER0_Cmd(M4_TMR02, Tim0_ChannelA, Disable);
    M4_TMR02->CNTAR_f.CNTA = 0;

//    USART_CH->DR_f.RDR;
    //--------------------超时，DMA地址重置-------------------------------//
//    DMA_Cmd(UART3_DMA2_UNIT,Disable);
//    DMA_Cmd(UART3_DMA2_UNIT,Enable); 
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Disable);
    DMA_ClearIrqFlag(UART3_DMA2_UNIT,UART3_DMA_CH, TrnCpltIrq);
    DMA_SetTransferCnt(UART3_DMA2_UNIT,UART3_DMA_CH,UART3_DMA_TRNCNT);
    DMA_SetDesAddress(UART3_DMA2_UNIT,UART3_DMA_CH,(uint32_t)(UART_RXbuff));
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Enable);
    flag_Uart_status |= STATUS_UART_OT;
}
void USART_RX_Callback(void)
{
    uint8_t data;
    data = USART_CH->DR_f.RDR;
}
void USART_RX_ERROR_Callback(void)
{
    if(USART_CH->SR_f.ORE == 1)
    {
        USART_CH->CR1_f.CORE = 1;       
    }  
    if(USART_CH->SR_f.FE == 1)//帧错误
    {
        USART_CH->CR1_f.CFE = 1;
    }
    if(USART_CH->SR_f.PE == 1)//校验错误
    {
        USART_CH->CR1_f.CPE = 1;
    }
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Disable);
    DMA_ClearIrqFlag(UART3_DMA2_UNIT,UART3_DMA_CH, TrnCpltIrq);
    DMA_SetTransferCnt(UART3_DMA2_UNIT,UART3_DMA_CH,UART3_DMA_TRNCNT);
    DMA_SetDesAddress(UART3_DMA2_UNIT,UART3_DMA_CH,(uint32_t)(UART_RXbuff));
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Enable);
}
void hwdmx_uartInit(void)//UART初始化
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

    PWC_Fcg1PeriphClockCmd(USART_CLK,Enable);
    Port_CFG.enPinMode = Pin_Mode_Out;
	PORT_Init(USART_TX_PORT, USART_TX_PIN, &Port_CFG);
    Port_CFG.enPinMode = Pin_Mode_In;
    PORT_Init(USART_TX_PORT, USART_TX_PIN, &Port_CFG);
    
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_FUNC, Disable);
    
//    USART_CH->CR1_f.OVER8 = 1;//双倍波特率
    USART_UART_Init(USART_CH,&stcUsartConf); //初始化串口 
		
    USART_SetBaudrate(USART_CH, USART_BAUDRATE);
		
    USART_FuncCmd(USART_CH, UsartTx, Enable);
	USART_FuncCmd(USART_CH, UsartRx, Enable);
    USART_FuncCmd(USART_CH, UsartTimeOut, Enable);
    USART_FuncCmd(USART_CH, UsartTimeOutInt, Enable);
    USART_FuncCmd(USART_CH, UsartRxInt, Enable);
    
    stcIrqRegiConf.enIntSrc = USART_RI_NUM;
    stcIrqRegiConf.enIRQn = USART_RX_IRQn;
    stcIrqRegiConf.pfnCallback = USART_RX_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_02);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = USART_EI_NUM;
    stcIrqRegiConf.enIRQn = USART_ER_IRQn;
    stcIrqRegiConf.pfnCallback = USART_RX_ERROR_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = INT_USART3_RTO;
    stcIrqRegiConf.enIRQn = USART_RTO_IRQn;
    stcIrqRegiConf.pfnCallback = USART_RX_TimerOut_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    hw_txdma_init();
    hw_rxdma_init();
    UART_RTO_Timer_Init();
}
void Test_UART_TX(void)
{
    while(USART_CH->SR_f.TC == 0);
    USART_SendData(USART_CH, 0x55);
}

void UART_DMA_Callback(void)
{
//    DMA_Cmd(UART3_DMA2_UNIT,Disable);
//    DMA_Cmd(UART3_DMA2_UNIT,Enable); 
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Disable);
    DMA_ClearIrqFlag(UART3_DMA2_UNIT,UART3_DMA_CH, TrnCpltIrq);
    DMA_SetTransferCnt(UART3_DMA2_UNIT,UART3_DMA_CH,UART3_DMA_TRNCNT);
    DMA_SetDesAddress(UART3_DMA2_UNIT,UART3_DMA_CH,(uint32_t)(&UART_RXbuff[0]));
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Enable); 
    flag_Uart_status |= STATUS_UART_RX;
}
void hw_rxdma_init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;//
    stcDmaCfg.u16TransferCnt = UART3_DMA_TRNCNT;//
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&UART_RXbuff[0]);//(&DMA0_Dre_Data[0]);//Target Address
    stcDmaCfg.u32SrcAddr = ((uint32_t)(&USART_CH->DR)+2);//USART2_DR_ADDRESS;//(uint32_t)(&DMA0_Src_data[0]);//Source Address
    
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = 0;
    stcDmaCfg.u16DesRptSize = UART3_DMA_TRNCNT;

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

    PWC_Fcg0PeriphClockCmd(UART3_DMA_CLK, Enable);
    
    
    /* Enable DMA1. */
    DMA_Cmd(UART3_DMA2_UNIT,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(UART3_DMA2_UNIT, UART3_DMA_CH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(UART3_DMA2_UNIT, UART3_DMA_CH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(UART3_DMA2_UNIT, UART3_DMA_CH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA2_TC0;
    stcIrqRegiConf.enIRQn = DMA2_CH0_IRQn;
    stcIrqRegiConf.pfnCallback =  UART_DMA_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    
    DMA_SetTriggerSrc(UART3_DMA2_UNIT,UART3_DMA_CH,DMA_Trg_Src);
    
}
void RTO_Timer_Callback(void)
{
    TIMER0_Cmd(M4_TMR02, Tim0_ChannelA, Disable);
}
void UART_RTO_Timer_Init(void)
{
	stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_tim0_trigger_init_t Triger_Init;
	
	MEM_ZERO_STRUCT(stcTimerCfg);
	MEM_ZERO_STRUCT(stcIrqRegiConf);
	
	/*Write-off protection*/
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02,Enable);//时钟使能
    
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;//同步计数
	stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;//时钟源PCLK1
	stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv2;//2分频/////---------------------------UART4 串口超时时间由Timer0 CHB设置时间决定。
	stcTimerCfg.Tim0_CmpValue = 6000-1;
    
    TIMER0_BaseInit(M4_TMR02, Tim0_ChannelA, &stcTimerCfg);
    Triger_Init.Tim0_InTrigClear = Enable;
    Triger_Init.Tim0_InTrigStart = Enable;
    TIMER0_HardTriggerInit(M4_TMR02, Tim0_ChannelA, &Triger_Init);//开启串口RTO触发	
	
    TIMER0_IntCmd(M4_TMR02,Tim0_ChannelA,Enable);//TIMER0_IntCmd(M4_TMR01,Tim0_ChannelA,Enable);
	
	stcIrqRegiConf.enIntSrc = INT_TMR02_GCMA;//INT_TMR01_GCMA
	stcIrqRegiConf.pfnCallback = RTO_Timer_Callback;
	stcIrqRegiConf.enIRQn = TIMER02_CHA_IRQn;
	enIrqRegistration(&stcIrqRegiConf);
	
//	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//使能中断
	
//    TIMER0_Cmd(M4_TMR02, Tim0_ChannelA, Enable);
}

void Uart_DMA_DeInit(void)//UART反初始化
{
    stc_dma_config_t stcDmaCfg;
    stc_tim0_base_init_t stcTimerCfg;
    stc_usart_uart_init_t stcUsartConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcUsartConf);
    USART_UART_Init(USART_CH,&stcUsartConf);
    TIMER0_Cmd(M4_TMR02, Tim0_ChannelA, Disable);
    TIMER0_BaseInit(M4_TMR02, Tim0_ChannelA, &stcTimerCfg);
    DMA_InitChannel(UART3_DMA2_UNIT, UART3_DMA_CH, &stcDmaCfg);
    enIrqResign(USART_RX_IRQn);
    enIrqResign(USART_ER_IRQn);
    enIrqResign(USART_RTO_IRQn);
    enIrqResign(TIMER02_CHA_IRQn);
    enIrqResign(DMA2_CH0_IRQn);   
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02,Disable);//时钟使能
    PWC_Fcg0PeriphClockCmd(UART3_DMA_CLK, Disable);
    PWC_Fcg1PeriphClockCmd(USART_CLK,Disable);
    
}
void DMA_CH0_TC_Callback(void)
{
    ;
}
void hw_txdma_init(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcDmaCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
    stcDmaCfg.u16BlockSize = 1;//
    stcDmaCfg.u16TransferCnt = 4;//传4个字节
    
    stcDmaCfg.u32DesAddr = (uint32_t)(&USART_CH->DR);//(&DMA0_Dre_Data[0]);//Target Address
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

    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable);
    
    
    /* Enable DMA1. */
    DMA_Cmd(UART3_DMA2_UNIT,Enable);   
    /* Initialize DMA. */
    DMA_InitChannel(UART3_DMA2_UNIT, DMA_TXCH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(UART3_DMA2_UNIT, DMA_TXCH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(UART3_DMA2_UNIT, DMA_TXCH,TrnCpltIrq);
    
    stcIrqRegiConf.enIntSrc = INT_DMA2_TC1;
    stcIrqRegiConf.enIRQn = DMA2_CH1_IRQn;
    stcIrqRegiConf.pfnCallback =  DMA_CH0_TC_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt

    
    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);//打开AOS时钟
    
    DMA_SetTriggerSrc(UART3_DMA2_UNIT,DMA_TXCH,DMA_TXCH0_TRGSRC);
}
void UART_DMA_TX_Write_Buffer(uint8_t *data, uint8_t len)
{
    DMA_SetSrcAddress(UART3_DMA2_UNIT,DMA_TXCH,(uint32_t)data);
    DMA_SetTransferCnt(UART3_DMA2_UNIT,DMA_TXCH,len);
    DMA_ClearIrqFlag(UART3_DMA2_UNIT,DMA_TXCH, TrnCpltIrq);
    DMA_ChannelCmd(UART3_DMA2_UNIT, DMA_TXCH,Enable);
    USART_CH->CR1_f.TE = 0;
    USART_CH->CR1_f.TE = 1;//触发发送数据
}


