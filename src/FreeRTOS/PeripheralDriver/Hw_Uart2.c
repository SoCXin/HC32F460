#include "hc32_ddl.h"
#include "Hw_Uart2.h"
uint8_t UART2_RXbuff[256];
static void USART2_RX_Callback(void)
{
    uint8_t data;
    data = USART2_UNIT->DR_f.RDR;
    while(USART2_UNIT->SR_f.TC == 0);
    USART_SendData(USART2_UNIT, data);
}
static void USART2_RX_ERROR_Callback(void)
{
    if(USART2_UNIT->SR_f.ORE == 1)
    {
        USART2_UNIT->CR1_f.CORE = 1;       
    }  
    if(USART2_UNIT->SR_f.FE == 1)//帧错误
    {
        USART2_UNIT->CR1_f.CFE = 1;
    }
    if(USART2_UNIT->SR_f.PE == 1)//校验错误
    {
        USART2_UNIT->CR1_f.CPE = 1;
    }
}
void Hw_Uart2_Init(void)
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

    PWC_Fcg1PeriphClockCmd(USART2_CLK,Enable);
    Port_CFG.enPinMode = Pin_Mode_Out;
	PORT_Init(USART2_TX_PORT, USART2_TX_PIN, &Port_CFG);
    Port_CFG.enPinMode = Pin_Mode_In;
    PORT_Init(USART2_TX_PORT, USART2_TX_PIN, &Port_CFG);
    
    PORT_SetFunc(USART2_RX_PORT, USART2_RX_PIN, USART2_RX_FUNC, Disable);
    PORT_SetFunc(USART2_TX_PORT, USART2_TX_PIN, USART2_TX_FUNC, Disable);
    
    USART_UART_Init(USART2_UNIT,&stcUsartConf); //初始化串口 
		
    USART_SetBaudrate(USART2_UNIT, USART2_BAUDRATE);
		
    USART_FuncCmd(USART2_UNIT, UsartTx, Enable);
	USART_FuncCmd(USART2_UNIT, UsartRx, Enable);
    USART_FuncCmd(USART2_UNIT, UsartRxInt, Enable);
    
    stcIrqRegiConf.enIntSrc = USART2_RI_NUM;
    stcIrqRegiConf.enIRQn = USART2_RX_IRQn;
    stcIrqRegiConf.pfnCallback = USART2_RX_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_02);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = USART2_EI_NUM;
    stcIrqRegiConf.enIRQn = USART2_ER_IRQn;
    stcIrqRegiConf.pfnCallback = USART2_RX_ERROR_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
}
void Test_UART2_TX(void)
{
    while(USART2_UNIT->SR_f.TC == 0);
    USART_SendData(USART2_UNIT, 0x55);
}
