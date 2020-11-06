#include "hc32_ddl.h"
#include "User_Uart.h"
#include "System_InterruptCFG_Def.h"
//#include "cmsis_os.h"
volatile uint8_t Rx2data[4000];
volatile bool flag_RX2_receviced;
volatile uint8_t Uart2data[4000];
volatile uint32_t rec_len=0, datalen;
bool state;
void USART2_RX_TimerOut_Callback(void)
{
    M4_USART2->CR1_f.CRTOF = 1;
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Disable);
    printf("UART TimerOut\r\n");
}
void USART2_RX_Callback(void)
{
    Rx2data[rec_len] = M4_USART2->DR_f.RDR;
    rec_len++;
    flag_RX2_receviced = true;
}
en_result_t Get_UART2_data(uint8_t *RX2_data, uint32_t *length)
{
    uint32_t pos;
    if(flag_RX2_receviced == true)
    {
        *length = rec_len;
        rec_len = 0;
        for(pos = 0;pos < *length; pos++)
        {
            RX2_data[pos] = Rx2data[pos];
        }      
        flag_RX2_receviced = false;
        return Ok;
    }
    return Error;
}
void test_Usart2(void)
{
    if(flag_RX2_receviced == true)
    {
       state = Get_UART2_data((uint8_t *)Uart2data,(uint32_t *)&datalen);
//       printf("datalenth: %d\r\n",datalen);
       for(int i= 0;i<datalen;i++)
        {
//            printf("0x%x ",Uart2data[i]);
            USART_SendData(M4_USART2, Uart2data[i]);
            while(M4_USART2->SR_f.TC == 0);
        } 
    }
}
void USART2_RX_ERROR_Callback(void)
{
    //printf("Usart2_Error\r\n");
    if(M4_USART2->SR_f.ORE == 1)
    {
        printf("Usart2_CORE\r\n");
        M4_USART2->CR1_f.CORE = 1;       
    }  
    if(M4_USART2->SR_f.FE == 1)//帧错误
    {
        M4_USART2->CR1_f.CFE = 1;
        printf("Usart2_FE\r\n");
    }
    if(M4_USART2->SR_f.PE == 1)//校验错误
    {
        M4_USART2->CR1_f.CPE = 1;
    }
}
void User_USART2_Init(void)
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

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART2,Enable);
    Port_CFG.enPinMode = Pin_Mode_Out;
	  PORT_Init(PortA, Pin02, &Port_CFG);
    
    PORT_SetFunc(PortA, Pin02, Func_Usart2_Tx, Disable);
    PORT_SetFunc(PortA, Pin03, Func_Usart2_Rx, Disable);
    
    M4_USART2->CR1_f.OVER8 = 1;//双倍波特率
    USART_UART_Init(M4_USART2,&stcUsartConf); //初始化串口 
		
    USART_SetBaudrate(M4_USART2, 115200);
		
    USART_FuncCmd(M4_USART2, UsartTx, Enable);
	USART_FuncCmd(M4_USART2, UsartRx, Enable);
    USART_FuncCmd(M4_USART2, UsartTimeOut, Enable);
    USART_FuncCmd(M4_USART2, UsartTimeOutInt, Enable);
    USART_FuncCmd(M4_USART2, UsartRxInt, Enable);
    
    stcIrqRegiConf.enIntSrc = INT_USART2_RI;
    stcIrqRegiConf.enIRQn = USART2_RX_IRQn;
    stcIrqRegiConf.pfnCallback = USART2_RX_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = INT_USART2_EI;
    stcIrqRegiConf.enIRQn = USART2_ERROR_IRQn;
    stcIrqRegiConf.pfnCallback = USART2_RX_ERROR_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = INT_USART2_RTO;
    stcIrqRegiConf.enIRQn = USART2_RTO_IRQn;
    stcIrqRegiConf.pfnCallback = USART2_RX_TimerOut_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    rec_len = 0;
}
void USART_RX_Callback(void)
{
    uint8_t data;
    data = USART_CH->DR_f.RDR;
}
void USART_RX_ERROR_Callback(void)
{
    //printf("Usart2_Error\r\n");
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
}
void User_USART_Init(void)
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

    PWC_Fcg1PeriphClockCmd(USART_CLK,Enable);
    Port_CFG.enPinMode = Pin_Mode_Out;
	PORT_Init(USART_TX_PORT, USART_TX_PIN, &Port_CFG);
    Port_CFG.enPinMode = Pin_Mode_In;
    PORT_Init(USART_TX_PORT, USART_TX_PIN, &Port_CFG);
    
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_FUNC, Disable);
    
//    USART_CH->CR1_f.OVER8 = 1;//双倍波特率
    USART_UART_Init(USART_CH,&stcUsartConf); //初始化串口 
		
    USART_SetBaudrate(USART_CH, 115200);
		
    USART_FuncCmd(USART_CH, UsartTx, Enable);
	USART_FuncCmd(USART_CH, UsartRx, Enable);
//    USART_FuncCmd(USART_CH, UsartTimeOut, Enable);
//    USART_FuncCmd(USART_CH, UsartTimeOutInt, Enable);
    USART_FuncCmd(USART_CH, UsartRxInt, Enable);
    
    stcIrqRegiConf.enIntSrc = USART_RI_NUM;
    stcIrqRegiConf.enIRQn = USART_RX_IRQn;
    stcIrqRegiConf.pfnCallback = USART_RX_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_02);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
    stcIrqRegiConf.enIntSrc = USART_EI_NUM;
    stcIrqRegiConf.enIRQn = USART_ER_IRQn;
    stcIrqRegiConf.pfnCallback = USART_RX_ERROR_Callback;
    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//
    
//    stcIrqRegiConf.enIntSrc = INT_USART2_RTO;
//    stcIrqRegiConf.enIRQn = USART2_RTO_IRQn;
//    stcIrqRegiConf.pfnCallback = USART2_RX_TimerOut_Callback;
//    enIrqRegistration(&stcIrqRegiConf);//配置中断向量及函数
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    rec_len = 0;
}
void Test_UART_TX(void)
{
    while(USART_CH->SR_f.TC == 0);
    USART_SendData(USART_CH, 0x55);
}