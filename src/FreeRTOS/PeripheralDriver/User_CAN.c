#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

#define PortCANTX   PortB
#define PinCANTX    Pin07
#define PortCANRX   PortB
#define PinCANRX    Pin06
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
stc_can_rxframe_t       stcRxFrame;
uint8_t                 u8RxFlag = false;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void CAN_RxIrqCallBack(void)
{
    if(true == CAN_IrqFlgGet(CanRxIrqFlg))
    {
        CAN_IrqFlgClr(CanRxIrqFlg);
//        CAN_IrqCmd(CanRxIrqEn, Disable);
        CAN_Receive(&stcRxFrame);

        u8RxFlag = true;
    }
    if(true == CAN_IrqFlgGet(CanTxBufFullIrqFlg))
    {
        printf("CanTxBufFullIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanRxOverIrqFlg))
    {
        printf("CanRxOverIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanRxBufFullIrqFlg))
    {
        printf("CanRxBufFullIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanRxBufAlmostFullIrqFlg))
    {
        printf("CanRxBufAlmostFullIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanTxPrimaryIrqFlg))
    {
        printf("CanTxPrimaryIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanTxSecondaryIrqFlg))
    {
        printf("CanTxSecondaryIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanErrorIrqFlg))
    {
        printf("CanErrorIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanAbortIrqFlg))
    {
        printf("CanAbortIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanErrorWarningIrqFlg))
    {
        printf("CanErrorWarningIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanErrorPassivenodeIrqFlg))
    {
        printf("CanErrorPassivenodeIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanErrorPassiveIrqFlg))
    {
        printf("CanErrorPassiveIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanArbiLostIrqFlg))
    {
        printf("CanArbiLostIrqFlg");
    }
    if(true == CAN_IrqFlgGet(CanBusErrorIrqFlg))
    {
        printf("CanBusErrorIrqFlg");
    }
}
void User_CAN_Init(void)
{
    stc_pwc_ram_cfg_t stcRamCfg;
    stc_can_init_config_t stcCanInitCfg;
    stc_can_filter_t stcFilter;   
    stc_irq_regi_conf_t stcIrqRegiConf;
    
    MEM_ZERO_STRUCT(stcRamCfg);
    MEM_ZERO_STRUCT(stcCanInitCfg);
    MEM_ZERO_STRUCT(stcFilter);
    
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
//    M4_SYSREG->PWR_FPRC = 0xa5ff;
//    M4_MSTP->FCG0PC = 0xa5a50001;
//   //%%*PRAMLPC = 0x00000000;              //打开CAN_BUF
//    M4_SYSREG->PWR_RAMPC0 = 0x00000000;     //必须32位操作
//    //<<CAN功能时钟使能
     //<<Enable can peripheral clock and buffer(ram)
    stcRamCfg.enRamOpMd = HighSpeedMd;
    stcRamCfg.enCan = DynamicCtl;
    PWC_RamCfg(&stcRamCfg);
    
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);
    
    PORT_SetFunc(PortCANRX, PinCANRX, Func_Can1_Rx, Disable);
    PORT_SetFunc(PortCANTX, PinCANTX, Func_Can1_Tx, Disable);
    PORT_ResetBits(PortD, Pin15);
    PORT_OE(PortD, Pin15, Enable);
//    M4_PORT->PODRD_f.POUT15  = 0;       //STB = 0
//    M4_PORT->POERD_f.POUTE15 = 1;
    //<<Can bit time config
    stcCanInitCfg.stcCanBt.PRESC = 1-1;//预分频
    stcCanInitCfg.stcCanBt.SEG_1 = 5-2;
    stcCanInitCfg.stcCanBt.SEG_2 = 3-1;
    stcCanInitCfg.stcCanBt.SJW   = 3-1;

    stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 10;//接收将满警告设置
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 16-1;//错误警告数设置

    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;//正常模式
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;//如果数据buffer已满，新接收的数据不存储
    stcCanInitCfg.enCanSAck      = CanSelfAckEnable;//自应答模式使能
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;//STB发送FIFO模式

    CAN_Init(&stcCanInitCfg);
     //<<Can filter config
    stcFilter.enAcfFormat = CanAllFrames;//
    stcFilter.enFilterSel = CanFilterSel1;//
    stcFilter.u32CODE     = 0x00000000;
    stcFilter.u32MASK     = 0x1FFFFFFF;
    CAN_FilterConfig(&stcFilter, Enable);
    
    CAN_IrqCmd(CanRxIrqEn, Enable);
    CAN_IrqCmd(CanErrorIrqEn, Enable);
    stcIrqRegiConf.enIntSrc = INT_CAN_INT;
	stcIrqRegiConf.pfnCallback = CAN_RxIrqCallBack;
	stcIrqRegiConf.enIRQn = CAN_INT_IRQn;
	enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

void User_CAN_Test(void)
{
    uint8_t pos;
    stc_can_txframe_t       stcTxFrame;
    MEM_ZERO_STRUCT(stcTxFrame);
    stcTxFrame.StdID = 0x123;
    stcTxFrame.Control_f.DLC = 0x8;
    stcTxFrame.Control_f.IDE = 0;
    for(pos = 0; pos<stcTxFrame.Control_f.DLC; pos++)
    {
        stcTxFrame.Data[pos] = pos;
    }
    while(M4_CAN->CFG_STAT_f.TACTIVE){;}
    CAN_SetFrame(&stcTxFrame);
    CAN_TransmitCmd(CanPTBTxCmd);
}

