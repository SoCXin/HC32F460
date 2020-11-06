#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

#define DCU_UNIT                        M4_DCU1

bool flag_DCU1_INT  = 0;
static void DCU1_INT_Callback(void)
{
    flag_DCU1_INT = 1;
    
}
void User_DCU_Init(void)
{
    stc_dcu_init_t stcDcuInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    
    MEM_ZERO_STRUCT(stcDcuInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    stcDcuInit.u32IntSel = 1;
    stcDcuInit.enIntWinMode = DcuWinIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit16;
    stcDcuInit.enOperation = DcuHwTrigOpAdd;
    
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS | PWC_FCG0_PERIPH_DCU1, Enable);
    
    DCU_Init(DCU_UNIT, &stcDcuInit);
    DCU_UNIT->CTL_f.INTEN = 1;
//    DCU_WriteDataHalfWord(DCU_UNIT, DcuRegisterData0, 0x0000);
    DCU_SetTriggerSrc(DCU_UNIT, EVT_DMA1_BTC0);
    
    stcIrqRegiConf.enIntSrc = INT_DCU1;
    stcIrqRegiConf.enIRQn = DCU1_IRQn;
    stcIrqRegiConf.pfnCallback =  DCU1_INT_Callback;   
    
    enIrqRegistration(&stcIrqRegiConf);
    
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

