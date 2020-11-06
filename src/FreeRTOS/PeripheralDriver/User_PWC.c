#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"


stc_pwc_pvd_cfg_t stcPwcPvdCfg;


/**
 *******************************************************************************
 ** \brief  PVD1 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Lvd1_IrqHandler(void)
{
    
}
/**
 *******************************************************************************
 ** \brief  PVD2 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Pvd2_IrqHandler(void)
{
    
}
/**
 *******************************************************************************
 ** \brief  PVD configuration function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/

void NMI_CONFIG(void)
{
    stc_nmi_config_t    stcNmiCfd;
    MEM_ZERO_STRUCT(stcNmiCfd);  
     /* Config NMI.*/
    /* Set PVD2 as NMI source. */
    stcNmiCfd.u16NmiSrc = NmiSrcVdu2;
    /* Disbale filter. */
    stcNmiCfd.enFilterEn = Disable;
    /* Set Pvd2 interrupt callback. */
    stcNmiCfd.pfnNmiCallback = Pvd2_IrqHandler;
    
    NMI_Init(&stcNmiCfd);
}

void PVD_CONFIG(void)
{
    MEM_ZERO_STRUCT(stcPwcPvdCfg);
    /* Config PVD1. */    
    /* Disable filter. */
    stcPwcPvdCfg.enPvd1FilterEn = Disable;
    /* Msk interrupt. */
    stcPwcPvdCfg.enPvd1Int = MskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdMode = PvdInt;
    /* Enable Pvd1 interrupt. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdCmpOutEn = Enable;
    /* PVD1 Threshold Voltage 2.8V. */
    stcPwcPvdCfg.enPvd1Level = Pvd1Level5; 
    
    /* Config PVD2.*/    
    /* Disable filter. */
    stcPwcPvdCfg.enPvd2FilterEn = Disable;
    /* Non-Msk interrupt. */
    stcPwcPvdCfg.enPvd2Int = NonMskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdMode = PvdInt;
    /* Enable Pvd2 interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdCmpOutEn = Enable;
    /* PVD2 Threshold Voltage 2.3V. */
    stcPwcPvdCfg.enPvd2Level = Pvd2Level1;
    
    PWC_PvdCfg(&stcPwcPvdCfg);
    
    NMI_CONFIG();
    
    /* Set PVD1 interrupt. */
    enShareIrqEnable(INT_PVD_PVD1);
    
    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(PVD_IRQn);
    NVIC_SetPriority(PVD_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(PVD_IRQn);
    
    /* Enable PVD1. */
    PWC_Pvd1Cmd(Enable);
    /* Enable PVD2. */
    PWC_Pvd2Cmd(Enable);
}
void system_sleep(void)
{
    stc_pwc_pwr_mode_cfg_t  stcPwcPwrMdCfg;
//    stc_pwc_wkup_edge_cfg_t stcPwcWkupEdgCfg;

    MEM_ZERO_STRUCT(stcPwcPwrMdCfg);
//    MEM_ZERO_STRUCT(stcPwcWkupEdgCfg);
    
    /* Config power down mode. */
    stcPwcPwrMdCfg.enPwrDownMd = PowerDownMd1;
    stcPwcPwrMdCfg.enRLdo = Enable;
    stcPwcPwrMdCfg.enIoRetain = IoPwrDownRetain;
    stcPwcPwrMdCfg.enRetSram = Disable;
    stcPwcPwrMdCfg.enVHrc = Disable;
    stcPwcPwrMdCfg.enVPll = Disable;
    stcPwcPwrMdCfg.enRunDrvs =  RunUlowspeed;
    stcPwcPwrMdCfg.enDrvAbility = Ulowspeed;
    stcPwcPwrMdCfg.enPwrDWkupTm = Vcap0047;

    PWC_PowerModeCfg(&stcPwcPwrMdCfg);
    
    PWC_EnterPowerDownMd();

//    M4_SYSREG->PWR_PWRC0_f.PWDN = Enable;//使能Powerdown模式
//    M4_SYSREG->PWR_PWRC0_f.IORTN = 0x00;
//    M4_SYSREG->PWR_PWRC0_f.VVDRSD = 0;//使用RLDO,掉电保持RAM,RTC，唤醒定时器工作
//    M4_SYSREG->PWR_PWRC0_f.PDMDS = 0x03;
//    __WFI();
    
}
