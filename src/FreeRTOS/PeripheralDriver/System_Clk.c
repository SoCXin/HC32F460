#include "hc32_ddl.h"
#include "system_clk_cfg.h"
#include "System_InterruptCFG_Def.h"
/*
system clock initial, all parameter refer to system_clk_cfg.h setting.
*/
void XTAL_FAULT_Config(void);
void system_clk_init(void)
{
    stc_clk_xtal_cfg_t XTAL_CFG;
    stc_clk_xtal32_cfg_t XTAL32_CFG;
    stc_clk_mpll_cfg_t MPLL_CFG;
    stc_clk_upll_cfg_t UPLL_CFG;
    stc_clk_sysclk_cfg_t SYS_CLK_CFG;
    stc_sram_config_t SRAM_CFG;
    MEM_ZERO_STRUCT(SRAM_CFG);
    MEM_ZERO_STRUCT(XTAL_CFG);
    MEM_ZERO_STRUCT(XTAL32_CFG);
    MEM_ZERO_STRUCT(MPLL_CFG);
    MEM_ZERO_STRUCT(UPLL_CFG);
    MEM_ZERO_STRUCT(SYS_CLK_CFG);
    
    
    CLK_HrcCmd((en_functional_state_t)HRC_ENABLE);
//    CLK_MrcCmd((en_functional_state_t)MRC_ENABLE);
    CLK_LrcCmd((en_functional_state_t)LRC_ENABLE);
#if (XTAL_ENABLE == ENABLE)
    XTAL_CFG.enMode = XTAL_MODE;
    XTAL_CFG.enDrv = XTAL_DRV;
    XTAL_CFG.enFastStartup = XTAL_SUPDRV_ENABLE;
    CLK_XtalStbConfig(XTAL_STB);
    CLK_XtalConfig(&XTAL_CFG);    
#endif
    CLK_XtalCmd((en_functional_state_t)XTAL_ENABLE);
#if (XTAL32_ENABLE == ENABLE)
    CLK_Xtal32Cmd(Disable);
    XTAL32_CFG.enFilterMode = XTAL32_NF_MODE;
//    XTAL32_CFG.enFastStartup = XTAL32_SUPDRV_ENABLE;
    XTAL32_CFG.enDrv = XTAL32_DRV;
    CLK_Xtal32Config(&XTAL32_CFG);
#endif
    CLK_Xtal32Cmd((en_functional_state_t)XTAL32_ENABLE);
    
    CLK_SetPllSource(PLL_CLK_SOURCE);//MPLL&UPLL Clock source
#if (MPLL_CLK_ENABLE == ENABLE)
    MPLL_CFG.pllmDiv = MPLL_CLK_M_DIV;
    MPLL_CFG.plln = MPLL_CLK_NUM;
    MPLL_CFG.PllpDiv = MPLL_CLK_P_DIV;
    MPLL_CFG.PllqDiv = MPLL_CLK_Q_DIV;
    MPLL_CFG.PllrDiv = MPLL_CLK_R_DIV;    
    CLK_MpllConfig(&MPLL_CFG);
#endif
    CLK_MpllCmd((en_functional_state_t)MPLL_CLK_ENABLE);
#if MPLL_CLK_ENABLE
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));
#endif    
#if (UPLL_CLK_ENABLE == ENABLE)
    UPLL_CFG.pllmDiv = UPLL_CLK_M_DIV;
    UPLL_CFG.plln = UPLL_CLK_NUM;
    UPLL_CFG.PllpDiv = UPLL_CLK_P_DIV;
    UPLL_CFG.PllqDiv = UPLL_CLK_Q_DIV;
    UPLL_CFG.PllrDiv = UPLL_CLK_R_DIV;
    CLK_UpllConfig(&UPLL_CFG);
#endif
    CLK_UpllCmd((en_functional_state_t)UPLL_CLK_ENABLE);
#if UPLL_CLK_ENABLE
    while(Set != CLK_GetFlagStatus(ClkFlagUPLLRdy));
#endif    
    CLK_SetUsbClkSource(USB_CLK_SOURCE);
    
    CLK_SetPeriClkSource((en_clk_peri_source_t)PERI_CLK_SOURCE);
    SYS_CLK_CFG.enExclkDiv = EXCKS_DIV;
    SYS_CLK_CFG.enHclkDiv = HCLK_DIV;
    SYS_CLK_CFG.enPclk0Div = PCLK0S_DIV;
    SYS_CLK_CFG.enPclk1Div = PCLK1S_DIV;
    SYS_CLK_CFG.enPclk2Div = PCLK2S_DIV;
    SYS_CLK_CFG.enPclk3Div = PCLK3S_DIV;
    SYS_CLK_CFG.enPclk4Div = PCLK4S_DIV;
//-------------Flash wait setting--------------//    
/* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_4);
    EFM_InstructionCacheCmd(Enable);
    EFM_Lock();
    SRAM_WT_Enable();
    SRAM_CK_Enable();
    M4_SRAMC->WTCR_f.SRAM12_RWT = SramCycle2;
    M4_SRAMC->WTCR_f.SRAM12_WWT = SramCycle2;
    M4_SRAMC->WTCR_f.SRAM3_RWT = SramCycle2;
    M4_SRAMC->WTCR_f.SRAM3_WWT = SramCycle2;
    M4_SRAMC->WTCR_f.SRAMR_RWT = SramCycle2;
    M4_SRAMC->WTCR_f.SRAMR_WWT = SramCycle2;
    SRAM_WT_Disable();
    SRAM_CK_Disable();
//-------------Switch system clock-----------------------//
    CLK_SetSysClkSource(SYSTEMCLKSOURCE);
    CLK_SysClkConfig(&SYS_CLK_CFG); 
	XTAL_FAULT_Config();	
}
static void XTAL_FAULT_Callback(void)
{
	stc_clk_mpll_cfg_t MPLL_CFG;
	MEM_ZERO_STRUCT(MPLL_CFG);
	CLK_ClearXtalStdFlag();
	CLK_SetPllSource(ClkPllSrcHRC);//MPLL&UPLL Clock source
    MPLL_CFG.pllmDiv = MPLL_CLK_M_DIV+1;
    MPLL_CFG.plln = MPLL_CLK_NUM;
    MPLL_CFG.PllpDiv = MPLL_CLK_P_DIV;
    MPLL_CFG.PllqDiv = MPLL_CLK_Q_DIV;
    MPLL_CFG.PllrDiv = MPLL_CLK_R_DIV;    
    CLK_MpllConfig(&MPLL_CFG);
	NVIC_DisableIRQ(XTAL_FAULT_IRQn);
}
void XTAL_FAULT_Config(void)
{
	stc_clk_xtal_stp_cfg_t XtalstpCfg;
	stc_irq_regi_conf_t stcIrqRegiCfg;
	MEM_ZERO_STRUCT(XtalstpCfg);
	MEM_ZERO_STRUCT(stcIrqRegiCfg);
	XtalstpCfg.enDetect = Enable;
	XtalstpCfg.enMode = ClkXtalStpModeInt;
	XtalstpCfg.enModeInt = Enable;
	CLK_XtalStpConfig(&XtalstpCfg);
	stcIrqRegiCfg.enIntSrc = INT_XTAL_STOP;
	stcIrqRegiCfg.enIRQn = XTAL_FAULT_IRQn;
	stcIrqRegiCfg.pfnCallback = XTAL_FAULT_Callback;
	enIrqRegistration(&stcIrqRegiCfg);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
}
/*End of file*/
