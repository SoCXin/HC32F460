#include "bsp_mco.h"


void mco_init(void)
{
    stc_clk_output_cfg_t mco_cfg;
    MEM_ZERO_STRUCT(mco_cfg);
    PORT_SetFunc(PortA, Pin08, Func_Mclkout, Disable);//MCO1 PA8
    mco_cfg.enOutputSrc = ClkOutputSrcSysclk;
    mco_cfg.enOutputDiv = ClkOutputDiv4;
    CLK_OutputClkConfig(ClkOutputCh1,&mco_cfg);
    CLK_OutputClkCmd(ClkOutputCh1,Enable);    
}



