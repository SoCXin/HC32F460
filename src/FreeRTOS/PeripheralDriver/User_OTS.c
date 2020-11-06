#include "hc32_ddl.h"
float32_t f_temperature;

void OtsBaseConfig(void)
{
    stc_ots_init_t stcOtsInit;

    stcOtsInit.enAutoOff   = OtsAutoOff_Enable;
    stcOtsInit.u8ClkFreq = 168;
    stcOtsInit.enClkSel    = OtsClkSel_Hrc;
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_OTS,Enable);    
    OTS_Init(&stcOtsInit);

    if (OtsClkSel_Hrc == stcOtsInit.enClkSel)
    {
        /* Enable HRC for OTS */
        CLK_HrcCmd(Enable);
        /* Enable XTAL32 while clock selcting HRC */
        CLK_Xtal32Cmd(Enable);
    }
    else
    {
        /* Enable XTAL for OTS */
        CLK_XtalCmd(Enable);
    }
    
    /* Enable XTAL for OTS */
    CLK_LrcCmd(Enable);
}

void User_OTS_Init(void)
{
    OtsBaseConfig();
}

void Print_CPU_Temperature(void)
{
    
    OTS_StartGetTemp(&f_temperature,500);
    printf("CPU Temperature is %f\r\n", f_temperature);
}

