#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

uint16_t ADC1_AIN10_Data;

uint16_t Get_AIN10Data(void)
{
    return ADC1_AIN10_Data;
}
void Get_ADC1_Data(uint16_t *data)
{
    *data = M4_ADC1->DR10;
}
void Get_ADC_V11(uint16_t *data)
{
	 *data = M4_ADC1->DR16;
}
void Set_ADC_Data(uint16_t data)
{
    ADC1_AIN10_Data = data;
}
void ADC_EOCA_CallBack(void)
{
    Get_ADC1_Data(&ADC1_AIN10_Data);
}

void ADC1_Start_convert(void)
{
    M4_ADC1->STR = 1;
}

void User_ADC_Init(void)
{
	uint8_t au8Adc1SaSampTime[3] = {0xFF,0xFF,0xFF};
	stc_adc_init_t stcAdcInit;
    stc_adc_ch_cfg_t  stcAdcBaseCFG;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t Port_CFG;
//    stc_adc_pga_cfg_t ADC_PGA_CFG;
    
    MEM_ZERO_STRUCT(stcAdcInit);
    MEM_ZERO_STRUCT(stcAdcBaseCFG);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(Port_CFG);
//    MEM_ZERO_STRUCT(ADC_PGA_CFG);
    
//    CLK_SetPeriClkSource(ClkPeriSrcMpllp);//MPLLP 3分频56MHz
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1|PWC_FCG3_PERIPH_CMP, Enable);
    stcAdcInit.enScanMode = AdcMode_SAOnce;
    stcAdcInit.enDataAlign = AdcDataAlign_Right;
    stcAdcInit.enResolution = AdcResolution_12Bit;
//    stcAdcInit.enAutoClear = AdcClren_Enable;
    
//    stcAdcInit.enAverageCount = AdcAvcnt_4;
    
    
    ADC_Init(M4_ADC1, &stcAdcInit);//配置ADC
//    ADC_Init(M4_ADC2, &stcAdcInit);

//ADC_PGA_CFG.enCtl = AdcPgaCtl_Amplify;//功能打开，使能PGA
//ADC_PGA_CFG.enFactor = AdcPgaFactor_2;//放大倍数2倍
//ADC_PGA_CFG.enNegativeIn = AdcPgaNegative_VSSA;//PGA负端输入接模拟地
//    ADC_PgaCmd(Enable);
//    ADC_ConfigPga(AdcPgaFactor_2,AdcPgaNegative_VSSA);//配置PGA
//    ADC_AddPgaChannel(PGA_CH1);//配置PGA通道为AN1;
//    ADC_PgaCmd(Disable);//PGA使能
    Port_CFG.enPinMode = Pin_Mode_Ana;
    PORT_Init(PortC, Pin00, &Port_CFG);//config PC00 As ADC_IN10
    
    stcAdcBaseCFG.u32Channel = ADC1_CH10|ADC1_CH16;
//    stcAdcBaseCFG.enAvgEnable = true;
    stcAdcBaseCFG.pu8SampTime = au8Adc1SaSampTime;
    stcAdcBaseCFG.u8Sequence = ADC_SEQ_A;//Must be setting, Default can not convert data
    ADC_AddAdcChannel(M4_ADC1, &stcAdcBaseCFG);

  stcIrqRegiConf.enIntSrc = INT_ADC1_EOCA;
	stcIrqRegiConf.pfnCallback = ADC_EOCA_CallBack;
	stcIrqRegiConf.enIRQn = ADC1_EOCA_IRQn;
	enIrqRegistration(&stcIrqRegiConf);
	M4_ADC1->ISR_f.EOCAF  = 0;
    M4_ADC1->ICR_f.EOCAIEN = 1;
	M4_CMP_CR->RVADC = 0x5500;
    M4_CMP_CR->RVADC_f.VREFSW = 1;
	PWC_PwrMonitorCmd(Enable);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt
	
}
