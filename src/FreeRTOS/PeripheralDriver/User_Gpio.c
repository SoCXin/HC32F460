#include "hc32_ddl.h"
#include "hc32f46x_gpio.h"
#include "User_Gpio.h"
#include "System_InterruptCFG_Def.h"
#include "ff.h"
#include "System_PowerDown.h"
bool flag_key0,flag_key1;

uint8_t LED_Status = 0;
void key0_int_callback(void)
{
//	CLK_SetSysClkSource(ClkSysSrcMRC);
	
	printf("key0!\r\n");
    flag_key0 = 1;	
	LPM_TEST();
}
void key1_int_callback(void)
{
	PWC_IrqClkRecover();
    flag_key1 = 1;
	printf("key1!\r\n");
	PWC_IrqClkBackup();
}
void User_Gpio_Init(void)
{
    stc_port_init_t Port_CFG;
    stc_keyscan_config_t key_CFG;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_exint_config_t stcEitCfg;
	MEM_ZERO_STRUCT(Port_CFG);
    MEM_ZERO_STRUCT(key_CFG);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
	Port_CFG.enPinMode = Pin_Mode_Out;

    
	PORT_Init(LED0_PORT, LED0_Pin, &Port_CFG);
    PORT_Init(LED1_PORT, LED1_Pin, &Port_CFG);
    PORT_Init(LED2_PORT, LED2_Pin, &Port_CFG);
    PORT_Init(LED3_PORT, LED3_Pin, &Port_CFG);
	PORT_Init(PortC, Pin15, &Port_CFG);
    Port_CFG.enPinMode = Pin_Mode_In;
    Port_CFG.enExInt = Enable;
    PORT_Init(Key0_PORT,Key0_Pin, &Port_CFG);
    PORT_Init(Key1_PORT,Key1_Pin, &Port_CFG);
    Port_CFG.enPullUp = Enable;
//    PORT_Init(PortE, Pin04, &Port_CFG);
    LED_Status = 0;
    
//    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_KEY, Enable);
//    key_CFG.enHizCycle = Hiz1K;
//    key_CFG.enKeyoutSel = Keyout0To3;
//    key_CFG.enKeyscanClk = KeyscanHclk;
//    key_CFG.u16KeyinSel = Keyin03;
//    key_CFG.enLowCycle = Low64;
//    KEYSCAN_Init(&key_CFG);
//    M4_KEYSCAN->SER_f.SEN = 1;
    stcEitCfg.enExitCh = ExtiCh03;
    stcEitCfg.enExtiLvl = ExIntFallingEdge;
    stcEitCfg.enFilterEn = Enable;
    stcEitCfg.enFltClk = Pclk3Div64;
//    stcEitCfg.pfnExtiCallback = key0_int_callback;
    EXINT_Init(&stcEitCfg);
    
    stcEitCfg.enExitCh = ExtiCh04;
//    stcEitCfg.pfnExtiCallback = key1_int_callback;
    EXINT_Init(&stcEitCfg);
//    
    stcIrqRegiCfg.enIntSrc = INT_PORT_EIRQ3;
    stcIrqRegiCfg.enIRQn = Key0_IRQn;
    stcIrqRegiCfg.pfnCallback =  key0_int_callback;   
    
    enIrqRegistration(&stcIrqRegiCfg);
    
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
    
    stcIrqRegiCfg.enIntSrc = INT_PORT_EIRQ4;
    stcIrqRegiCfg.enIRQn = Key1_IRQn;
    stcIrqRegiCfg.pfnCallback =  key1_int_callback;   
    
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
    
}
void LED_Close(void)
{
    PORT_ResetBits(LED0_PORT,LED0_Pin);
    PORT_ResetBits(LED1_PORT,LED1_Pin);
    PORT_ResetBits(LED2_PORT,LED2_Pin);
    PORT_ResetBits(LED3_PORT,LED3_Pin);
}
void LED0_Toggle(void)
{
    if(LED_Status>3)
    {
        LED_Status = 0;
    }
    switch(LED_Status)
    {
        case 0:
            PORT_ResetBits(LED0_PORT,LED0_Pin);
            PORT_ResetBits(LED1_PORT,LED1_Pin);
            PORT_ResetBits(LED2_PORT,LED2_Pin);
            PORT_SetBits(LED3_PORT,LED3_Pin);
//            PORT_ResetBits(PortE,Pin04);
            break;
        case 1:
            PORT_ResetBits(LED0_PORT,LED0_Pin);
            PORT_ResetBits(LED1_PORT,LED1_Pin);
            PORT_SetBits(LED2_PORT,LED2_Pin);
            PORT_ResetBits(LED3_PORT,LED3_Pin);
//            PORT_SetBits(PortE,Pin04);
            break;
        case 2:
            PORT_ResetBits(LED0_PORT,LED0_Pin);
            PORT_SetBits(LED1_PORT,LED1_Pin);
            PORT_ResetBits(LED2_PORT,LED2_Pin);
            PORT_ResetBits(LED3_PORT,LED3_Pin);
//            PORT_SetBits(PortE,Pin04);
            break;
        case 3:
            PORT_SetBits(LED0_PORT,LED0_Pin);
            PORT_ResetBits(LED1_PORT,LED1_Pin);
            PORT_ResetBits(LED2_PORT,LED2_Pin);
            PORT_ResetBits(LED3_PORT,LED3_Pin);
//            PORT_SetBits(PortE,Pin04);
            break;
        default:
            break;
    }
    LED_Status++;
}
void Test_GPIO(void)
{
    PORT_Toggle(LED0_PORT,LED0_Pin);
}

void file_test(void)
{
    FIL fil;
    FRESULT res;
    res = f_open(&fil, "hello.txt", FA_CREATE_NEW | FA_WRITE);
}
