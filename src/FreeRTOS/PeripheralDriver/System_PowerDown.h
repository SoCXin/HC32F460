#ifndef SYSTEM_POWERDOWN
#define SYSTEM_POWERDOWN
#include "hc32_ddl.h"

#define KEY_WK0_PORT    PortB
#define KEY_WK0_PIN     Pin01
#define KEY_WK0_INT     INT_PORT_EIRQ1
#define KEY_WK0_EICH    ExtiCh01

#define KEY_WK1_PORT    PortB
#define KEY_WK1_PIN     Pin05
#define KEY_WK1_INT     INT_PORT_EIRQ5
#define KEY_WK1_EICH    ExtiCh05

#define KEY_WK2_PORT    PortA
#define KEY_WK2_PIN     Pin08
#define KEY_WK2_INT     INT_PORT_EIRQ8
#define KEY_WK2_EICH    ExtiCh08

#define KEY_WK3_PORT    PortA
#define KEY_WK3_PIN     Pin15
#define KEY_WK3_INT     INT_PORT_EIRQ15
#define KEY_WK3_EICH    ExtiCh15



#define KEY_WK0_EN  PWC_PDWKEN0_WKUP01
#define KEY_WK1_EN  PWC_PDWKEN0_WKUP11
#define KEY_WK2_EN  PWC_PDWKEN1_WKUP20
#define KEY_WK3_EN  PWC_PDWKEN1_WKUP33



#ifdef __cplusplus
extern "C" {
#endif

void System_Enter_StopMode(void);
void System_Enter_PowerDown(void);
uint16_t GetWakeupFlag(void);
void Key_Wakeup_Init(void);
void LPM_TEST(void);

#ifdef __cplusplus
};
#endif




#endif
