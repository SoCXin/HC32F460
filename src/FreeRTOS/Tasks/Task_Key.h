#ifndef TASK_KEY_H
#define TASK_KEY_H
#include "hc32_ddl.h"
#include "User_Gpio.h"
#include "cmsis_os.h"

uint8_t Task_KeyScan_Start(void);
uint8_t Task_KeyScan_Suspend(void);
uint8_t Task_KeyScan_Resume(void);
uint8_t Task_KeyScan_Stop(void);
#endif
