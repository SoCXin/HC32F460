#ifndef USER_API_H
#define USER_API_H
#include "cmsis_os.h"
#include "User_Gpio.h"
void Create_Task(TaskFunction_t pfunc) __attribute__((section(".ARM.__at_0x00030000")));
void API_Delay(uint32_t ms) __attribute__((section(".ARM.__at_0x00030400")));
void API_TogLed(uint32_t ms) __attribute__((section(".ARM.__at_0x00030C00")));
//void User_System_Init(void) __attribute__((section(".ARM.__at_0x00031000")));
#endif

