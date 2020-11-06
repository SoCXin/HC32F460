#ifndef USER_GPIO_H
#define USER_GPIO_H
#include "hc32_ddl.h"
#define LED0_PORT   PortE
#define LED0_Pin    Pin06
#define LED1_PORT   PortA
#define LED1_Pin    Pin07
#define LED2_PORT   PortB
#define LED2_Pin    Pin05
#define LED3_PORT   PortB
#define LED3_Pin    Pin09
#define Key0_PORT   PortD
#define Key0_Pin    Pin03
#define Key1_PORT   PortD
#define Key1_Pin    Pin04

extern bool flag_key0,flag_key1;
void User_Gpio_Init(void);
void LED0_Toggle(void);
void LED_Close(void);
void Test_GPIO(void);
void file_test(void);
#endif
