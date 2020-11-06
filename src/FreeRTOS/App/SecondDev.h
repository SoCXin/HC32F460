#ifndef SECONDDEV_H
#define SECONDDEV_H
#include "User_API.h"
//void UserMain (void *param) __attribute__((section(".ARM.__at_0x00040400")));
void UserSystem_Init(void) __attribute__((section(".ARM.__at_0x00040000")));
#endif
