#include "SecondDev.h"
#include "User_API.h"
#include "User_Gpio.h"

void UserMain(void *param)
{
	while(1)
	{
		PORT_Toggle(LED0_PORT,LED0_Pin);
		API_Delay(1000);
	}
}
void UserSystem_Init(void)
{
	Create_Task(UserMain);
}