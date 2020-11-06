#include "User_API.h"
TaskHandle_t Task_list[100] __attribute__((section(".ARM.__at_0x00030800")));
static uint8_t handel_num;
void API_Delay(uint32_t ms)
{
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
void Create_Task(TaskFunction_t pfunc)
{
	if(handel_num>99)
	{
		return;
	}
	xTaskCreate(pfunc,(const char *)"LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, &Task_list[handel_num] );
	handel_num++;	
}
void API_TogLed(uint32_t ms)
{
	PORT_Toggle(LED1_PORT,LED1_Pin);
}	


