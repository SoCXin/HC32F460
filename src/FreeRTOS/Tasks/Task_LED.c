#include "../Tasks/Task_LED.h"
#include "System_PowerDown.h"
TaskHandle_t H_Task_LED;
#define STACKSIZE_LED	configMINIMAL_STACK_SIZE
#define PRIORITY_TASKLED	(tskIDLE_PRIORITY+3)
static void Task_LED(void* param)
{
	User_Gpio_Init();	
	while(1)
		{
//			LED0_Toggle();
		    vTaskDelay(1000/portTICK_PERIOD_MS);
		}
}
uint8_t Task_LED_Start(void)
{
	xTaskCreate(Task_LED,(const char *)"LED Task",STACKSIZE_LED,NULL,PRIORITY_TASKLED,&H_Task_LED);
	return Ok;
}
