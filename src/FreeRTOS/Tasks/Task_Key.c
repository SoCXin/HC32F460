#include"../Tasks/Task_Key.h"
TaskHandle_t H_Task_KEY;
#define STACKSIZE_KEY	configMINIMAL_STACK_SIZE
#define PRIORITY_TASKKEY	(tskIDLE_PRIORITY+3)
static void Task_KeyScan(void* param)
{
	while(1)
		{
			
		}
}
uint8_t Task_KeyScan_Start(void)
{
	xTaskCreate(Task_KeyScan,(const char *)"File System Task",STACKSIZE_KEY,NULL,PRIORITY_TASKKEY,&H_Task_KEY);
	return Ok;
}
uint8_t Task_KeyScan_Suspend(void)
{
	vTaskSuspend(H_Task_KEY);
	return Ok;
}
uint8_t Task_KeyScan_Resume(void)
{
	vTaskResume(H_Task_KEY);
	return Ok;
}
uint8_t Task_KeyScan_Stop(void)
{
	vTaskDelete(H_Task_KEY);
	return Ok;
}

