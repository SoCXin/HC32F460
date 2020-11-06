#include "../Tasks/Task_display.h"
#define STACKSIZE_DISPLAY	configMINIMAL_STACK_SIZE
#define PRIORITY_DISPLAY	(tskIDLE_PRIORITY+3)
TaskHandle_t H_Task_Display;

static void Task_Display(void* param)
{
	OLED_Init();
//	TestDrawline();
//	OLED_ShowString2(0,16,(unsigned char *)"HC32F460 DEMO");
	while(1)
	{
		OLED_Refresh();
        vTaskDelay(10/portTICK_PERIOD_MS);
	}
}
uint8_t Task_Display_Start(void)
{
	xTaskCreate(Task_Display,(const char *)"Display Task",STACKSIZE_DISPLAY,NULL,PRIORITY_DISPLAY,&H_Task_Display);
	return Ok;
}

