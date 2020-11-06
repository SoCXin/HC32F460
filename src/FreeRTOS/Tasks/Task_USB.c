#include "../Tasks/Task_USB.h"
#define STACKSIZE_USB	2048
#define PRIORITY_USB	(tskIDLE_PRIORITY+3)
TaskHandle_t H_Task_USB;
extern USB_OTG_CORE_HANDLE  USB_OTG_dev;
static void Task_USB(void* param)
{
    int8_t  x=0, y=0;
    static uint8_t HID_Buffer [4]; 
//	hd_sdio_hw_init();
	    USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_FS
              USB_OTG_FS_CORE_ID,
#else
              USB_OTG_HS_CORE_ID,
#endif
              &USR_desc,
              &USBD_MSC_cb,
              &USR_cb);
    while(1)
    {
//        if(Reset == PORT_GetBit(Key0_PORT,Key0_Pin))
//        {
//            y = -5;
//        }
//        if(Reset == PORT_GetBit(Key2_PORT,Key2_Pin))
//        {
//            y = 5;
//        }
//        if(Reset == PORT_GetBit(Key1_PORT,Key1_Pin))
//        {
//            x = -5;
//        }
//        if(Reset == PORT_GetBit(Key3_PORT,Key3_Pin))
//        {
//            x = 5;
//        }
        HID_Buffer[0] = (uint8_t)0;
        HID_Buffer[1] = (uint8_t)x;
        HID_Buffer[2] = (uint8_t)y;
        HID_Buffer[3] = (uint8_t)0;
         if((HID_Buffer[1] != 0u) ||(HID_Buffer[2] != 0u))
        {
//            USBD_HID_SendReport (&USB_OTG_dev, HID_Buffer, 4u);
        }
        x = y =0;
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}
uint8_t Task_USB_Start(void)
{
	xTaskCreate(Task_USB,(const char *)"Display Task",STACKSIZE_USB,NULL,PRIORITY_USB,&H_Task_USB);
	return Ok;
}
