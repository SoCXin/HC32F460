#ifndef TASK_ADC_H
#define TASK_ADC_H
#include "hc32_ddl.h"
#include "cmsis_os.h"
#include "User_DCU.h"
#include "User_DMA.h"
#include "User_ADC.h"
#include "OLED.h"
#include "ff.h"
#include "diskio.h"
#include "SEGGER_RTT.h"
#ifdef __cplusplus
extern "C" {
#endif
	
uint8_t Task_ADC_Start(void);
void Get_ADC1_Data(uint16_t *data);	
#ifdef __cplusplus
};
#endif

#endif
