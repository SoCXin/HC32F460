#ifndef USER_ADC_H
#define USER_ADC_H
#include "hc32_ddl.h"
uint16_t Get_AIN10Data(void);
void Get_ADC_V11(uint16_t *data);
void Set_ADC_Data(uint16_t data);
void ADC_EOCA_CallBack(void);
void User_ADC_Init(void);
void ADC1_Start_convert(void);
#endif
