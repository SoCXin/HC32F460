#ifndef USER_TIMER_H
#define USER_TIMER_H
#include "hc32_ddl.h"
void Timer02_Init(void);
en_result_t Timer_Delay_us(uint16_t us);
void Delay_ms(uint16_t ms);
void Delay_us(uint16_t us);
en_result_t Timer02B_Stop(void);
en_result_t Timer02B_Start(uint16_t Count);

#endif
