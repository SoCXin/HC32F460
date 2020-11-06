#ifndef USER_TIMERA_32BIT_H
#define USER_TIMERA_32BIT_H

#include "hc32_ddl.h"

#define TIMERA_L16_CLOCK PWC_FCG2_PERIPH_TIMA1
#define TIMERA_UnitL                    M4_TMRA1
#define TIMERA_UNITL_OVERFLOW_INT       INT_TMRA1_OVF

#define TIMERA_UNIT1_COMPARE_INT        INT_TMRA1_CMP
#define TIMERA_UNIT1_CH1                TimeraCh1
#define TIMERA_UNIT1_CH1_PORT           PortE
#define TIMERA_UNIT1_CH1_PIN            Pin09
#define TIMERA_UNIT1_CH1_FUNC           Func_Tima0
#define TIMERA_UNIT1_CH1_INT_FLAG       TimeraFlagCaptureOrCompareCh1
#define TIMERA_UNIT1_CH1_INT            TimeraIrqCaptureOrCompareCh1

#define TIMERA_UNIT1_CH2                TimeraCh2
#define TIMERA_UNIT1_CH2_PORT           PortE
#define TIMERA_UNIT1_CH2_PIN            Pin11
#define TIMERA_UNIT1_CH2_FUNC           Func_Tima0
#define TIMERA_UNIT1_CH2_INT_FLAG       TimeraFlagCaptureOrCompareCh2
#define TIMERA_UNIT1_CH2_INT            TimeraIrqCaptureOrCompareCh2

#define TIMERA_H16_CLOCK PWC_FCG2_PERIPH_TIMA2
#define TIMERA_UnitH                    M4_TMRA2
#define TIMERA_UNITH_OVERFLOW_INT       INT_TMRA2_OVF

#define TIMERA1_IRQn            Int001_IRQn
#define TIMERA2_IRQn            Int002_IRQn



void TimerA_L16Bit_config(void);
void TimerA_H16Bit_config(void);
































#endif




