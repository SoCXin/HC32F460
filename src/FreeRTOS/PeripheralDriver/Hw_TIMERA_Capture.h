#ifndef HW_TIMA_CAP_H
#define HW_TIMA_CAP_H
#include "hc32_ddl.h"
#define UNIT_CAPTIM						(M4_TMRA1)
/* TIMERA unit and clock definition */
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TIMERA_UNIT1_COMPARE_INT        (INT_TMRA1_CMP)

/* TIMERA channel 1 Port/Pin definition */
#define TIMERA_UNIT1_CH1                (TimeraCh1)
#define TIMERA_UNIT1_CH1_PORT           (PortE)
#define TIMERA_UNIT1_CH1_PIN            (Pin09)
#define TIMERA_UNIT1_CH1_FUNC           (Func_Tima0)
#define TIMERA_UNIT1_CH1_INT_FLAG       (TimeraFlagCaptureOrCompareCh1)
#define TIMERA_UNIT1_CH1_INT            (TimeraIrqCaptureOrCompareCh1)

/* TIMERA channel 2 Port/Pin definition */
#define TIMERA_UNIT1_CH2                (TimeraCh2)
#define TIMERA_UNIT1_CH2_PORT           (PortE)
#define TIMERA_UNIT1_CH2_PIN            (Pin11)
#define TIMERA_UNIT1_CH2_FUNC           (Func_Tima0)
#define TIMERA_UNIT1_CH2_INT_FLAG       (TimeraFlagCaptureOrCompareCh2)
#define TIMERA_UNIT1_CH2_INT            (TimeraIrqCaptureOrCompareCh2)


#ifdef __cplusplus
extern "C" {
#endif
void TimerACaptureInit(void);
uint32_t GetTime(void);
uint32_t GetFrequence(void);
#ifdef __cplusplus
};
#endif
#endif

