#ifndef HW_TIMER43_H
#define HW_TIMER43_H
/* Timer4 CNT */
#define TIMER43_UNIT                     M4_TMR43
#define TIMER43_CNT_CYCLE_VAL            (1000)        // Timer4 counter cycle value

/* Timer4 OCO */
#define TIMER43_OCO_LOW_UCH               Timer4OcoOul   // only Timer4OcoOul  Timer4OcoOvl  Timer4OcoOwl
#define TIMER43_OCO_LOW_VCH               Timer4OcoOvl
#define TIMER43_OCO_LOW_WCH               Timer4OcoOwl

#define TIMER43_PWM_UCH                   (Timer4PwmU) 
#define TIMER43_PWM_VCH                   (Timer4PwmV)
#define TIMER43_PWM_WCH                   (Timer4PwmW)
/* Define port and pin for Timer4Pwm */
#define TIMER43_PWM_UH_PORT               PortB          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER43_PWM_UH_PIN                Pin09
#define TIMER43_PWM_UL_PORT               PortB          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER43_PWM_UL_PIN                Pin08
#define TIMER43_PWM_VH_PORT               PortB          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER43_PWM_VH_PIN                Pin07
#define TIMER43_PWM_VL_PORT               PortB          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER43_PWM_VL_PIN                Pin06
#define TIMER43_PWM_WH_PORT               PortB          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER43_PWM_WH_PIN                Pin05
#define TIMER43_PWM_WL_PORT               PortB          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER43_PWM_WL_PIN                Pin04

#define TIMER43_EMB_UNIT                  M4_EMB4


void Hw_Timer43_init(void);
void Timer43_EMB_Break(void);
void Timer43_EMB_UnBreak(void);
#endif




