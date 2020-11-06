#ifndef HW_TIMER41_H
#define HW_TIMER41_H
/* Timer4 CNT */
#define TIMER41_UNIT                     M4_TMR41
#define TIMER41_CNT_CYCLE_VAL            (1000)        // Timer4 counter cycle value

/* Timer4 OCO */
#define TIMER41_OCO_LOW_UCH               Timer4OcoOul   // only Timer4OcoOul  Timer4OcoOvl  Timer4OcoOwl
#define TIMER41_OCO_LOW_VCH               Timer4OcoOvl
#define TIMER41_OCO_LOW_WCH               Timer4OcoOwl

#define TIMER41_PWM_UCH                   (Timer4PwmU) 
#define TIMER41_PWM_VCH                   (Timer4PwmV)
#define TIMER41_PWM_WCH                   (Timer4PwmW)
/* Define port and pin for Timer4Pwm */
#define TIMER41_PWM_UH_PORT               PortA          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER41_PWM_UH_PIN                Pin08
#define TIMER41_PWM_UL_PORT               PortA          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER41_PWM_UL_PIN                Pin07
#define TIMER41_PWM_VH_PORT               PortA          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER41_PWM_VH_PIN                Pin09
#define TIMER41_PWM_VL_PORT               PortB          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER41_PWM_VL_PIN                Pin00
#define TIMER41_PWM_WH_PORT               PortA          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER41_PWM_WH_PIN                Pin10
#define TIMER41_PWM_WL_PORT               PortB          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER41_PWM_WL_PIN                Pin15

#define TIMER41_EMB_UNIT                  M4_EMB2


void Hw_Timer41_init(void);
void Timer41_EMB_Break(void);
void Timer41_EMB_UnBreak(void);
#endif

