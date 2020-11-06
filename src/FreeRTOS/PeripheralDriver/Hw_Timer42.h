#ifndef HW_TIMER42_H
#define HW_TIMER42_H
/* Timer4 CNT */
#define TIMER42_UNIT                     M4_TMR42
#define TIMER42_CNT_CYCLE_VAL            (1000)        // Timer4 counter cycle value

/* Timer4 OCO */
#define TIMER42_OCO_LOW_UCH               Timer4OcoOul   // only Timer4OcoOul  Timer4OcoOvl  Timer4OcoOwl
#define TIMER42_OCO_LOW_VCH               Timer4OcoOvl
#define TIMER42_OCO_LOW_WCH               Timer4OcoOwl

#define TIMER42_PWM_UCH                   (Timer4PwmU) 
#define TIMER42_PWM_VCH                   (Timer4PwmV)
#define TIMER42_PWM_WCH                   (Timer4PwmW)
/* Define port and pin for Timer4Pwm */
#define TIMER42_PWM_UH_PORT               PortA          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER42_PWM_UH_PIN                Pin00
#define TIMER42_PWM_UL_PORT               PortA          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER42_PWM_UL_PIN                Pin01
#define TIMER42_PWM_VH_PORT               PortA          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER42_PWM_VH_PIN                Pin02
#define TIMER42_PWM_VL_PORT               PortA          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER42_PWM_VL_PIN                Pin03
#define TIMER42_PWM_WH_PORT               PortA          // TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13
#define TIMER42_PWM_WH_PIN                Pin04
#define TIMER42_PWM_WL_PORT               PortA          // TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12
#define TIMER42_PWM_WL_PIN                Pin05

#define TIMER42_EMB_UNIT                  M4_EMB3


void Hw_Timer42_init(void);
void Timer42_EMB_Break(void);
void Timer42_EMB_UnBreak(void);
#endif




