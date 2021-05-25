/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief This example demonstrates how to use the reload timer mode function
 **        of Timer4Pwm.
 **
 **   - 2021-04-16 CDT First version for Device Driver Library of Timer4Pwm
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Timer4 CNT */
#define TIMER4_UNIT                     (M4_TMR41)

/* Timer4 PWM */
#define TIMER4_PWM_CH                   (Timer4PwmU)    /* Timer4PwmU  Timer4PwmV  Timer4PwmW */
#define TIMER4_PWM_RT_VAL               (50000u)

/* Timer4 RT interrupt number */
#define TIMER4_RT_INT_NUM               (INT_TMR41_RLOU)

/* Wave I/O */
#define WAVE_IO_PORT                    (PortE)
#define WAVE_IO_PIN                     (Pin09)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ToggleIo(void);
static void PwmCoupleChIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Toggle gpio.
 **
 ** \param [in]                   None
 **
 ** \retval                       None
 **
 ******************************************************************************/
static void ToggleIo(void)
{
    PORT_Toggle(WAVE_IO_PORT, WAVE_IO_PIN);
    Ddl_Delay1ms(5ul);
    PORT_Toggle(WAVE_IO_PORT, WAVE_IO_PIN);
}

/**
 *******************************************************************************
 ** \brief pwm couple channel reload timer interrupt handler
 **
 ******************************************************************************/
static void PwmCoupleChIrqCallback(void)
{
    ToggleIo();

    TIMER4_PWM_ClearIrqFlag(TIMER4_UNIT, TIMER4_PWM_CH);
}

/**
 *******************************************************************************
 ** \brief  Main function of project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_port_init_t stcPortInit;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_pwm_init_t stcPwmInit;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);

    /* Initialize Port/Pin */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(WAVE_IO_PORT, WAVE_IO_PIN, &stcPortInit);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 PWM: Initialize PWM couple channel configuration structure */
    stcPwmInit.enMode = PwmDeadTimerMode;  /* Change: PwmDeadTimerMode  PwmThroughMode */
    stcPwmInit.enClkDiv = PwmPlckDiv16;
    stcPwmInit.enRtIntMaskCmd = Disable;
    stcPwmInit.enOutputState = PwmHPwmLHold;  /* change: PwmHPwmLHold  PwmHPwmLReverse  PwmHReversePwmLHold  PwmHHoldPwmLReverse */
    TIMER4_PWM_Init(TIMER4_UNIT, TIMER4_PWM_CH, &stcPwmInit);  /* Initialize PWM channel */
    TIMER4_PWM_SetFilterCountValue(TIMER4_UNIT, TIMER4_PWM_CH, TIMER4_PWM_RT_VAL);

    /* Set Timer4 PWM RT IRQ */
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = &PwmCoupleChIrqCallback;
    stcIrqRegiCfg.enIntSrc = TIMER4_RT_INT_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Start pwm count */
    TIMER4_PWM_StartTimer(TIMER4_UNIT, TIMER4_PWM_CH);

    /* Toggle I/O */
    ToggleIo();

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
