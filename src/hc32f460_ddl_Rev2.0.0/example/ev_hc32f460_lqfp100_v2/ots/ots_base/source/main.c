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
 ** \brief OTS sample
 **
 **   - 2021-04-16  CDT First version for Device Driver Library of
 **     OTS
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
/* OTS clock selection. */
#define OTS_CLK_SEL_XTAL            (0u)
#define OTS_CLK_SEL_HRC             (1u)

/* Select XTAL as OTS clock. */
#define OTS_CLK_SEL                 (OTS_CLK_SEL_XTAL)

/* Function control. Non-zero to enable. */
#define OTS_USE_TRIG                (1u)

#if (OTS_USE_TRIG > 0U)
    #define OTS_USE_INTERRUPT       (OTS_USE_TRIG)
#else
    #define OTS_USE_INTERRUPT       (1u)
#endif

/*
 * Definitions about OTS interrupt for the example.
 * OTS independent IRQn: [Int000_IRQn, Int031_IRQn], [Int110_IRQn, Int113_IRQn].
 */
#if (OTS_USE_INTERRUPT > 0u)
    #define OTS_INT_PRIO            (DDL_IRQ_PRIORITY_03)
    #define OTS_INT_SRC             (INT_OTS)
    #define OTS_IRQn                (Int113_IRQn)
#endif /* #if (OTS_USE_INTERRUPT > 0u) */

/* OTS parameters, slope K and offset M. Different chip, different parameters. */
#define OTS_XTAL_K                  (737272.73f)
#define OTS_XTAL_M                  (27.55f)
#define OTS_HRC_K                   (3002.59f)
#define OTS_HRC_M                   (27.92f)

/* Timeout value. */
#define TIMEOUT_VAL                 (10000u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void OtsConfig(void);
static void OtsInitConfig(void);
static void OtsClockConfig(void);

#if (OTS_USE_INTERRUPT > 0u)
    static void OtsIrqConfig(void);
    static void OtsIrqCallback(void);
#endif

#if (OTS_USE_TRIG > 0u)
    static void OtsTrigConfig(void);
#endif

#if ((OTS_USE_INTERRUPT > 0u) || (OTS_USE_TRIG > 0u))
    static void OtsStart(void);
#endif

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static float32_t m_f32Temperature = 0.0f;

#if (OTS_USE_INTERRUPT > 0u)
static uint8_t m_u8OtsIntFlag = 0u;
#endif

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None.
 **
 ** \retval int32_t return value, if needed.
 **
 ******************************************************************************/
int32_t main(void)
{
    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    /* Config OTS. */
    OtsConfig();

    /***************** Configuration end, application start **************/
#if ((OTS_USE_INTERRUPT > 0U) || (OTS_USE_TRIG > 0U))
    /* Starts OTS. */
    OtsStart();
#endif

    while (1u)
    {
#if (OTS_USE_INTERRUPT > 0U)
        if (m_u8OtsIntFlag != 0U)
        {
            m_u8OtsIntFlag = 0U;
            DDL_Printf("Temperature: %.2f\n", m_f32Temperature);
#if (OTS_USE_TRIG == 0U)
            Ddl_Delay1ms(1000U);
            OTS_Start();
#endif /* #if (OTS_USE_TRIG == 0U) */
        }
#else
        (void)OTS_Polling(&m_f32Temperature, TIMEOUT_VAL);
        DDL_Printf("Temperature: %.2f\n", m_f32Temperature);
        m_f32Temperature = 0.0f;
        Ddl_Delay1ms(1000U);
#endif /* #if (OTS_USE_INTERRUPT > 0U) */
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  OTS configuration, including initial configuration and
 **         clock configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsConfig(void)
{
    OtsInitConfig();
    OtsClockConfig();
}

/**
 *******************************************************************************
 ** \brief  OTS initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsInitConfig(void)
{
    stc_ots_init_t stcOtsInit;

    stcOtsInit.enAutoOff = OtsAutoOff_Disable;
#if (OTS_CLK_SEL == OTS_CLK_SEL_HRC)
    stcOtsInit.enClkSel   = OtsClkSel_Hrc;
    stcOtsInit.f32SlopeK  = OTS_HRC_K;
    stcOtsInit.f32OffsetM = OTS_HRC_M;
#else
    stcOtsInit.enClkSel   = OtsClkSel_Xtal;
    stcOtsInit.f32SlopeK  = OTS_XTAL_K;
    stcOtsInit.f32OffsetM = OTS_XTAL_M;
#endif

    /* 1. Enable OTS. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_OTS, Enable);
    /* 2. Initialize OTS. */
    OTS_Init(&stcOtsInit);
#if (OTS_USE_INTERRUPT > 0U)
    OtsIrqConfig();
#endif

#if (OTS_USE_TRIG > 0U)
    OtsTrigConfig();
#endif
}

/**
 *******************************************************************************
 ** \brief  OTS clock configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsClockConfig(void)
{
#if (OTS_CLK_SEL == OTS_CLK_SEL_HRC)
    /* Enable HRC for OTS. */
    CLK_HrcCmd(Enable);
    /* Enable XTAL32 while clock selecting HRC. */
    CLK_Xtal32Cmd(Enable);
#else
    /* Enable XTAL for OTS. */
    CLK_XtalCmd(Enable);
#endif

    /* Enable LRC for OTS. */
    CLK_LrcCmd(Enable);
}

#if (OTS_USE_INTERRUPT > 0u)
/**
 *******************************************************************************
 ** \brief  OTS interrupt configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsIrqConfig(void)
{
    stc_irq_regi_conf_t stcOtsIrqCfg;
    en_result_t         enIrqRegResult;

    stcOtsIrqCfg.enIntSrc    = OTS_INT_SRC;
    stcOtsIrqCfg.enIRQn      = OTS_IRQn;
    stcOtsIrqCfg.pfnCallback = &OtsIrqCallback;
    enIrqRegResult           = enIrqRegistration(&stcOtsIrqCfg);

    if (Ok == enIrqRegResult)
    {
        NVIC_ClearPendingIRQ(stcOtsIrqCfg.enIRQn);
        NVIC_SetPriority(stcOtsIrqCfg.enIRQn, OTS_INT_PRIO);
        NVIC_EnableIRQ(stcOtsIrqCfg.enIRQn);

        /* Enable the specified interrupts of OTS. */
        OTS_IntCmd(Enable);
    }
}

/**
 *******************************************************************************
 ** \brief  OTS interrupt callback function.
 **         Its main function is to get temperature value.
 **
 ******************************************************************************/
static void OtsIrqCallback(void)
{
    m_f32Temperature = OTS_CalculateTemp();
    m_u8OtsIntFlag = 1U;
}
#endif


#if (OTS_USE_TRIG > 0U)
/**
 *******************************************************************************
 ** \brief  Specifies an event as the trigger source event of OTS.
 **
 ******************************************************************************/
static void OtsTrigConfig(void)
{
    /*
     * If a peripheral is used to generate the event which is used as a start trigger condition of OTS, \
     *   call the API of the peripheral to configure the peripheral.
     * The following operations are only used in this example.
     */
    stc_tim0_base_init_t stcTimerCfg;

    MEM_ZERO_STRUCT(stcTimerCfg);

    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable);

    /*config register for channel B */
    stcTimerCfg.Tim0_CounterMode     = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision   = Tim0_ClkDiv128;
    stcTimerCfg.Tim0_CmpValue        = 62500u - 1u;
    TIMER0_BaseInit(M4_TMR02, Tim0_ChannelB, &stcTimerCfg);

    /* Specifies event 'EVT_TMR02_GCMB' as the trigger source event of OTS. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
    OTS_SetTriggerSrc(EVT_TMR02_GCMB);
}
#endif /* #if (OTS_USE_TRIG > 0U) */

#if ((OTS_USE_INTERRUPT > 0U) || (OTS_USE_TRIG > 0U))
/**
 *******************************************************************************
 ** \brief  Start OTS.
 **
 ******************************************************************************/
static void OtsStart(void)
{
    /*
     * If a peripheral is used to generate the event which is used as a start trigger condition of OTS, \
     *   call the API of the peripheral to start the peripheral here or anywhere else you need.
     * The following operations are only used in this example.
     */

#if (OTS_USE_TRIG > 0U)
    /*start timer0*/
    TIMER0_Cmd(M4_TMR02, Tim0_ChannelB, Enable);
#elif (OTS_USE_INTERRUPT > 0U)
    OTS_Start();
#endif
}
#endif /* #if ((OTS_USE_INTERRUPT > 0U) || (OTS_USE_TRIG > 0U)) */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
