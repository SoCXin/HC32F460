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
 ** \brief The example of ICG SWDT Reset function
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of ICG.
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
/* KEY10 Port/Pin definition */
#define KEY10_PORT                      (PortB)
#define KEY10_PIN                       (Pin01)
#define KEY10_EXINT_CH                  (ExtiCh01)
#define KEY10_INT_SRC                   (INT_PORT_EIRQ1)
#define KEY10_IRQn                      (Int007_IRQn)

/* SWDT count cycle definition */
#define SWDT_COUNT_CYCLE                (16384u)

/* Reset source definition */
#define RESET_SWDT_TRIGGER              (0u)
#define RESET_OTHER_TRIGGER             (1u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8SysWorkSta;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  main function for ICG SWDT Reset function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /**
     ***************************************************************************
     @verbatim
     ** Modify hc32f460_icg.h file of defines
     ** #define ICG0_SWDT_HARDWARE_START        ICG_FUNCTION_ON
     **
     ** #define ICG0_SWDT_AUTS                  SWDT_AUTO_START_AFTER_RESET
     ** #define ICG0_SWDT_ITS                   SWDT_RESET_REQUEST
     ** #define ICG0_SWDT_PERI                  SWDT_COUNT_UNDERFLOW_CYCLE_16384
     ** #define ICG0_SWDT_CKS                   SWDT_COUNT_SWDTCLK_DIV1
     ** #define ICG0_SWDT_WDPT                  SWDT_0To25PCT
     ** #define ICG0_SWDT_SLTPOFF               SWDT_SPECIAL_MODE_COUNT_STOP
     @endverbatim
     **************************************************************************/
    uint16_t u16CmpVal;
    stc_rmu_rstcause_t stcRmuRstCause;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcRmuRstCause);

    /* BSP initialization */
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Get RMU information */
    RMU_GetResetCause(&stcRmuRstCause);
    if (Set == stcRmuRstCause.enSwdt)
    {
        u8SysWorkSta = RESET_SWDT_TRIGGER;
        BSP_LED_On(LED_RED);
    }
    else
    {
        u8SysWorkSta = RESET_OTHER_TRIGGER;
    }
    RMU_ClrResetFlag();

    /* Wait for SWDT module to complete initial load */
    Ddl_Delay1ms(200u);
    /* Count cycle=16384,range=0%-25% */
    u16CmpVal = SWDT_COUNT_CYCLE / 4u;

    while (1)
    {
        if (Set == BSP_KEY_GetStatus(BSP_KEY_1))
        {
            u16CmpVal = SWDT_COUNT_CYCLE / 2u;
        }

        if (SWDT_GetCountValue() < u16CmpVal)
        {
            SWDT_RefreshCounter();
            /* wait for the count value to update */
            Ddl_Delay1ms(10u);
            if (RESET_OTHER_TRIGGER == u8SysWorkSta)
            {
                BSP_LED_Toggle(LED_RED);
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
