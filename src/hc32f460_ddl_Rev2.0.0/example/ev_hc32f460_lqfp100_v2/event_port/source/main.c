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
 ** \brief Event port sample
 **
 **   - 2021-04-16 CDT First version for Device Driver Library of event port.
 **
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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Main function of event port project
 **
 ** \param  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_event_port_init_t stcEPConfig;

    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);
    /* Set PB1 as Event Port 2.1 */
    PORT_SetFunc(PortB, Pin01, Func_Evnpt, Disable);

    /* Set PD3, PD15 as Event Port 4.3, Event Port 4.15 */
    PORT_SetFunc(PortD, (Pin03 | Pin15), Func_Evnpt, Disable);

    /* Enable Event port operation clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Set Event Port 2.1 falling edge detect enable */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enFallingDetect = Enable;
    EVENTPORT_Init(EventPort2, EventPin01, &stcEPConfig);

    /* Set Event Port 2 event as the trigger source for Event Port 4*/
    EVENTPORT_SetTriggerSrc(EventPort4, EVT_EVENT_PORT2);

    /* Set Event Port 4.3 as output function */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enDirection = EventPortOut;
    stcEPConfig.enSet = Enable;
    stcEPConfig.enReset = Enable;
    EVENTPORT_Init(EventPort4, EventPin03, &stcEPConfig);

    /* Set Event Port 4.15 as input function */
    MEM_ZERO_STRUCT(stcEPConfig);
    stcEPConfig.enDirection = EventPortIn;
    EVENTPORT_Init(EventPort4, EventPin15, &stcEPConfig);

    while(1)
    {
        /* Event Port4.15 is set after trigger */
        if (Set == EVENTPORT_GetBit(EventPort4, EventPin15))
        {
            DDL_Printf("EventPort4_15 is set.\n");
            break;
        }
    }
    /* de-init event port if necessary */
//    EVENTPORT_DeInit();
    while (1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
