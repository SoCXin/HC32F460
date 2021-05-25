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
 ** \brief efm sample
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of
 **     efm
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
#define EFM_SWITCH_ADDRESS              (0x0007FFDCu)
#define EFM_SWITCH_DATA                 (0xFFFF4321u)

#define DDL_DELAY                       (1000u)

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
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /* Led initialization */
    BSP_LED_Init();

    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);

    if (Set == EFM_GetSwitchStatus())
    {
        BSP_LED_On(LED_RED); /* boot swap is on */
    }
    else
    {
        BSP_LED_On(LED_BLUE); /* boot swap is off */
    }

    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
        ;
    }

    while(1)
    {
        /* k10 */
        if (Reset == PORT_GetBit(PortB, Pin01))
        {
            if (EFM_SWITCH_DATA == *(uint32_t*)EFM_SWITCH_ADDRESS)
            {
                EFM_SectorErase(EFM_SWITCH_ADDRESS);
                Ddl_Delay1ms(DDL_DELAY);
            }
            else
            {
                EFM_SingleProgram(EFM_SWITCH_ADDRESS, EFM_SWITCH_DATA);
                Ddl_Delay1ms(DDL_DELAY);
            }
        }
    }
}
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
