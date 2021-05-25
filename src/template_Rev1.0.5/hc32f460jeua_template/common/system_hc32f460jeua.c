/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file system_hc32f460.c
 **
 ** System clock initialization.
 **
 **   - 2018-1-15  CDT  First version
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_common.h"

/*******************************************************************************
 * Global pre-processor symbols/macros ('define')
 ******************************************************************************/

/**
 ******************************************************************************
 ** System Clock Frequency (Core Clock) Variable according CMSIS
 ******************************************************************************/
uint32_t SystemCoreClock = MRC_VALUE;

/**
 ******************************************************************************
 ** \brief  Setup the microcontroller system. Initialize the System and update
 ** the SystemCoreClock variable.
 **
 ** \param  none
 ** \return none
 ******************************************************************************/
void SystemInit(void)
{
    SystemCoreClockUpdate();
}

void SystemCoreClockUpdate (void) // Update SystemCoreClock variable
{
    uint8_t tmp=0, plln = 19, pllp = 1, pllm = 0, pllsource = 0;
    tmp = M4_SYSREG->CMU_CKSWR_f.CKSW;
    switch (tmp)
    {
        case 0x00:  /* use internal high speed RC */
            SystemCoreClock = HRC_VALUE;
        break;

        case 0x01:  /* use internal middle speed RC */
            SystemCoreClock = MRC_VALUE;
        break;

        case 0x02:  /* use internal low speed RC */
            SystemCoreClock = LRC_VALUE;
        break;

        case 0x03:  /* use external high speed RC */
            SystemCoreClock = XTAL_VALUE;
        break;

        case 0x04:  /* use external low speed RC */
            SystemCoreClock = XTAL32_VALUE;
        break;

        case 0x05:  /* use MPLL */
            /* PLLCLK = ((pllsrc / pllm) * plln) / pllp */
            pllsource = M4_SYSREG->CMU_PLLCFGR_f.PLLSRC;
            plln = M4_SYSREG->CMU_PLLCFGR_f.MPLLN;
            pllp = M4_SYSREG->CMU_PLLCFGR_f.MPLLP;
            pllm = M4_SYSREG->CMU_PLLCFGR_f.MPLLM;
            /* use exteranl high speed OSC as PLL source */
            if (0 == pllsource)
            {
                SystemCoreClock = (XTAL_VALUE)/(pllm+1)*(plln+1)/(pllp + 1);
            }
            /* use interanl high RC as PLL source */
            else if (1 == pllsource)
            {
                SystemCoreClock = (HRC_VALUE)/(pllm+1)*(plln+1)/(pllp + 1);
            }
        break;

    }
}
