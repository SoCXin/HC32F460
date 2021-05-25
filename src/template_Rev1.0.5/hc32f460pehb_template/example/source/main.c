/**
 *******************************************************************************
 * @file  main.c
 * @brief Main program template.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2020-06-30        CDT         First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
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
 /**
 * @brief ICG parameters configuration
 */
 /* The ICG area is filled with F by default, HRC = 16MHZ,
    Please modify this value as required */
#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
const uint32_t u32ICG[] __attribute__((section(".icg_sec"))) =
#elif defined (__CC_ARM)
const uint32_t u32ICG[] __attribute__((at(0x400))) =
#elif defined (__ICCARM__)
__root const uint32_t u32ICG[] @ 0x400 =
#else
#error "unsupported compiler!!"
#endif
{
    /* ICG 0~ 3 */
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
    /* ICG 4~ 7 */
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
    0xFFFFFFFFUL,
};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 * @brief  Main function of template project
 * @param  None
 * @retval int32_t return value, if needed
 */
int32_t main(void)
{
    /* add your code here */
    while(1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
