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
 ** \brief This sample demonstrates how to set GPIO as output function.
 **
 **   - 2021-04-16 CDT first version for Device Driver Library of GPIO.
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
/* LED0 Port/Pin definition */
#define  LED0_PORT        (PortB)
#define  LED0_PIN         (Pin04)

/* LED1 Port/Pin definition */
#define  LED1_PORT        (PortA)
#define  LED1_PIN         (Pin07)

/* LED2 Port/Pin definition */
#define  LED2_PORT        (PortB)
#define  LED2_PIN         (Pin03)

/* LED3 Port/Pin definition */
#define  LED3_PORT        (PortB)
#define  LED3_PIN         (Pin06)

/* LED0~3 toggle definition */
#define  LED0_TOGGLE()    (PORT_Toggle(LED0_PORT, LED0_PIN))
#define  LED1_TOGGLE()    (PORT_Toggle(LED1_PORT, LED1_PIN))
#define  LED2_TOGGLE()    (PORT_Toggle(LED2_PORT, LED2_PIN))
#define  LED3_TOGGLE()    (PORT_Toggle(LED3_PORT, LED3_PIN))

#define  DLY_MS           (100ul)

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
 ** \brief  Main function of GPIO output
 **
 ** \param  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* LED2 Port/Pin initialization */
    PORT_Init(LED2_PORT, LED2_PIN, &stcPortInit);

    /* LED3 Port/Pin initialization */
    PORT_Init(LED3_PORT, LED3_PIN, &stcPortInit);

    while(1)
    {
        LED0_TOGGLE();
        Ddl_Delay1ms(DLY_MS);
        LED1_TOGGLE();
        Ddl_Delay1ms(DLY_MS);
        LED2_TOGGLE();
        Ddl_Delay1ms(DLY_MS);
        LED3_TOGGLE();
        Ddl_Delay1ms(DLY_MS);
        /* de-init if necessary */
        //PORT_DeInit();
    };
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
