/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_bsp.c
 **
 ** \brief  This file is responsible to offer board support package and is
 **         configurable by user.
 **
 **   - 2018-12-25  1.0  CDT First version for USB composite device demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_bsp.h"
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
uint8_t PrevXferDone = 1u;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
extern  usb_core_instance      usb_dev;
extern  void hd_usb_isr_handler (usb_core_instance *pdev);

/**
 ******************************************************************************
 ** \brief  Usb interrupt handle
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
void USB_IRQ_Handler(void)
{
    hd_usb_isr_handler(&usb_dev);
}

/**
 ******************************************************************************
 ** \brief  Initialize the usb clock for the sample
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
static void UsbClkIni(void)
{
    stc_clk_upll_cfg_t      stcUpllCfg;

    stcUpllCfg.pllmDiv = 2u;
    stcUpllCfg.plln = 84u;
    stcUpllCfg.PllpDiv = 7u;//48M
    stcUpllCfg.PllqDiv = 7u;
    stcUpllCfg.PllrDiv = 7u;
    CLK_UpllConfig(&stcUpllCfg);
    CLK_UpllCmd(Enable);
    /* Wait UPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagUPLLRdy))
    {
        ;
    }

    /* Set USB clock source */
    CLK_SetUsbClkSource(ClkUsbSrcUpllp);
}

/**
 ** \brief  Initilizes BSP configurations
 **
 ** \param  None
 ** \retval None
 ******************************************************************************/
void hd_usb_bsp_init(usb_core_instance *pdev)
{
    stc_port_init_t stcPortInit;

   /* clock config */
    BSP_CLK_Init();
    UsbClkIni();

    /* LED port initialize */
    BSP_LED_Init();

    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);
    DDL_Printf("USBFS start !!\n");

    /* port config */
    /* Disable digital function for DM DP */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Ana;
    PORT_Init(PortA, Pin11, &stcPortInit);
    PORT_Init(PortA, Pin12, &stcPortInit);
    PORT_SetFunc(PortA, Pin09, Func_UsbF, Disable); //VBUS
    PORT_SetFunc(PortA, Pin11, Func_UsbF, Disable); //DM
    PORT_SetFunc(PortA, Pin12, Func_UsbF, Disable); //DP

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USBFS, Enable);
}

/**
 *******************************************************************************
 ** \brief  Enabele USB Global interrupt
 ** \param  None
 ** \retval None
 ******************************************************************************/
void hd_usb_bsp_nvicconfig(void)
{
    stc_irq_regi_conf_t stcIrqRegiConf;
    /* Register INT_USBFS_GLB Int to Vect.No.024 */
    stcIrqRegiConf.enIRQn = Int024_IRQn;
    /* Select INT_USBFS_GLB interrupt function */
    stcIrqRegiConf.enIntSrc = INT_USBFS_GLB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &USB_IRQ_Handler;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief  This function provides delay time in micro sec
 ** \param  usec : Value of delay required in micro sec
 ** \retval None
 ******************************************************************************/
#define Fclk SystemCoreClock
void hd_usb_udelay(const uint32_t usec)
{
    __IO uint32_t i;
    uint32_t j = Fclk / 1000000ul * usec;
    for(i = 0ul; i < j; i++)
    {
        ;
    }
}

/**
 *******************************************************************************
 ** \brief  This function provides delay time in milli sec
 ** \param  msec : Value of delay required in milli sec
 ** \retval None
 ******************************************************************************/
void hd_usb_mdelay (const uint32_t msec)
{
    hd_usb_udelay(msec * 1000ul);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
