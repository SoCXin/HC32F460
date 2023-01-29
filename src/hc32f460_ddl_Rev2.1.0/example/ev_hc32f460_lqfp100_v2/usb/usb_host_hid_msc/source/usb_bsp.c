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
 ** A detailed description is available at
 ** @link
        This file is responsible to offer board support package and is
        configurable by user.
    @endlink
 **
 **   - 2021-04-16  CDT First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_bsp.h"
#include "hc32_ddl.h"
#include "usb_host_int.h"

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
void USB_IRQ_Handler(void)
{
    hd_usb_host_isr(&usb_app_instance);
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
 *******************************************************************************
 ** \brief  Initilizes BSP configurations
 ** \param  none
 ** \retval none
 ******************************************************************************/
void hd_usb_bsp_init(usb_core_instance *pdev)
{
    stc_port_init_t stcPortInit;
    /* clock config */
    BSP_CLK_Init();
    UsbClkIni();
    BSP_KEY_Init();

    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);
    DDL_Printf("USBFS start !!\n");

    /* port config */
    /* Disable digital function for DM DP */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Ana;
    PORT_Init(PortA, Pin11, &stcPortInit);
    PORT_Init(PortA, Pin12, &stcPortInit);
    PORT_SetFunc(PortA, Pin11, Func_UsbF, Disable);
    PORT_SetFunc(PortA, Pin12, Func_UsbF, Disable);
    PORT_SetFunc(PortB, Pin08, Func_UsbF, Disable);

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USBFS, Enable);
}

/**
 *******************************************************************************
 ** \brief  Enabele USB Global interrupt
 ** \param  None
 ** \retval None
 ******************************************************************************/
void usb_bsp_nvic(void)
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
 ** \brief  Drives the Vbus signal through IO
 ** \param  speed : Full, Low
 ** \param  state : VBUS states
 ** \retval None
 ******************************************************************************/
void usb_bsp_drivevbus(usb_core_instance *pdev,uint8_t state)
{
  USB_CORE_HPRT_TypeDef  hprt;

  hprt.d32 = hd_usb_rdhprt(pdev);
  hprt.b.prtlnsts = state;
  HD_USB_WRREG32(pdev->regs.HPRT0, hprt.d32);
}

/**
 *******************************************************************************
 ** \brief  Configures the IO for the Vbus and OverCurrent
 ** \param  speed : Full, Low
 ** \retval None
 ******************************************************************************/
void  hd_usb_bsp_cfgvbus(usb_core_instance *pdev)
{

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
