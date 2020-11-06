/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file usb_bsp_template.c
 **
 ** \brief  This file is responsible to offer board support package and is
 **         configurable by user.
 **
 **   - 2018-12-25  1.0  Wangmin First version for USB mouse demo.
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

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
extern  USB_OTG_CORE_HANDLE      USB_OTG_dev;
extern  uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);


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
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock for the sample
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
static void SysClkIni(void)
{
    en_clk_sys_source_t     enSysClkSrc;
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;
//    stc_clk_output_cfg_t    stcOutputClkCfg;
    stc_clk_upll_cfg_t      stcUpllCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal32 as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln =42;
    stcMpllCfg.PllpDiv = 2;     //MPLLP = 84
    stcMpllCfg.PllqDiv = 2;
    stcMpllCfg.PllrDiv = 7;    //MPLLR 168/4 = 48
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);
    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);

    stcUpllCfg.pllmDiv = 1;
    stcUpllCfg.plln = 42;
    stcUpllCfg.PllpDiv = 7;//48M
    stcUpllCfg.PllqDiv = 7;
    stcUpllCfg.PllrDiv = 7;
    CLK_UpllConfig(&stcUpllCfg);
    CLK_UpllCmd(Enable);
    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagUPLLRdy));

    /* Set USB clock source */
    CLK_SetUsbClkSource(ClkUsbSrcUpllp);

#if 0
    /* Clk output.*/
    stcOutputClkCfg.enOutputSrc = ClkOutputSrcUpllp;
    stcOutputClkCfg.enOutputDiv = ClkOutputDiv8;
    CLK_OutputClkConfig(ClkOutputCh1,&stcOutputClkCfg);
    CLK_OutputClkCmd(ClkOutputCh1,Enable);

    PORT_SetFunc(PortA, Pin08, Func_Mclkout, Disable);
#endif
}

/**
 ******************************************************************************
  * \brief  USB_OTG_BSP_Init
  *         Initilizes BSP configurations
  *
  * \param  None
  * \retval None
 ******************************************************************************/
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
    /* clock config */
//    SysClkIni();

	Ddl_UartInit();
    printf("USBFS start !!\n");

    /* port config */
    PORT_SetFunc(PortA, Pin08, Func_UsbF, Disable); //SOF
    PORT_SetFunc(PortA, Pin09, Func_UsbF, Disable); //VBUS
    PORT_SetFunc(PortA, Pin10, Func_UsbF, Disable); //ID
    PORT_SetFunc(PortA, Pin11, Func_UsbF, Disable); //DM
    PORT_SetFunc(PortA, Pin12, Func_UsbF, Disable); //DP
    PORT_SetFunc(PortB, Pin08, Func_UsbF, Disable); //DRVVBUS

    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USBFS, Enable);
}

/**
  * \brief  USB_OTG_BSP_EnableInterrupt
  *         Enabele USB Global interrupt
  * \param  None
  * \retval None
  */
void USB_OTG_BSP_EnableInterrupt(void)
{
    stc_irq_regi_conf_t stcIrqRegiConf;
    /* Register INT_USBFS_GLB Int to Vect.No.030 */
    stcIrqRegiConf.enIRQn = Int030_IRQn;
    /* Select INT_USBFS_GLB interrupt function */
    stcIrqRegiConf.enIntSrc = INT_USBFS_GLB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = USB_IRQ_Handler;
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
  * \brief  BSP_Drive_VBUS
  *         Drives the Vbus signal through IO
  * \param  speed : Full, Low
  * \param  state : VBUS states
  * \retval None
  */

void USB_OTG_BSP_DriveVBUS(uint32_t speed, uint8_t state)
{
#if 0
	/* Vbus control */
	M4_PORT->PWPR = 0xA501;
#if (USB_OTG_FS_BASE_ADDR == 0x400c0000) && defined USE_USB_OTG_FS
	M4_PORT->PFSR006 = 0x0000;
	M4_PORT->PCR006 = 0x0003;
#else
	M4_PORT->PFSR004 = 0x0000;
	M4_PORT->PCR004 = 0x0003;
#endif
#endif
}

/**
  * \brief  USB_OTG_BSP_ConfigVBUS
  *         Configures the IO for the Vbus and OverCurrent
  * \param  Speed : Full, Low
  * \retval None
  */

void  USB_OTG_BSP_ConfigVBUS(uint32_t speed)
{

}

/**
  * \brief  USB_OTG_BSP_TimeInit
  *         Initialises delay unit Systick timer /Timer2
  * \param  None
  * \retval None
  */
void USB_OTG_BSP_TimeInit ( void )
{

}

/**
  * \brief  USB_OTG_BSP_uDelay
  *         This function provides delay time in micro sec
  * \param  usec : Value of delay required in micro sec
  * \retval None
  */
#define	Fclk	15000000
void USB_OTG_BSP_uDelay (const uint32_t t)
{
	uint32_t	i;
	uint32_t	j;
	j=Fclk / 1000000 * t;
	for(i = 0; i < j; i++);
}


/**
  * \brief  USB_OTG_BSP_mDelay
  *          This function provides delay time in milli sec
  * \param  msec : Value of delay required in milli sec
  * \retval None
  */
void USB_OTG_BSP_mDelay (const uint32_t msec)
{
    USB_OTG_BSP_uDelay(msec * 1000);
}


/**
  * \brief  USB_OTG_BSP_TimerIRQ
  *         Time base IRQ
  * \param  None
  * \retval None
  */

void USB_OTG_BSP_TimerIRQ (void)
{

}
