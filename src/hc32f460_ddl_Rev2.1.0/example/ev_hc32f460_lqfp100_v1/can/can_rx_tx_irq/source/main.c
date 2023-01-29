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
 ** \brief can tx rx Irq mode sample
 **
 **   - 2018-12-22  CDT First version
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
#define CAN_FILTERS_COUNT           (1u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void CanInitConfig(void);
static void CanIrqConfig(void);
static void CanTxRx(void);

static void CAN_RxIrqCallBack(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_can_rxframe_t stcRxFrame;
__IO static uint8_t u8RxFlag  = false;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function of can tx rx Irq mode project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    //<< System clk initial
    BSP_CLK_Init();
    //<< CAN initial
    CanInitConfig();
    //<< CAN interrupt config
    CanIrqConfig();
    //<< CAN TX and RX.
    CanTxRx();

    return 0;
}

/**
 *******************************************************************************
 ** \brief  CAN configuration.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void CanInitConfig(void)
{
    stc_can_init_config_t stcCanInitCfg;
    stc_can_filter_t astcFilters[CAN_FILTERS_COUNT] = \
    {
        {0x00000000ul, 0x1FFFFFFFul, CanFilterSel1, CanAllFrames}
    };

    //<< Enable can peripheral clock and buffer(ram)
    PWC_RamOpMdConfig(HighSpeedMd);
    PWC_RamPwrdownCmd(PWC_RAMPWRDOWN_CAN, Enable);
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);

    //<< CAN GPIO config
    PORT_SetFunc(PortB, Pin06, Func_Can1_Rx, Disable);
    PORT_SetFunc(PortB, Pin07, Func_Can1_Tx, Disable);
    PORT_ResetBits(PortD, Pin15);
    PORT_OE(PortD, Pin15, Enable);

    MEM_ZERO_STRUCT(stcCanInitCfg);
    //<< Can bit time config
    stcCanInitCfg.stcCanBt.PRESC = 1u-1u;
    stcCanInitCfg.stcCanBt.SEG_1 = 5u-2u;
    stcCanInitCfg.stcCanBt.SEG_2 = 3u-1u;
    stcCanInitCfg.stcCanBt.SJW   = 3u-1u;

    stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 10u;
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 16u-1u;

    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;
    stcCanInitCfg.enCanSAck      = CanSelfAckEnable;
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;

    stcCanInitCfg.pstcFilter     = astcFilters;
    stcCanInitCfg.u8FilterCount  = CAN_FILTERS_COUNT;

    CAN_Init(&stcCanInitCfg);
}

/**
 *******************************************************************************
 ** \brief  CAN TX and RX.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void CanTxRx(void)
{
    stc_can_txframe_t stcTxFrame;
    uint8_t u8Idx = 0u;

    MEM_ZERO_STRUCT(stcTxFrame);

    while(1)
    {
        if(true == u8RxFlag)
        {
            u8RxFlag = false;

            if(1u == stcRxFrame.Cst.Control_f.RTR)
            {
                // continue;
            }
            else
            {
                //<<Can Tx
                stcTxFrame.StdID         = 0x123ul;
                stcTxFrame.Control_f.DLC = stcRxFrame.Cst.Control_f.DLC;
                stcTxFrame.Control_f.IDE = stcRxFrame.Cst.Control_f.IDE;

                for(u8Idx=0u; u8Idx<stcRxFrame.Cst.Control_f.DLC; u8Idx++)
                {
                    stcTxFrame.Data[u8Idx] = stcRxFrame.Data[u8Idx];
                }

                CAN_SetFrame(&stcTxFrame);
                CAN_TransmitCmd(CanPTBTxCmd);
            }
        }
    }
}

/**
 *******************************************************************************
 ** \brief  CAN interrupt configuration.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void CanIrqConfig(void)
{
    stc_irq_regi_conf_t     stcIrqRegiConf;

    stcIrqRegiConf.enIRQn = Int000_IRQn;
    stcIrqRegiConf.enIntSrc = INT_CAN_INT;
    stcIrqRegiConf.pfnCallback = &CAN_RxIrqCallBack;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief  CAN interrupt callback function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void CAN_RxIrqCallBack(void)
{
    if(true == CAN_IrqFlgGet(CanRxIrqFlg))
    {
        CAN_IrqFlgClr(CanRxIrqFlg);
        CAN_IrqCmd(CanRxIrqEn, Disable);

        CAN_Receive(&stcRxFrame);

        u8RxFlag = true;
    }

    if(true == CAN_IrqFlgGet(CanTxPrimaryIrqFlg))
    {
        CAN_IrqFlgClr(CanTxPrimaryIrqFlg);

        CAN_IrqCmd(CanRxIrqEn, Enable);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
