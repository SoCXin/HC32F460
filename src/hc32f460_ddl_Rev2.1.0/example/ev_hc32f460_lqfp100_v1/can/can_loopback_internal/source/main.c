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
 ** \brief can loop back internal mode sample
 **
 **   - 2018-12-12  CDT First version
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
static void CanTxRx(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Main function of can loop back internal mode project
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

    stcCanInitCfg.enCanRxBufAll  = CanRxAll;
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;
    stcCanInitCfg.enCanSAck      = CanSelfAckEnable;
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;

    stcCanInitCfg.pstcFilter     = astcFilters;
    stcCanInitCfg.u8FilterCount  = CAN_FILTERS_COUNT;

    CAN_Init(&stcCanInitCfg);

    //<< Loop back internal
    CAN_ModeConfig(CanExternalLoopBackMode, Enable);
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
    stc_can_rxframe_t stcRxFrame;
    uint8_t u8Idx = 0u;

    MEM_ZERO_STRUCT(stcTxFrame);
    MEM_ZERO_STRUCT(stcRxFrame);

    //<<Can Tx 1st
    stcTxFrame.StdID = 0x123ul;
    for(u8Idx = 0u; u8Idx < 0x08u; u8Idx++)
    {
        stcTxFrame.Data[u8Idx] = u8Idx;
    }

    stcTxFrame.Control_f.DLC = 0x08ul;
    stcTxFrame.Control_f.IDE = 0x00ul;
    CAN_SetFrame(&stcTxFrame);
    CAN_TransmitCmd(CanPTBTxCmd);
    //<< Wait transmitting done.
    while (false == CAN_IrqFlgGet(CanTxPrimaryIrqFlg));
    CAN_IrqFlgClr(CanTxPrimaryIrqFlg);

    while(1)
    {
        if(true == CAN_IrqFlgGet(CanRxIrqFlg))
        {
            CAN_IrqFlgClr(CanRxIrqFlg);
            //<< Rx
            CAN_Receive(&stcRxFrame);
            //<< Tx
            CAN_SetFrame(&stcTxFrame);
            CAN_TransmitCmd(CanPTBTxCmd);
            //<< Wait transmitting done.
            while (false == CAN_IrqFlgGet(CanTxPrimaryIrqFlg));
            CAN_IrqFlgClr(CanTxPrimaryIrqFlg);
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
