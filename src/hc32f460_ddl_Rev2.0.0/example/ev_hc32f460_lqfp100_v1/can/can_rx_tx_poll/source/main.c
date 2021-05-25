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
 ** \brief can rx tx poll mode sample
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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
__IO uint32_t can_id, can_data[2]={0};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Main function of can rx tx poll mode project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;
    stc_can_txframe_t       stcTxFrame;
    stc_can_rxframe_t       stcRxFrame;

    uint8_t u8Idx = 0u;

    MEM_ZERO_STRUCT(stcCanInitCfg);
    MEM_ZERO_STRUCT(stcFilter);
    MEM_ZERO_STRUCT(stcTxFrame);
    MEM_ZERO_STRUCT(stcRxFrame);

    //<<System clk initial
    BSP_CLK_Init();

    //<<Enable can peripheral clock and buffer(ram)
    PWC_RamOpMdConfig(HighSpeedMd);
    PWC_RamPwrdownCmd(PWC_RAMPWRDOWN_CAN, Enable);
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);

    //<<CAN GPIO config
    PORT_SetFunc(PortB, Pin06, Func_Can1_Rx, Disable);
    PORT_SetFunc(PortB, Pin07, Func_Can1_Tx, Disable);
    PORT_ResetBits(PortD, Pin15);
    PORT_OE(PortD, Pin15, Enable);

    //<<Can bit time config
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

    CAN_Init(&stcCanInitCfg);

    //<<Can filter config
    stcFilter.enAcfFormat = CanAllFrames;
    stcFilter.enFilterSel = CanFilterSel1;
    stcFilter.u32CODE     = 0x00000000ul;
    stcFilter.u32MASK     = 0x1FFFFFFFul;
    CAN_FilterConfig(&stcFilter, Enable);

    CAN_IrqCmd(CanRxIrqEn, Enable);

    while(1)
    {
        if(true == CAN_IrqFlgGet(CanRxIrqFlg))
        {
            CAN_IrqFlgClr(CanRxIrqFlg);

            CAN_Receive(&stcRxFrame);

            if(1u == stcRxFrame.Cst.Control_f.RTR)
            {
//                continue;
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

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
