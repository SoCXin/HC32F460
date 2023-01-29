/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file cdc_data_process.c
 **
 ** \brief  Generic media access Layer.
 **
 **   - 2019-11-19  1.0  CDT First version.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "cdc_data_process.h"
#include "usb_app_conf.h"
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
LINE_CODING linecoding =
  {
    115200, // baud rate - 115200
    0x00,   // stop bits - 1
    0x00,   // parity    - none
    0x08    // data bit  - 8
  };

extern uint8_t uart_rx_buffer[APP_RX_DATA_SIZE]; 
extern uint32_t APP_Rx_ptr_in;

void comport_config(void);  
void UsartErrIrqCallback(void);
void UsartRxIrqCallback(void);

/**
 *******************************************************************************
 ** \brief  Initializes the configuration of usart port
 ** \param  none
 ** \retval none
 ******************************************************************************/
void vcp_init(void)
{
    stc_usart_uart_init_t stcInitCfg;
    stc_irq_regi_conf_t stcIrqRegiCfg;

    MEM_ZERO_STRUCT(stcInitCfg);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);

    /* PC13 --> RX, PH02 --> TX for full-duplex */
    PORT_SetFunc(PortC, Pin13, Func_Usart3_Rx, Disable); //RX
    PORT_SetFunc(PortH, Pin02, Func_Usart3_Tx, Disable); //TX
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3, Enable);
    stcInitCfg.enClkMode        = UsartIntClkCkNoOutput;
    stcInitCfg.enClkDiv         = UsartClkDiv_1;
    stcInitCfg.enDataLength     = UsartDataBits8;
    stcInitCfg.enDirection      = UsartDataLsbFirst;
    stcInitCfg.enStopBit        = UsartOneStopBit;
    stcInitCfg.enParity         = UsartParityNone;
    stcInitCfg.enSampleMode     = UsartSampleBit16;
    stcInitCfg.enDetectMode     = UsartStartBitFallEdge;
    stcInitCfg.enHwFlow         = UsartRtsEnable;
    USART_UART_Init(CDC_COMM, &stcInitCfg);
    if(Ok == USART_SetBaudrate(CDC_COMM, 500000ul))
    {
        /* Set USART RX IRQ */
        stcIrqRegiCfg.enIRQn = Int000_IRQn;
        stcIrqRegiCfg.pfnCallback = &UsartRxIrqCallback;
        stcIrqRegiCfg.enIntSrc = INT_USART3_RI;
        enIrqRegistration(&stcIrqRegiCfg);
        NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_01);
        NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
        NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
        /* Set USART RX error IRQ */
        stcIrqRegiCfg.enIRQn = Int001_IRQn;
        stcIrqRegiCfg.pfnCallback = &UsartErrIrqCallback;
        stcIrqRegiCfg.enIntSrc = INT_USART3_EI;
        enIrqRegistration(&stcIrqRegiCfg);
        NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_01);
        NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
        NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

        USART_FuncCmd(CDC_COMM, UsartTx, Enable);
        USART_FuncCmd(CDC_COMM, UsartRx, Enable);
        USART_FuncCmd(CDC_COMM, UsartRxInt, Enable);
    }
}

/**
 *******************************************************************************
 ** \brief  deInitializes the Media
 ** \param  none
 ** \retval none
 ******************************************************************************/
void vcp_deinit(void)
{

}


/**
 *******************************************************************************
 ** \brief  Manage the CDC class requests
 ** \param  Cmd: Command code
 ** \param  Buf: data to be sent or received
 ** \param  Len: data length in bytes
 ** \retval status
 ******************************************************************************/
void vcp_ctrlpare(uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
    switch (Cmd)
    {
        case SEND_ENCAPSULATED_COMMAND:
        /* Not  needed for this driver */
        break;

        case GET_ENCAPSULATED_RESPONSE:
        /* Not  needed for this driver */
        break;

        case SET_COMM_FEATURE:
        /* Not  needed for this driver */
        break;

        case GET_COMM_FEATURE:
        /* Not  needed for this driver */
        break;

        case CLEAR_COMM_FEATURE:
        /* Not  needed for this driver */
        break;

        case SET_LINE_CODING:
            linecoding.bitrate = ((uint32_t)Buf[0] | ((uint32_t)Buf[1] << 8u) | ((uint32_t)Buf[2] << 16u) | ((uint32_t)Buf[3] << 24u));
            linecoding.format = Buf[4];
            linecoding.paritytype = Buf[5];
            linecoding.datatype = Buf[6];
            /* Set the new configuration */
            comport_config();  /* MISRAC 2004*/
        break;
        case GET_LINE_CODING:
            Buf[0] = (uint8_t)(linecoding.bitrate);
            Buf[1] = (uint8_t)(linecoding.bitrate >> 8u);
            Buf[2] = (uint8_t)(linecoding.bitrate >> 16u);
            Buf[3] = (uint8_t)(linecoding.bitrate >> 24u);
            Buf[4] = linecoding.format;
            Buf[5] = linecoding.paritytype;
            Buf[6] = linecoding.datatype;
        break;
        case SET_CONTROL_LINE_STATE:
        /* Not  needed for this driver */
        break;
        case SEND_BREAK:
        /* Not  needed for this driver */
        break;
        default:
        break;
    }
}

/**
 *******************************************************************************
 ** \brief  CDC received data to be send over USB IN endpoint are managed in
 **         this function.
 ** \param  none
 ** \retval none
 ******************************************************************************/
void vcp_txdata(void)
{
    if (linecoding.datatype == 7u)
    {
        uart_rx_buffer[APP_Rx_ptr_in] = (uint8_t)USART_RecData(CDC_COMM) & 0x7Fu;
    }
    else if (linecoding.datatype == 8u)
    {
        uart_rx_buffer[APP_Rx_ptr_in] = (uint8_t)USART_RecData(CDC_COMM);
    }
    else
    {
        //
    }

    APP_Rx_ptr_in++;

    /* To avoid buffer overflow */
    if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
    {
        APP_Rx_ptr_in = 0u;
    }
}

/**
 *******************************************************************************
 ** \brief  Data received from USB are sent to uart port
 ** \param  Buf: buffer to be sent to uart port
 ** \param  Len: data length in bytes
 ** \retval none
 ******************************************************************************/
void vcp_rxdata(uint8_t* Buf, uint32_t Len)
{
    uint32_t i;

    for (i = 0ul; i < Len; i++)
    {
        while(Set != USART_GetStatus(CDC_COMM, UsartTxEmpty))
        {
            ;
        }
        USART_SendData(CDC_COMM, (uint16_t)*(Buf + i));
    }
}

/**
 *******************************************************************************
 ** \brief  configurate the uart port
 ** \param  none
 ** \retval none
 ******************************************************************************/
void VCP_COMConfigDefault(void)
{
    stc_usart_uart_init_t stcInitCfg;
    uint8_t u8Cnt;
    MEM_ZERO_STRUCT(stcInitCfg);

    stcInitCfg.enClkMode        = UsartIntClkCkNoOutput;
    stcInitCfg.enClkDiv         = UsartClkDiv_1;
    stcInitCfg.enDataLength     = UsartDataBits8;
    stcInitCfg.enDirection      = UsartDataLsbFirst;
    stcInitCfg.enStopBit        = UsartOneStopBit;
    stcInitCfg.enParity         = UsartParityNone;
    stcInitCfg.enSampleMode     = UsartSampleBit16;
    stcInitCfg.enDetectMode     = UsartStartBitFallEdge;
    stcInitCfg.enHwFlow         = UsartRtsEnable;
    USART_UART_Init(CDC_COMM, &stcInitCfg);
    for (u8Cnt=0u; u8Cnt < 4u; u8Cnt++)
    {
        if(Ok == USART_SetBaudrate(CDC_COMM, 500000ul))
        {
            USART_FuncCmd(CDC_COMM, UsartTx, Enable);
            USART_FuncCmd(CDC_COMM, UsartRx, Enable);
            USART_FuncCmd(CDC_COMM, UsartRxInt, Enable);
            break;
        }
        else
        {
            USART_SetClockDiv(CDC_COMM, (en_usart_clk_div_t)u8Cnt);
        }
    }
    if (u8Cnt == 4u)
    {

    }
}

/**
 *******************************************************************************
 ** \brief  configurate the uart port
 ** \param  none
 ** \retval none
 ******************************************************************************/
void comport_config(void)
{
    stc_usart_uart_init_t stcInitCfg;
    uint8_t u8Cnt;
    MEM_ZERO_STRUCT(stcInitCfg);
    uint8_t u8ReturnFlag = 0u;

    /* set the Stop bit*/
    switch (linecoding.format)
    {
        case 0u:
            stcInitCfg.enStopBit = UsartOneStopBit;
            break;
        case 1u:
            stcInitCfg.enStopBit = UsartOneStopBit;
            break;
        case 2u:
            stcInitCfg.enStopBit = UsartTwoStopBit;
            break;
        default :
            VCP_COMConfigDefault();
            u8ReturnFlag = 1u;
            break;
    }

    if(1u != u8ReturnFlag)
    {
        /* set the parity bit*/
        switch (linecoding.paritytype)
        {
            case 0u:
                stcInitCfg.enParity = UsartParityNone;
                break;
            case 1u:
                stcInitCfg.enParity = UsartParityEven;
                break;
            case 2u:
                stcInitCfg.enParity = UsartParityOdd;
                break;
            default :
                VCP_COMConfigDefault();
                u8ReturnFlag = 1u;
                break;
        }

        if(1u != u8ReturnFlag)
        {
            /*set the data type : only 8bits and 9bits is supported */
            switch (linecoding.datatype)
            {
                case 0x07u:
                    /* With this configuration a parity (Even or Odd) should be set */
                    stcInitCfg.enDataLength = UsartDataBits8;
                    break;
                case 0x08u:
                    if (stcInitCfg.enParity == UsartParityNone)
                    {
                        stcInitCfg.enDataLength = UsartDataBits8;
                    }
                    else
                    {
                        stcInitCfg.enDataLength = UsartDataBits9;
                    }
                    break;
                default :
                    VCP_COMConfigDefault();
                    u8ReturnFlag = 1u;
                    break;
            }

            if(1u != u8ReturnFlag)
            {
                /* Configure and enable the USART */
                stcInitCfg.enHwFlow = UsartRtsEnable;
                USART_UART_Init(CDC_COMM, &stcInitCfg);

                for (u8Cnt=0u; u8Cnt < 4u; u8Cnt++)
                {
                    if(Ok == USART_SetBaudrate(CDC_COMM, 500000ul))
                    {
                        USART_FuncCmd(CDC_COMM, UsartTx, Enable);
                        USART_FuncCmd(CDC_COMM, UsartRx, Enable);
                        USART_FuncCmd(CDC_COMM, UsartRxInt, Enable);
                        break;
                    }
                    else
                    {
                        USART_SetClockDiv(CDC_COMM, (en_usart_clk_div_t)u8Cnt);
                    }
                }
                if (u8Cnt == 4u)
                {

                }
            }
        }
    }
}

/**
 *******************************************************************************
 ** \brief USART RX irq callback function.
 ** \param  none
 ** \retval none
 ******************************************************************************/
void UsartRxIrqCallback(void)
{
    if (Set == USART_GetStatus(CDC_COMM, UsartRxNoEmpty))
    {
        vcp_txdata( );
    }
}

/**
 *******************************************************************************
 ** \brief  USART RX error irq callback function.
 ** \param  none
 ** \retval none
 ******************************************************************************/
void UsartErrIrqCallback(void)
{
    if (Set == USART_GetStatus(CDC_COMM, UsartFrameErr))
    {
        USART_ClearStatus(CDC_COMM, UsartFrameErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(CDC_COMM, UsartParityErr))
    {
        USART_ClearStatus(CDC_COMM, UsartParityErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(CDC_COMM, UsartOverrunErr))
    {
        USART_ClearStatus(CDC_COMM, UsartOverrunErr);
    }
    else
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
