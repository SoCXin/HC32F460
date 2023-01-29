/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file cdc_data_process.h
 **
 ** Header for usbd_cdc_vcp.c file.
 **
 **   - 2019-6-3  1.0  CDT First version for USB CDC VCP demo.
 **
 ******************************************************************************/
#ifndef __CDC_DATA_PROCESS_H__
#define __CDC_DATA_PROCESS_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "usb_dev_cdc_class.h"
#include "usb_dev_conf.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief all needed parameters to be configured for the ComPort.
 ** These parameters can modified on the fly by the host through CDC class
 ** command class requests.
 **
 ******************************************************************************/
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}LINE_CODING;

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define CDC_COMM                        (M4_USART3)
#define DEFAULT_CONFIG                  0u
#define OTHER_CONFIG                    1u


extern void vcp_init(void);
extern void vcp_deinit(void);
extern void vcp_ctrlpare(uint32_t Cmd, uint8_t* Buf, uint32_t Len);
extern void vcp_txdata(void);
extern void vcp_rxdata(uint8_t* Buf, uint32_t Len);


#ifdef __cplusplus
}
#endif

#endif /* __CDC_DATA_PROCESS_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
