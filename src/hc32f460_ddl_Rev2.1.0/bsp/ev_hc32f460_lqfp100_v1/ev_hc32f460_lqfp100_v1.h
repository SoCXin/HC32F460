/*****************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file ev_hc32f460_lqfp100_v1.h
 **
 ** A detailed description is available at
 ** @link EV_HC32F460_LQFP100_V1 EV_HC32F460_LQFP100_V1 description @endlink
 **
 **   - 2020-02-22  CDT  First version for Device Driver Library of BSP.
 **
 ******************************************************************************/
#ifndef __EV_HC32F460_LQFP100_V1_H__
#define __EV_HC32F460_LQFP100_V1_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_common.h"
#include "hc32f460_utility.h"
#include "hc32f460_clk.h"
#include "hc32f460_efm.h"
#include "hc32f460_gpio.h"
#include "hc32f460_exint_nmi_swi.h"
#include "hc32f460_interrupts.h"
#include "hc32f460_pwc.h"
#include "hc32f460_sram.h"
#include "hc32f460_keyscan.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup EV_HC32F460_LQFP100_V1
 * @{
 */

#if (BSP_EV_HC32F460_LQFP100_V1 == BSP_EV_HC32F460)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F460_LQFP100_V1_Global_Macros EV_HC32F460_LQFP100_V1 Global Macros
 * @{
 */

/** 
 * @defgroup EV_HC32F460_LQFP100_V1_KEY_Sel EV_HC32F460_LQFP100_V1 KEY definition
 * @{
 */
#define BSP_KEY_2               (0x0800UL)    /*!< BSP KEY 2. Independent key. */
#define BSP_KEY_3               (0x1000UL)    /*!< BSP KEY 3. Independent key. */
#define BSP_KEY_4               (0x2000UL)    /*!< BSP KEY 4. Independent key. */
#define BSP_KEY_5               (0x4000UL)    /*!< BSP KEY 5. Independent key. */
#define BSP_KEY_6               (0x0001UL)    /*!< BSP KEY 6. Matrix key. */
#define BSP_KEY_7               (0x0002UL)    /*!< BSP KEY 7. Matrix key. */
#define BSP_KEY_8               (0x0004UL)    /*!< BSP KEY 8. Matrix key. */
#define BSP_KEY_9               (0x0010UL)    /*!< BSP KEY 9. Matrix key. */
#define BSP_KEY_10              (0x0020UL)    /*!< BSP KEY 10. Matrix key. */
#define BSP_KEY_11              (0x0040UL)    /*!< BSP KEY 11. Matrix key. */
#define BSP_KEY_12              (0x0100UL)    /*!< BSP KEY 12. Matrix key. */
#define BSP_KEY_13              (0x0200UL)    /*!< BSP KEY 13. Matrix key. */
#define BSP_KEY_14              (0x0400UL)    /*!< BSP KEY 14. Matrix key. */
/**
 * @}
 */

/** 
 * @defgroup EV_HC32F460_LQFP100_V1_LED_Sel EV_HC32F460_LQFP100_V1 LED definition
 * @{
 */
#define LED_RED                 (0x01U)
#define LED_GREEN               (0x02U)
#define LED_YELLOW              (0x04U)
#define LED_BLUE                (0x08U)
/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V1_Local_Macros EV_HC32F460_LQFP100_V1 Local Macros
 * @{
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V1_LED_Number EV_HC32F460_LQFP100_V1 LED Number
 * @{
 */
#define BSP_LED_NUM             (4U)
/**
 * @}
 */

/** @defgroup EV_HC32F460_LQFP100_V1_LED_PortPin_Sel EV_HC32F460_LQFP100_V1 LED port/pin definition
 * @{
 */
#define BSP_LED_RED_PORT        (PortE)
#define BSP_LED_RED_PIN         (Pin06)
#define BSP_LED_GREEN_PORT      (PortA)
#define BSP_LED_GREEN_PIN       (Pin07)
#define BSP_LED_YELLOW_PORT     (PortB)
#define BSP_LED_YELLOW_PIN      (Pin05)
#define BSP_LED_BLUE_PORT       (PortB)
#define BSP_LED_BLUE_PIN        (Pin09)
/**
 * @}
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V1_KEY_Number EV_HC32F460_LQFP100_V1 KEY Number
 * @{
 */
#define BSP_KEY_ROW_NUM         (3U)
#define BSP_KEY_COL_NUM         (3U)
#define BSP_KEY_INDEPENDENT_NUM (4U)
/**
 * @}
 */

/** @defgroup EV_HC32F460_LQFP100_V1_KEY_PortPin EV_HC32F460_LQFP100_V1 KEY port/pin definition
 * @{
 */
#define BSP_KEY_KEY2_PORT       (PortD)
#define BSP_KEY_KEY2_PIN        (Pin03)
#define BSP_KEY_KEY2_EXINT      (ExtiCh03)
#define BSP_KEY_KEY2_INT_SRC    (INT_PORT_EIRQ3)
#define BSP_KEY_KEY2_IRQn       (Int025_IRQn)

#define BSP_KEY_KEY3_PORT       (PortD)
#define BSP_KEY_KEY3_PIN        (Pin05)
#define BSP_KEY_KEY3_EXINT      (ExtiCh05)
#define BSP_KEY_KEY3_INT_SRC    (INT_PORT_EIRQ5)
#define BSP_KEY_KEY3_IRQn       (Int027_IRQn)

#define BSP_KEY_KEY4_PORT       (PortD)
#define BSP_KEY_KEY4_PIN        (Pin04)
#define BSP_KEY_KEY4_EXINT      (ExtiCh04)
#define BSP_KEY_KEY4_INT_SRC    (INT_PORT_EIRQ4)
#define BSP_KEY_KEY4_IRQn       (Int026_IRQn)

#define BSP_KEY_KEY5_PORT       (PortD)
#define BSP_KEY_KEY5_PIN        (Pin06)
#define BSP_KEY_KEY5_EXINT      (ExtiCh06)
#define BSP_KEY_KEY5_INT_SRC    (INT_PORT_EIRQ6)
#define BSP_KEY_KEY5_IRQn       (Int028_IRQn)

#define BSP_KEYOUT0_PORT        (PortA)
#define BSP_KEYOUT0_PIN         (Pin04)
#define BSP_KEYOUT1_PORT        (PortA)
#define BSP_KEYOUT1_PIN         (Pin05)
#define BSP_KEYOUT2_PORT        (PortA)
#define BSP_KEYOUT2_PIN         (Pin06)

#define BSP_KEYIN0_PORT         (PortD)
#define BSP_KEYIN0_PIN          (Pin12)
#define BSP_KEY_ROW0_EXINT      (ExtiCh12)
#define BSP_KEY_ROW0_INT_SRC    (INT_PORT_EIRQ12)
#define BSP_KEY_ROW0_IRQn       (Int029_IRQn)

#define BSP_KEYIN1_PORT         (PortD)
#define BSP_KEYIN1_PIN          (Pin13)
#define BSP_KEY_ROW1_EXINT      (ExtiCh13)
#define BSP_KEY_ROW1_INT_SRC    (INT_PORT_EIRQ13)
#define BSP_KEY_ROW1_IRQn       (Int030_IRQn)

#define BSP_KEYIN2_PORT         (PortD)
#define BSP_KEYIN2_PIN          (Pin14)
#define BSP_KEY_ROW2_EXINT      (ExtiCh14)
#define BSP_KEY_ROW2_INT_SRC    (INT_PORT_EIRQ14)
#define BSP_KEY_ROW2_IRQn       (Int031_IRQn)
/**
 * @}
 */

/** @defgroup EV_HC32F460_LQFP100_V1_KEYSCAN_CONFIG EV_HC32F460_LQFP100_V1 KEYSCAN Configure definition
 * @{
 */
#define BSP_KEYOUT_SELECT       (Keyout0To2)
#define BSP_KEYIN_SELECT        (Keyin12 | Keyin13 | Keyin14)
/**
 * @}
 */

/** @defgroup EV_HC32F460_LQFP100_V1_PRINT_CONFIG EV_HC32F460_LQFP100_V1 PRINT Configure definition
 * @{
 */
#define BSP_PRINTF_DEVICE       (M4_USART3)

#define BSP_PRINTF_BAUDRATE     (115200)

#define BSP_PRINTF_PORT         (PortE)
#define BSP_PRINTF_PIN          (Pin05)
#define BSP_PRINTF_PORT_FUNC    (Func_Usart3_Tx)
/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup EV_HC32F460_LQFP100_V1_Global_Functions
 * @{
 */

void BSP_CLK_Init(void);

void BSP_KEY_Init(void);
en_flag_status_t BSP_KEY_GetStatus(uint32_t u32Key);

void BSP_LED_Init(void);
void BSP_LED_On(uint8_t u8Led);
void BSP_LED_Off(uint8_t u8Led);
void BSP_LED_Toggle(uint8_t u8Led);

void BSP_PRINTF_PortInit(void);

/* It cann't get the status of the KEYx by calling BSP_KEY_GetStatus when you reimplement BSP_KEY_KEYx_IrqHandler. */
void BSP_KEY_KEY2_IrqHandler(void);
void BSP_KEY_KEY3_IrqHandler(void);
void BSP_KEY_KEY4_IrqHandler(void);
void BSP_KEY_KEY5_IrqHandler(void);

/**
 * @}
 */

#endif /* BSP_EV_HC32F460_LQFP100_V1 */
/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __EV_HC32F460_LQFP100_V1_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
