/******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file ev_hc32f460_lqfp100_v1.c
 **
 ** A detailed description is available at
 ** @link EV_HC32F460_LQFP100_V1 EV_HC32F460_LQFP100_V1 description @endlink
 **
 **   - 2020-02-22  CDT  First version for Device Driver Library of BSP.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "ev_hc32f460_lqfp100_v1.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V1 EV_HC32F460_LQFP100_V1
 * @{
 */

#if (BSP_EV_HC32F460_LQFP100_V1 == BSP_EV_HC32F460)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief BSP Port and pin configuration structure
 ******************************************************************************/
typedef struct
{
    en_port_t port;
    en_pin_t  pin;
} BSP_Port_Pin;

/**
 *******************************************************************************
 ** \brief BSP key in configuration structure
 ******************************************************************************/
typedef struct
{
    en_port_t    port;
    en_pin_t     pin;
    en_exti_ch_t ch;
    en_int_src_t int_src;
    IRQn_Type    irq;
    func_ptr_t   callback;
} BSP_KeyIn_Config;

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void BSP_KEY_ROW0_IrqCallback(void);
static void BSP_KEY_ROW1_IrqCallback(void);
static void BSP_KEY_ROW2_IrqCallback(void);
static void BSP_KEY_ROW_Init(void);
static void BSP_KEY_COL_Init(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const BSP_Port_Pin BSP_LED_PORT_PIN[BSP_LED_NUM] = {
    {BSP_LED_RED_PORT,      BSP_LED_RED_PIN},
    {BSP_LED_GREEN_PORT,    BSP_LED_GREEN_PIN},
    {BSP_LED_YELLOW_PORT,   BSP_LED_YELLOW_PIN},
    {BSP_LED_BLUE_PORT,     BSP_LED_BLUE_PIN}};

static const BSP_Port_Pin BSP_KEYOUT_PORT_PIN[BSP_KEY_COL_NUM] = {
    {BSP_KEYOUT0_PORT, BSP_KEYOUT0_PIN},
    {BSP_KEYOUT1_PORT, BSP_KEYOUT1_PIN},
    {BSP_KEYOUT2_PORT, BSP_KEYOUT2_PIN}};

static const BSP_KeyIn_Config BSP_KEYIN_PORT_PIN[BSP_KEY_ROW_NUM + BSP_KEY_INDEPENDENT_NUM] = {
    {BSP_KEYIN0_PORT, BSP_KEYIN0_PIN, BSP_KEY_ROW0_EXINT, BSP_KEY_ROW0_INT_SRC, BSP_KEY_ROW0_IRQn, BSP_KEY_ROW0_IrqCallback},
    {BSP_KEYIN1_PORT, BSP_KEYIN1_PIN, BSP_KEY_ROW1_EXINT, BSP_KEY_ROW1_INT_SRC, BSP_KEY_ROW1_IRQn, BSP_KEY_ROW1_IrqCallback},
    {BSP_KEYIN2_PORT, BSP_KEYIN2_PIN, BSP_KEY_ROW2_EXINT, BSP_KEY_ROW2_INT_SRC, BSP_KEY_ROW2_IRQn, BSP_KEY_ROW2_IrqCallback},

    {BSP_KEY_KEY2_PORT, BSP_KEY_KEY2_PIN, BSP_KEY_KEY2_EXINT, BSP_KEY_KEY2_INT_SRC, BSP_KEY_KEY2_IRQn, BSP_KEY_KEY2_IrqHandler},
    {BSP_KEY_KEY3_PORT, BSP_KEY_KEY3_PIN, BSP_KEY_KEY3_EXINT, BSP_KEY_KEY3_INT_SRC, BSP_KEY_KEY3_IRQn, BSP_KEY_KEY3_IrqHandler},
    {BSP_KEY_KEY4_PORT, BSP_KEY_KEY4_PIN, BSP_KEY_KEY4_EXINT, BSP_KEY_KEY4_INT_SRC, BSP_KEY_KEY4_IRQn, BSP_KEY_KEY4_IrqHandler},
    {BSP_KEY_KEY5_PORT, BSP_KEY_KEY5_PIN, BSP_KEY_KEY5_EXINT, BSP_KEY_KEY5_INT_SRC, BSP_KEY_KEY5_IRQn, BSP_KEY_KEY5_IrqHandler}};

static uint32_t gu32GlobalKey = 0UL;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F460_LQFP100_V1_Global_Functions EV_HC32F460_LQFP100_V1 Global Functions
 * @{
 */

/**
 * @brief  BSP clock initialize.
 *         Set board system clock to MPLL@200MHz
 * @param  None
 * @retval None
 */
void BSP_CLK_Init(void)
{
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;
    stc_sram_config_t       stcSramConfig;

    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);
    MEM_ZERO_STRUCT(stcSramConfig);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv  = ClkSysclkDiv1;
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Config Xtal and Enable Xtal */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* sram init include read/write wait cycle setting */
    stcSramConfig.u8SramIdx = Sram12Idx | Sram3Idx | SramHsIdx | SramRetIdx;
    stcSramConfig.enSramRC = SramCycle2;
    stcSramConfig.enSramWC = SramCycle2;
    SRAM_Init(&stcSramConfig);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* MPLL config (XTAL / pllmDiv * plln / PllpDiv = 200M). */
    stcMpllCfg.pllmDiv = 1ul;
    stcMpllCfg.plln    = 50ul;
    stcMpllCfg.PllpDiv = 2ul;
    stcMpllCfg.PllqDiv = 2ul;
    stcMpllCfg.PllrDiv = 2ul;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);
    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
        ;
    }
    /* Switch driver ability */
    PWC_HS2HP();
    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 * @brief  BSP key initialize
 * @param  None
 * @retval None
 */
void BSP_KEY_Init(void)
{
    uint8_t i;

    BSP_KEY_ROW_Init();
    BSP_KEY_COL_Init();
    /* Clear all KEYIN interrupt flag before enable */
    for (i = 0U; i < BSP_KEY_ROW_NUM+BSP_KEY_INDEPENDENT_NUM; i++)
    {
        EXINT_IrqFlgClr(BSP_KEYIN_PORT_PIN[i].ch);
    }
    KEYSCAN_Start();
}

/**
 * @brief  Get BSP key status
 * @param  [in] u32Key chose one macro from below
 *   @arg  BSP_KEY_2
 *   @arg  BSP_KEY_3
 *   @arg  BSP_KEY_4
 *   @arg  BSP_KEY_5
 *   @arg  BSP_KEY_6
 *   @arg  BSP_KEY_7
 *   @arg  BSP_KEY_8
 *   @arg  BSP_KEY_9
 *   @arg  BSP_KEY_10
 *   @arg  BSP_KEY_11
 *   @arg  BSP_KEY_12
 *   @arg  BSP_KEY_13
 *   @arg  BSP_KEY_14
 * @retval en_flag_status_t
 *   @arg  Set, Key pressed.
 *   @arg  Reset, Key released.
 */
en_flag_status_t BSP_KEY_GetStatus(uint32_t u32Key)
{
    en_flag_status_t enRet = Reset;

    if (0UL != (gu32GlobalKey & u32Key))
    {
        enRet = Set;
        gu32GlobalKey &= ~u32Key;
    }
    else
    {
    }
    return enRet;
}

/**
 * @brief  LED initialize.
 * @param  None
 * @retval none
 */
void BSP_LED_Init(void)
{
    uint8_t i;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    /* Initialize LED pin */
    for (i = 0U; i < BSP_LED_NUM; i++)
    {
        PORT_Init(BSP_LED_PORT_PIN[i].port, BSP_LED_PORT_PIN[i].pin, &stcPortInit);
    }
}

/**
 * @brief  Turn on LEDs.
 * @param  [in] u8Led LED
 *   @arg  LED_RED
 *   @arg  LED_GREEN
 *   @arg  LED_YELLOW
 *   @arg  LED_BLUE
 * @retval none
 */
void BSP_LED_On(uint8_t u8Led)
{
    uint8_t i;

    for (i = 0U; i < BSP_LED_NUM; i++)
    {
        if (0U != ((u8Led >> i) & 1U))
        {
            PORT_SetBits(BSP_LED_PORT_PIN[i].port, BSP_LED_PORT_PIN[i].pin);
        }
    }
}

/**
 * @brief  Turn off LEDs.
 * @param  [in] u8Led LED
 *   @arg  LED_RED
 *   @arg  LED_GREEN
 *   @arg  LED_YELLOW
 *   @arg  LED_BLUE
 * @retval none
 */
void BSP_LED_Off(uint8_t u8Led)
{
    uint8_t i;

    for (i = 0U; i < BSP_LED_NUM; i++)
    {
        if (0U != ((u8Led >> i) & 1U))
        {
            PORT_ResetBits(BSP_LED_PORT_PIN[i].port, BSP_LED_PORT_PIN[i].pin);
        }
    }
}

/**
 * @brief  Toggle LEDs.
 * @param  [in] u8Led LED
 *   @arg  LED_RED
 *   @arg  LED_GREEN
 *   @arg  LED_YELLOW
 *   @arg  LED_BLUE
 * @retval none
 */
void BSP_LED_Toggle(uint8_t u8Led)
{
    uint8_t i;

    for (i = 0U; i < BSP_LED_NUM; i++)
    {
        if (0U != ((u8Led >> i) & 1U))
        {
            PORT_Toggle(BSP_LED_PORT_PIN[i].port, BSP_LED_PORT_PIN[i].pin);
        }
    }
}

/**
 * @brief  BSP printf port initialize
 * @param  None
 * @retval None
 */
void BSP_PRINTF_PortInit(void)
{
    PORT_SetFunc(BSP_PRINTF_PORT, BSP_PRINTF_PIN, BSP_PRINTF_PORT_FUNC, Disable);
}

/**
 * @}
 */

/**
 * @brief  BSP Key2 callback function
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_KEY_KEY2_IrqHandler(void)
{
    gu32GlobalKey |= BSP_KEY_2;
    while (Reset == PORT_GetBit(BSP_KEY_KEY2_PORT, BSP_KEY_KEY2_PIN));
    EXINT_IrqFlgClr(BSP_KEY_KEY2_EXINT);
}

/**
 * @brief  BSP Key3 callback function
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_KEY_KEY3_IrqHandler(void)
{
    gu32GlobalKey |= BSP_KEY_3;
    while (Reset == PORT_GetBit(BSP_KEY_KEY3_PORT, BSP_KEY_KEY3_PIN));
    EXINT_IrqFlgClr(BSP_KEY_KEY3_EXINT);
}

/**
 * @brief  BSP Key4 callback function
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_KEY_KEY4_IrqHandler(void)
{
    gu32GlobalKey |= BSP_KEY_4;
    while (Reset == PORT_GetBit(BSP_KEY_KEY4_PORT, BSP_KEY_KEY4_PIN));
    EXINT_IrqFlgClr(BSP_KEY_KEY4_EXINT);
}

/**
 * @brief  BSP Key5 callback function
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_KEY_KEY5_IrqHandler(void)
{
    gu32GlobalKey |= BSP_KEY_5;
    while (Reset == PORT_GetBit(BSP_KEY_KEY5_PORT, BSP_KEY_KEY5_PIN));
    EXINT_IrqFlgClr(BSP_KEY_KEY5_EXINT);
}

/**
 * @defgroup BSP_Local_Functions BSP Local Functions
 * @{
 */

/**
 * @brief  BSP Key row 0 callback function
 * @param  None
 * @retval None
 */
static void BSP_KEY_ROW0_IrqCallback(void)
{
    uint8_t u8Idx = KEYSCAN_GetColIdx();
    if (Set == EXINT_IrqFlgGet(BSP_KEYIN_PORT_PIN[0].ch))
    {
        for (;;)
        {
            if (Reset == PORT_GetBit(BSP_KEYIN_PORT_PIN[0].port, BSP_KEYIN_PORT_PIN[0].pin))
            {
                gu32GlobalKey |= (0x01UL) << u8Idx;
            }
            else
            {
                /* clear int request flag  after KEY released */
                EXINT_IrqFlgClr(BSP_KEYIN_PORT_PIN[0].ch);
                break;
            }
        }
    }
}

/**
 * @brief  BSP Key row 1 callback function
 * @param  None
 * @retval None
 */
static void BSP_KEY_ROW1_IrqCallback(void)
{
    uint8_t u8Idx = KEYSCAN_GetColIdx();
    if (Set == EXINT_IrqFlgGet(BSP_KEYIN_PORT_PIN[1].ch))
    {
        for (;;)
        {
            if (Reset == PORT_GetBit(BSP_KEYIN_PORT_PIN[1].port, BSP_KEYIN_PORT_PIN[1].pin))
            {
                gu32GlobalKey |= (0x10UL) << u8Idx;
            }
            else
            {
                /* clear int request flag after KEY released */
                EXINT_IrqFlgClr(BSP_KEYIN_PORT_PIN[1].ch);
                break;
            }
        }
    }
}

/**
 * @brief  BSP Key row 2 callback function
 * @param  None
 * @retval None
 */
static void BSP_KEY_ROW2_IrqCallback(void)
{
    uint8_t u8Idx = KEYSCAN_GetColIdx();
    if (Set == EXINT_IrqFlgGet(BSP_KEYIN_PORT_PIN[2].ch))
    {
        for (;;)
        {
            if (Reset == PORT_GetBit(BSP_KEYIN_PORT_PIN[2].port, BSP_KEYIN_PORT_PIN[2].pin))
            {
                gu32GlobalKey |= (0x100UL) << u8Idx;
            }
            else
            {
                /* clear int request flag after KEY released */
                EXINT_IrqFlgClr(BSP_KEYIN_PORT_PIN[2].ch);
                break;
            }
        }
    }
}

/**
 * @brief  BSP key row initialize
 * @param  None
 * @retval None
 */
static void BSP_KEY_ROW_Init(void)
{
    uint8_t i;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /* GPIO config */
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    for (i = 0U; i < BSP_KEY_ROW_NUM+BSP_KEY_INDEPENDENT_NUM; i++)
    {
        PORT_Init(BSP_KEYIN_PORT_PIN[i].port, BSP_KEYIN_PORT_PIN[i].pin, &stcPortInit);
    }

    /* Exint config */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk   = Pclk3Div8;
    stcExtiConfig.enExtiLvl  = ExIntFallingEdge;
    for (i = 0U; i < BSP_KEY_ROW_NUM+BSP_KEY_INDEPENDENT_NUM; i++)
    {
        stcExtiConfig.enExitCh = BSP_KEYIN_PORT_PIN[i].ch;
        EXINT_Init(&stcExtiConfig);
    }

    /* IRQ sign-in */
    for (i = 0U; i < BSP_KEY_ROW_NUM+BSP_KEY_INDEPENDENT_NUM; i++)
    {
        stcIrqRegiConf.enIntSrc = BSP_KEYIN_PORT_PIN[i].int_src;
        stcIrqRegiConf.enIRQn   = BSP_KEYIN_PORT_PIN[i].irq;
        stcIrqRegiConf.pfnCallback = BSP_KEYIN_PORT_PIN[i].callback;
        enIrqRegistration(&stcIrqRegiConf);

        /* NVIC config */
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    }
}

/**
 * @brief  BSP key column initialize
 * @param  None
 * @retval None
 */
static void BSP_KEY_COL_Init(void)
{
    uint8_t i;
    stc_port_init_t stcPortInit;
    stc_keyscan_config_t stcKeyscanConfig;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcKeyscanConfig);

    /* Set corresponding pins to KEYSCAN function */
    for (i = 0U; i < BSP_KEY_COL_NUM; i++)
    {
        PORT_Init(BSP_KEYOUT_PORT_PIN[i].port, BSP_KEYOUT_PORT_PIN[i].pin , &stcPortInit);
        PORT_SetFunc(BSP_KEYOUT_PORT_PIN[i].port, BSP_KEYOUT_PORT_PIN[i].pin, Func_Key, Disable);
    }

    /* enable KEYSCAN module source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_KEY, Enable);
    CLK_LrcCmd(Enable);

    /* configuration KEYSCAN */
    stcKeyscanConfig.enHizCycle   = Hiz4;
    stcKeyscanConfig.enLowCycle   = Low512;
    stcKeyscanConfig.enKeyscanClk = KeyscanLrc;
    stcKeyscanConfig.enKeyoutSel  = BSP_KEYOUT_SELECT;
    stcKeyscanConfig.u16KeyinSel  = BSP_KEYIN_SELECT;
    KEYSCAN_Init(&stcKeyscanConfig);
}

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

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
