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
 ** \brief Example project for i2s full duplex
 **
 **   - 2018-11-01  CDT First version for Device Driver Library example
 **     for I2S
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "wm8731.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Define if need play by speaker*/
#define SPEAKER_ON                  (true)
/* Define if use exclk */
#define EXCK_ON                     (true)
/* Select Record source */
#define RECORD_MIC                  (true)

/* Define I2C unit used for the example */
#define I2C_CH                      (M4_I2C2)
/* Define port and pin for SDA and SCL */
#define I2C2_SCL_PORT               (PortD)
#define I2C2_SCL_PIN                (Pin00)
#define I2C2_SDA_PORT               (PortD)
#define I2C2_SDA_PIN                (Pin01)

/* Define I2S unit used for the example */
#define I2S_CH                      (M4_I2S3)
#define I2S_ERR_IRQ                 (INT_I2S3_ERRIRQOUT)

#define I2S3_WS_PORT                (PortB)
#define I2S3_WS_PIN                 (Pin13)
#define I2S3_SD_PORT                (PortB)
#define I2S3_SD_PIN                 (Pin14)
#define I2S3_SD_IN_PORT             (PortB)
#define I2S3_SD_IN_PIN              (Pin15)
#define I2S3_CK_PORT                (PortB)
#define I2S3_CK_PIN                 (Pin12)
/* Define port and pin for i2s1 function */
#if(EXCK_ON)
/* if exck enable*/
#define I2S3_EXCK_PORT              (PortB)
#define I2S3_EXCK_PIN               (Pin10)
#else
/* if exck disable */
#define I2S3_MCK_PORT               (PortB)
#define I2S3_MCK_PIN                (Pin12)
#endif

#define SPK_EN_PORT                 (PortB)
#define SPK_EN_PIN                  (Pin00)
#define SPEAKER_EN()                (PORT_SetBits(SPK_EN_PORT, SPK_EN_PIN))
#define SPEAKER_DISEN()             (PORT_ResetBits(SPK_EN_PORT, SPK_EN_PIN))

/* DMA for Play */
#define DMA_UNIT_PLAY               (M4_DMA2)
#define DMA_CH_PLAY                 (DmaCh0)
#define DMA_PLAY_FCG                (PWC_FCG0_PERIPH_DMA2)
#define DMA_PLAY_IRQ_SRC            (INT_DMA2_TC0)

/* DMA for Record */
#define DMA_UNIT_RECORD             (M4_DMA1)
#define DMA_CH_RECORD               (DmaCh1)
#define DMA_RECORD_FCG              (PWC_FCG0_PERIPH_DMA1)
#define DMA_RECORD_IRQ_SRC          (INT_DMA1_TC1)

#define PACKET                      (96U)
#define NUM                         (8U)
#define TOTAL_IN_BUF_SIZE           (PACKET*NUM)

#define IDLE_STATE                  (0U)
#define RECORD_STATE                (1U)
#define ALL_RUN_STATE               (2U)
#define STOP_STATE                  (3U)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void Record_Repeat(uint32_t buf, uint16_t size);
static void Record_Start(uint32_t buf, uint16_t size);
static void DMA_TC_PlayCallBack(void);
static void DMA_TC_RecordCallback(void);
static void Play_Repeat(uint32_t Addr, uint32_t Size);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t u8Play_tx_flag = 0U;
uint16_t IsocInBuff[TOTAL_IN_BUF_SIZE];

uint16_t* IsocInWrPtr;
uint16_t IsocInWrCnt;

uint16_t IsocInRdCnt = 0U;
uint8_t work_state = IDLE_STATE;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief  DMA transfer complete irq call back function for play
 **
 ******************************************************************************/
static void DMA_TC_PlayCallBack(void)
{
    DMA_ClearIrqFlag(DMA_UNIT_PLAY, DMA_CH_PLAY, BlkTrnCpltIrq);
    DMA_ClearIrqFlag(DMA_UNIT_PLAY, DMA_CH_PLAY, TrnCpltIrq);
    u8Play_tx_flag = 1U;
}

/**
 ******************************************************************************
 ** \brief  DMA transfer complete irq call back function for record
 **
 ******************************************************************************/
static void DMA_TC_RecordCallback(void)
{
    DMA_ClearIrqFlag(DMA_UNIT_RECORD, DMA_CH_RECORD, TrnCpltIrq);

    Record_Repeat((uint32_t)(IsocInBuff+PACKET*IsocInRdCnt), PACKET);

    IsocInRdCnt++;
    if(IsocInRdCnt>=NUM)
    {
        IsocInRdCnt = 0U;
    }
}

/**
 ******************************************************************************
 ** \brief  reset the DTCTL and enable the DMA channel
 **
 ******************************************************************************/
static void Record_Repeat(uint32_t buf, uint16_t size)
{
    /* note: Don't use driver function for speed */
    DMA_UNIT_RECORD->DAR1 = buf;             //Rewrite the destination address
    DMA_UNIT_RECORD->DTCTL1_f.CNT = size;    //Rewrite the count
    DMA_UNIT_RECORD->DTCTL1_f.BLKSIZE = 1U;  //Rewrite the block size

    DMA_ChannelCmd(DMA_UNIT_RECORD, DMA_CH_RECORD, Enable);
}

/**
 ******************************************************************************
 ** \brief  Start record
 **
 ******************************************************************************/
static void Record_Start(uint32_t buf, uint16_t size)
{
    /* note: Don't use driver function for speed */
    DMA_UNIT_RECORD->DAR1 = buf;              //Rewrite the destination address
    DMA_UNIT_RECORD->DTCTL1_f.CNT = size;     //Rewrite the count
    DMA_UNIT_RECORD->DTCTL1_f.BLKSIZE = 1U;   //Rewrite the block size
    DMA_ChannelCmd(DMA_UNIT_RECORD, DMA_CH_RECORD, Enable);

    /* note: Don't use driver function for speed */
    I2S_CH->ER_f.RXERR = 1U;         //Clear the RX ERROR
    I2S_CH->CTRL_f.RXE = 1U;         //Enable the RX of I2S
}

/**
 ******************************************************************************
 ** \brief  Stop record
 **
 ******************************************************************************/
void Record_Stop(void)
{
    DMA_ChannelCmd(DMA_UNIT_RECORD, DMA_CH_RECORD, Disable);
    I2S_FuncCmd(I2S_CH, RxEn, Disable);
    I2S_ClrErrFlag(I2S_CH, ClrRxErrFlag);

    DMA_ChannelCmd(DMA_UNIT_PLAY, DMA_CH_PLAY, Disable);
    I2S_FuncCmd(I2S_CH, TxEn, Disable);
    I2S_ClrErrFlag(I2S_CH, ClrRxErrFlag);
}

/**
 ******************************************************************************
 ** \brief  Starts playing audio stream from the audio Media.
 ** \param  [in] Addr                   The storage address of audio data.
 ** \param  [in] Size                   The size  of audio data.
 ** \retval None
 **
 ******************************************************************************/
static void Play_Repeat(uint32_t Addr, uint32_t Size)
{
    /* note: Don't use driver function for speed */
    DMA_UNIT_PLAY->DTCTL0_f.BLKSIZE = 1U;
    DMA_UNIT_PLAY->DTCTL0_f.CNT     = Size;
    DMA_UNIT_PLAY->SAR0             = (uint32_t)Addr;

    DMA_ChannelCmd(DMA_UNIT_PLAY, DMA_CH_PLAY, Enable);

    /* note: Don't use driver function for speed */
    if(I2S_CH->ER_f.TXERR == 1U)               //Clear the tx error if it is true
    {
        I2S_CH->ER_f.TXERR = 1U;
    }
    I2S_CH->CTRL_f.TXE = 1U;                   //Enable the TX of I2S
}

/**
 ******************************************************************************
 ** \brief  configure the GPIO of I2S3 and I2C2
 **
 ******************************************************************************/
void WM8731_IO_Ini(void)
{
    stc_port_init_t stcPortIni;
    /* Enable I2S Peripheral*/
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_I2S3, Enable);
    /* Initialize i2s port for codec wm8731 recorder function */
    MEM_ZERO_STRUCT(stcPortIni);
    stcPortIni.enPullUp = Enable;
    stcPortIni.enPinDrv = Pin_Drv_H;
    PORT_Init(I2S3_CK_PORT, I2S3_CK_PIN, &stcPortIni);
    PORT_Init(I2S3_WS_PORT, I2S3_WS_PIN, &stcPortIni);
    PORT_Init(I2S3_SD_PORT, I2S3_SD_PIN, &stcPortIni);
    PORT_SetFunc(I2S3_CK_PORT, I2S3_CK_PIN, Func_I2s3_Ck, Disable);
    PORT_SetFunc(I2S3_WS_PORT, I2S3_WS_PIN, Func_I2s3_Ws, Disable);
    PORT_SetFunc(I2S3_SD_IN_PORT, I2S3_SD_IN_PIN, Func_I2s3_Sdin, Disable);
    PORT_SetFunc(I2S3_SD_PORT, I2S3_SD_PIN, Func_I2s3_Sd, Disable);
#ifdef EXCK_ON
    PORT_Init(I2S3_EXCK_PORT, I2S3_EXCK_PIN, &stcPortIni);
    PORT_SetFunc(I2S3_EXCK_PORT, I2S3_EXCK_PIN, Func_I2s, Disable);
#else
    PORT_Init(I2S3_MCK_PORT, I2S3_MCK_PIN, &stcPortIni);
    PORT_SetFunc(I2S3_MCK_PORT, I2S3_MCK_PIN, Func_I2s, Disable);
#endif

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_I2C2, Enable);
    /* Initialize i2c port for codec wm8731 */
    PORT_SetFunc(I2C2_SCL_PORT, I2C2_SCL_PIN, Func_I2c2_Scl, Disable);
    PORT_SetFunc(I2C2_SDA_PORT, I2C2_SDA_PIN, Func_I2c2_Sda, Disable);
}

/**
 *******************************************************************************
 ** \brief  Configuration Codec to record wav file
 ** \param  None
 ** \retval None
 ******************************************************************************/
static void WM8731_CodecConfig(void)
{
    stc_wm8731_reg_t stcWm8731Reg;
    MEM_ZERO_STRUCT(stcWm8731Reg);
    /* Config codec */
    /* Reset register */
    stcWm8731Reg.RESET              = 0x00U;    // Reset WM8731
    /* Left & right line input */
    stcWm8731Reg.LLIN_f.LINVOL      = 0x00U;    // Left channel line input volume: 0dB--0x17u
    stcWm8731Reg.LLIN_f.LINMUTE     = 1U;       // Enable left channel line input mute
    stcWm8731Reg.LLIN_f.LRINBOTH    = 0U;       // Disable simultaneous input volume and mute load from left to right
    stcWm8731Reg.RLIN_f.RINVOL      = 0x00U;    // Right channel line input volume 0dB
    stcWm8731Reg.RLIN_f.RINMUTE     = 1U;       // Enable right channel line input mute
    stcWm8731Reg.RLIN_f.RINBOTH     = 0U;       // Disable simultaneous input volume and mute load from right to left
    /* Left & right headphone output */
    stcWm8731Reg.LHOUT_f.LHPVOL     = 0x5F;     // Set volume of left headphone to 0dB. 0x30(-73dB) ~ 0x7F(+6dB), 0 ~ 0x2F: mute
    stcWm8731Reg.LHOUT_f.LZCEN      = 0U;       // Disable left channel zero cross detect
    stcWm8731Reg.LHOUT_f.LRHPBOTH   = 0U;       // Disable simultaneous output volume and mute load from left to right
    stcWm8731Reg.RHOUT_f.RHPVOL     = 0x5F;     // Set volume of right headphone to 0dB. 0x30(-73dB) ~ 0x7F(+6dB), 0 ~ 0x2F: mute
    stcWm8731Reg.RHOUT_f.RZCEN      = 0U;       // Enable right channel zero cross detect
    stcWm8731Reg.RHOUT_f.RLHPBOTH   = 0U;       // Disable simultaneous output volume and mute load from right to left
    /* Analog audio path control */
    stcWm8731Reg.AAPC_f.MICBOOST    = 1U;       // Disable boost
    stcWm8731Reg.AAPC_f.MUTEMIC     = 0U;       // Enable mute to ADC
    stcWm8731Reg.AAPC_f.INSEL       = 1U;       // Line input select to ADC
    stcWm8731Reg.AAPC_f.BYPASS      = 0U;       // Enbale bypass
    stcWm8731Reg.AAPC_f.DACSEL      = 1U;       // Select DAC
    stcWm8731Reg.AAPC_f.SIDETONE    = 0U;       // Disable side tone
    stcWm8731Reg.AAPC_f.SIDEATT     = 0U;       // 0: -6dB, 1: -12dB, 2: -9dB, 3: -15dB.
    /* Digital audio path control */
    stcWm8731Reg.DAPC_f.ADCHPD      = 0U;       // Enable high pass filter
    stcWm8731Reg.DAPC_f.DEEMP       = 0U;       // De-emphasis contrl. 0: disable, 1: 32kHz, 2: 44.1kHz, 3: 48kHz
    stcWm8731Reg.DAPC_f.DACMU       = 0U;       // 0:Disable soft mute   1: Enable soft mute
    stcWm8731Reg.DAPC_f.HPOR        = 0U;       // Clear offset when high pass
    /* Power down control */
    stcWm8731Reg.PDC_f.LINEINPD     = 1U;       // Disable line input power down
    stcWm8731Reg.PDC_f.MICPD        = 0U;       // Disable microphone input power down
    stcWm8731Reg.PDC_f.ADCPD        = 0U;       // Disable ADC power down
    stcWm8731Reg.PDC_f.DACPD        = 0U;       // Disable DAC power down
    stcWm8731Reg.PDC_f.OUTPD        = 0U;       // Disable output power down
    stcWm8731Reg.PDC_f.OSCPD        = 0U;       // Disable oscillator power down
    stcWm8731Reg.PDC_f.CLKOUTPD     = 0U;       // Disable CLKOUT power down
    stcWm8731Reg.PDC_f.POWEROFF     = 0U;       // Disable power off mode
    /* Digital audio interface format */
    stcWm8731Reg.DAIF_f.FORMAT      = 2U;       // 0: MSB-First, right justified, 1: MSB-first, left justified, 2: I2S-format, 3: DSP mode
    stcWm8731Reg.DAIF_f.IWL         = 0U;       // 0: 16 bits, 1: 20 bits, 2: 24 bits, 3: 32 bits
    stcWm8731Reg.DAIF_f.LRP         = 0U;       // 1: right channel DAC data when DACLRC (WS) is high,  0: right channel DAC data when DACLRC (WS) is low
    stcWm8731Reg.DAIF_f.LRSWAP      = 0U;       // 1: swap left channel and right channel, 0: don't swap
    stcWm8731Reg.DAIF_f.MS          = 0U;       // 1: Enable master mode, 0: Enable slave mode
    stcWm8731Reg.DAIF_f.BCLKINV     = 0U;       // Don't invert BCLK
    /* Sampling control */
    stcWm8731Reg.SC_f.NORMAL_USB    = 0U;       // 0: normal mode, 1: USB mode
    stcWm8731Reg.SC_f.BOSR          = 0U;       // Nomrmal mode: 0: 256fs, 1: 384fs
                                                // USB mode: 0: 250fs, 1:272fs
    stcWm8731Reg.SC_f.SR            = 0U;       // Sample rate setting
    stcWm8731Reg.SC_f.CLKDIV2       = 0U;       // 0: core clock is MCLK, 1: core clock is MCLK divided by 2
    stcWm8731Reg.SC_f.CLKODIV2      = 0U;       // 0: output clock is core clock, 1: core clock is core clock/2
    // Active control
    stcWm8731Reg.AC_f.ACTIVE        = 1U;       // 0: inactive, 1: active

    if(Ok != WM8731_Init(I2C_CH, &stcWm8731Reg))
    {
        while(1);
    }
    WM8731_SetHpVolume(I2C_CH, 0x6f,0x6f);  //0x2F-MUTE ~ 0x7F Maximum
}

/**
 ******************************************************************************
 ** \brief  configure the channel0 of DMA2, use it to move data from buf to I2S3_TX buf
 **
 ******************************************************************************/
void DMA_ConfigPlay(void)
{
    stc_dma_config_t stcDmaCfg;
    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Disable DMA1. */
    DMA_Cmd(DMA_UNIT_PLAY,Disable);
    DMA_ClearIrqFlag(DMA_UNIT_PLAY, DMA_CH_PLAY, BlkTrnCpltIrq);
    DMA_ClearIrqFlag(DMA_UNIT_PLAY, DMA_CH_PLAY, TrnCpltIrq);
    /* set the destination address */
    stcDmaCfg.u32DesAddr = (uint32_t)(&I2S_CH->TXBUF);

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma16Bit;

    /* Enable DMA clock. */
    PWC_Fcg0PeriphClockCmd(DMA_PLAY_FCG, Enable);

    /* Enable DMA1. */
    DMA_Cmd(DMA_UNIT_PLAY, Enable);
    /* Initialize DMA. */
    DMA_InitChannel(DMA_UNIT_PLAY, DMA_CH_PLAY, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(DMA_UNIT_PLAY, DMA_CH_PLAY, Enable);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
    DMA_SetTriggerSrc(DMA_UNIT_PLAY, DMA_CH_PLAY, EVT_I2S3_TXIRQOUT);
}

/**
 *******************************************************************************
 ** \brief  Initialize DMAC function for recorder
 ** \param  None
 ** \retval None
 ******************************************************************************/
void DMA_ConfigRecord(void)
{
    stc_dma_config_t stcDmaCfg;
    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Set data block size. */
    stcDmaCfg.u16BlockSize = 1U;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = PACKET;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&I2S_CH->RXBUF);

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressFix;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma16Bit;

    /* Enable DMA clock. */
    PWC_Fcg0PeriphClockCmd(DMA_RECORD_FCG, Enable);

    /* Enable DMA1. */
    DMA_Cmd(DMA_UNIT_RECORD,Enable);
    /* Initialize DMA. */
    DMA_InitChannel(DMA_UNIT_RECORD, DMA_CH_RECORD, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(DMA_UNIT_RECORD, DMA_CH_RECORD,Enable);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    DMA_SetTriggerSrc(DMA_UNIT_RECORD, DMA_CH_RECORD, EVT_I2S3_RXIRQOUT);
}

/**
 ******************************************************************************
 ** \brief  configure the interrupt vector of channel2 of DMA2
 **
 ******************************************************************************/
void DMA_IrqConfigRecord(void)
{
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    DMA_EnableIrq(DMA_UNIT_RECORD, DMA_CH_RECORD, TrnCpltIrq);
    /* Register TXIRQOUT Int to Vect.No.001 */
    stcIrqRegiConf.enIRQn = Int003_IRQn;
    /* Select interrupt function */
    stcIrqRegiConf.enIntSrc = DMA_RECORD_IRQ_SRC;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = DMA_TC_RecordCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 ******************************************************************************
 ** \brief  configure the interrupt vector of channel0 of DMA2
 **
 ******************************************************************************/
void DMA_IrqConfigPlay(void)
{
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    DMA_EnableIrq(DMA_UNIT_PLAY, DMA_CH_PLAY, TrnCpltIrq);
    /* Register TXIRQOUT Int to Vect.No.001 */
    stcIrqRegiConf.enIRQn = Int001_IRQn;
    /* Select interrupt function */
    stcIrqRegiConf.enIntSrc = DMA_PLAY_IRQ_SRC;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = DMA_TC_PlayCallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_14);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief  Get I2S clock frequency.
 **
 ** \param  [in] pstcI2sReg             Pointer to I2S register
 ** \arg    M4_I2S1                     I2s channel 1
 ** \arg    M4_I2S2                     I2s channel 2
 ** \arg    M4_I2S3                     I2s channel 3
 ** \arg    M4_I2S4                     I2s channel 4
 ** \retval uint32_t                    The  I2S clock frequency.
 ** \note   None
 ******************************************************************************/
uint32_t GetI2SClkFreq(M4_I2S_TypeDef* pstcI2sReg)
{
    en_clk_peri_source_t enSrc = ClkPeriSrcPclk;
    uint32_t u32Freq = 0ul;
    stc_clk_freq_t stcClkFreq;
    stc_pll_clk_freq_t stcPllClkFreq;

    /* Check parameters */
    if(NULL != pstcI2sReg)
    {
        enSrc = CLK_GetI2sClkSource(pstcI2sReg);
        CLK_GetClockFreq(&stcClkFreq);
        CLK_GetPllClockFreq(&stcPllClkFreq);
        switch(enSrc)
        {
            case ClkPeriSrcPclk:
                u32Freq = stcClkFreq.pclk1Freq;
                break;
            case ClkPeriSrcMpllp:
                u32Freq = stcPllClkFreq.mpllp;
                break;
            case ClkPeriSrcMpllq:
                u32Freq = stcPllClkFreq.mpllq;
                break;
            case ClkPeriSrcMpllr:
                u32Freq = stcPllClkFreq.mpllr;
                break;
            case ClkPeriSrcUpllp:
                u32Freq = stcPllClkFreq.upllp;
                break;
            case ClkPeriSrcUpllq:
                u32Freq = stcPllClkFreq.upllq;
                break;
            case ClkPeriSrcUpllr:
                u32Freq = stcPllClkFreq.upllr;
                break;
            default:
                break;
        }
    }
    return u32Freq;
}

/**
 ******************************************************************************
 ** \brief  configure the registers of I2S3
 **
 ******************************************************************************/
void I2s_Config(void)
{
    stc_i2s_config_t stcI2sCfg;
    MEM_ZERO_STRUCT(stcI2sCfg);

    /* Config i2s peripheral */
    I2s_DeInit(I2S_CH);
    stcI2sCfg.u32I2sInterClkFreq = GetI2SClkFreq(I2S_CH);
    stcI2sCfg.enStandrad = Std_Philips;
    stcI2sCfg.enMode = I2sMaster;
    stcI2sCfg.enFullDuplexMode = I2s_FullDuplex;
    stcI2sCfg.enChanelLen = I2s_ChLen_16Bit;
    stcI2sCfg.enDataBits = I2s_DataLen_16Bit;
    stcI2sCfg.u32AudioFreq = I2S_AudioFreq_48k;
#ifdef EXCK_ON
    stcI2sCfg.enMcoOutEn = Disable;
    stcI2sCfg.enExckEn = Enable;
#else
    stcI2sCfg.enMcoOutEn = Enable;
    stcI2sCfg.enExckEn = Disable;
#endif
    I2s_Init(I2S_CH, &stcI2sCfg);
}

/**
 *******************************************************************************
 ** \brief  Main function of project
 ** \param  None
 ** \retval int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();

    WM8731_IO_Ini();
    WM8731_CodecConfig();

    DMA_ConfigPlay();
    DMA_ConfigRecord();
    DMA_IrqConfigPlay();
    DMA_IrqConfigRecord();
    I2s_Config();

    while(1)
    {
        switch (work_state)
        {
            case STOP_STATE:
                /* K2 kick start record */
                if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
                {
                    Ddl_Delay1ms(400UL);
                    DMA_ConfigPlay();
                    I2s_Config();
                    Record_Start((uint32_t)IsocInBuff, PACKET);
                    work_state = RECORD_STATE;
                    BSP_LED_Toggle(LED_RED);
                }
                break;
            case IDLE_STATE:
                /* K2 kick start record */
                if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
                {
                    Ddl_Delay1ms(400UL);

                    Record_Start((uint32_t)IsocInBuff, PACKET);
                    work_state = RECORD_STATE;
                    BSP_LED_Toggle(LED_RED);
                }
                break;
            case RECORD_STATE:
                if(IsocInRdCnt >= (NUM>>1U))
                {
                    /* Kick start play */
                    work_state = ALL_RUN_STATE;
                    IsocInWrPtr = IsocInBuff;
                    Play_Repeat((uint32_t)IsocInWrPtr, PACKET);
                    IsocInWrCnt++;
                }
                break;

            case ALL_RUN_STATE:
                if(u8Play_tx_flag == 1U)
                {
                    /* Play next packet audio data */
                    u8Play_tx_flag = 0U;
                    Play_Repeat((uint32_t)(IsocInWrPtr+PACKET*IsocInWrCnt), PACKET);
                    IsocInWrCnt++;

                    if(IsocInWrCnt>=NUM)
                    {
                        IsocInWrCnt = 0U;
                        IsocInWrPtr = IsocInBuff;
                    }
                }
                /* K2 stop record and play */
                if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
                {
                    Ddl_Delay1ms(400UL);
                    Record_Stop();

                    work_state = STOP_STATE;
                    IsocInRdCnt = 0U;
                    BSP_LED_Toggle(LED_RED);
                }
            default:
                break;
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
