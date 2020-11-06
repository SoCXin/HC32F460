/*****************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
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
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
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
/** \file wm8731.h
 **
 ** A detailed description is available at
 ** @link Wm8731Group WM8731 description @endlink
 **
 **   - 2018-10-13  1.0 Wangmin First version for Device Driver Library.
 **
 ******************************************************************************/
#ifndef __WM8731_H__
#define __WM8731_H__


/*******************************************************************************
 * Include files
 ******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 *******************************************************************************
 ** \defgroup Wm8731Group Wm8731 CODECs with an integrated headphone driver
 **
 ******************************************************************************/
//@{

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*****************************************************************************/
/* Global pre-processor symbols/macros ('define')                            */
/*****************************************************************************/
#define WM8731_ADDRESS             (0x1Au)      // WM8731 chip address on I2C bus

#define WM8731_REG_LLINE_IN        (0x00u)      // Left Channel Line Input Volume Control
#define WM8731_REG_RLINE_IN        (0x01u)      // Right Channel Line Input Volume Control
#define WM8731_REG_LHPHONE_OUT     (0x02u)      // Left Channel Headphone Output Volume Control
#define WM8731_REG_RHPHONE_OUT     (0x03u)      // Right Channel Headphone Output Volume Control
#define WM8731_REG_ANALOG_PATH     (0x04u)      // Analog Audio Path Control
#define WM8731_REG_DIGITAL_PATH    (0x05u)      // Digital Audio Path Control
#define WM8731_REG_PDOWN_CTRL      (0x06u)      // Power Down Control Register
#define WM8731_REG_DIGITAL_IF      (0x07u)      // Digital Audio Interface Format
#define WM8731_REG_SAMPLING_CTRL   (0x08u)      // Sampling Control Register
#define WM8731_REG_ACTIVE_CTRL     (0x09u)      // Active Control
#define WM8731_REG_RESET           (0x0Fu)      // Reset register

/******************************************************************************
 * WM8731 registers bit field
 ******************************************************************************/
typedef struct stc_wm8731_llin_field
{
    uint16_t LINVOL   : 5;
    uint16_t RESERVED : 2;
    uint16_t LINMUTE  : 1;
    uint16_t LRINBOTH : 1;

}stc_wm8731_llin_field_t;

typedef struct stc_wm8731_rlin_field
{
    uint16_t RINVOL   : 5;
    uint16_t RESERVED : 2;
    uint16_t RINMUTE  : 1;
    uint16_t RINBOTH  : 1;

}stc_wm8731_rlin_field_t;

typedef struct stc_wm8731_lhout_field
{
    uint16_t LHPVOL   : 7;
    uint16_t LZCEN    : 1;
    uint16_t LRHPBOTH : 1;

}stc_wm8731_lhout_field_t;

typedef struct stc_wm8731_rhout_field
{
    uint16_t RHPVOL   : 7;
    uint16_t RZCEN    : 1;
    uint16_t RLHPBOTH : 1;

}stc_wm8731_rhout_field_t;

typedef struct stc_wm8731_aapc_field
{
    uint16_t MICBOOST   : 1;
    uint16_t MUTEMIC    : 1;
    uint16_t INSEL      : 1;
    uint16_t BYPASS     : 1;
    uint16_t DACSEL     : 1;
    uint16_t SIDETONE   : 1;
    uint16_t SIDEATT    : 2;

}stc_wm8731_aapc_field_t;

typedef struct stc_wm8731_dapc_field
{
    uint16_t ADCHPD   : 1;
    uint16_t DEEMP    : 2;
    uint16_t DACMU    : 1;
    uint16_t HPOR     : 1;

}stc_wm8731_dapc_field_t;

typedef struct stc_wm8731_pdc_field
{
    uint16_t LINEINPD   : 1;
    uint16_t MICPD      : 1;
    uint16_t ADCPD      : 1;
    uint16_t DACPD      : 1;
    uint16_t OUTPD      : 1;
    uint16_t OSCPD      : 1;
    uint16_t CLKOUTPD   : 1;
    uint16_t POWEROFF   : 1;

}stc_wm8731_pdc_field_t;

typedef struct stc_wm8731_daif_field
{
    uint16_t FORMAT     : 2;
    uint16_t IWL        : 2;
    uint16_t LRP        : 1;
    uint16_t LRSWAP     : 1;
    uint16_t MS         : 1;
    uint16_t BCLKINV    : 1;

}stc_wm8731_daif_field_t;

typedef struct stc_wm8731_sc_field
{
    uint16_t NORMAL_USB : 1;
    uint16_t BOSR       : 1;
    uint16_t SR         : 4;
    uint16_t CLKDIV2    : 1;
    uint16_t CLKODIV2   : 1;
}stc_wm8731_sc_field_t;

typedef struct stc_wm8731_ac_field
{
    uint16_t ACTIVE : 1;

}stc_wm8731_ac_field_t;

/******************************************************************************
 * WM8731 registers structure
 ******************************************************************************/
typedef struct stc_wm8731_reg
{
  union {
    __IO  uint16_t LLIN;                    ///< Left line in register
    stc_wm8731_llin_field_t LLIN_f;
  };

  union {
    __IO  uint16_t RLIN;                   ///< Right line in register
    stc_wm8731_rlin_field_t RLIN_f;
  };

  union {
    __IO  uint16_t LHOUT;                 ///< Left headphone out register
    stc_wm8731_lhout_field_t LHOUT_f;
  };

  union {
    __IO  uint16_t RHOUT;                 ///< Right headphone out register
    stc_wm8731_rhout_field_t RHOUT_f;
  };

  union {
    __IO  uint16_t AAPC;                  ///< Analog audio path control register
    stc_wm8731_aapc_field_t AAPC_f;
  };

  union {
    __IO  uint16_t DAPC;                 ///< Digital audio path control register
    stc_wm8731_dapc_field_t DAPC_f;
  };

  union {
    __IO  uint16_t PDC;                  ///< Power down control register
    stc_wm8731_pdc_field_t PDC_f;
  };

  union {
    __IO  uint16_t DAIF;                 ///< Digital audio interface format register
    stc_wm8731_daif_field_t DAIF_f;
  };

  union {
    __IO  uint16_t SC;                   ///< Sampling control register
    stc_wm8731_sc_field_t SC_f;
  };

  union {
    __IO  uint16_t AC;                   ///< Active control register
    stc_wm8731_ac_field_t AC_f;
  };

  __IO  uint16_t RESET;                ///< Reset register

}stc_wm8731_reg_t;


/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes (definition in C source)
 ******************************************************************************/
uint8_t WM8731_Init(M4_I2C_TypeDef* pstcI2c, stc_wm8731_reg_t* pstcReg);
uint8_t WM8731_SetHpVolume(M4_I2C_TypeDef* pstcI2c, uint16_t u16LeftVol, uint16_t u16RightVol);

//@} // Wm8731Group

#ifdef __cplusplus
}
#endif


#endif /* __WM8731_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
