#ifndef SYSTEM_CLK_CFG_H
#define SYSTEM_CLK_CFG_H
#include "hc32_ddl.h"

#define ENABLE	1u
#define DISABLE	0u

#define XTAL_OSCMODE     ClkXtalModeOsc
#define XTAL_CLOCKMODE   ClkXtalModeExtClk

#define XTALDRV_H   ClkXtalHighDrv//驱动能力高
#define XTALDRV_M   ClkXtalMidDrv//驱动能力中
#define XTALDRV_L   ClkXtalLowDrv//驱动能力低
#define XTALDRV_XL  ClkXtalTinyDrv//超低驱动能力

#define XTALSTB_35CLK       1u
#define XTALSTB_67CLK       2u
#define XTALSTB_131CLK      3u
#define XTALSTB_259CLK      4u
#define XTALSTB_547CLK      5u
#define XTALSTB_1059CLK     6u
#define XTALSTB_2147CLK     7u
#define XTALSTB_4291CLK     8u
#define XTALSTB_8163CLK     9u

#define XTAL32DRV_H   ClkXtal32HighDrv//驱动能力高
#define XTAL32DRV_M   ClkXtal32MidDrv//驱动能力中
#define XTAL32DRV_L   ClkXtal32LowDrv//驱动能力低
#define XTAL32DRV_XL  ClkXtal32TinyDrv//超低驱动能力

#define XTAL32NF_ALWAYS_ON  ClkXtal32FilterModeFull
#define XTAL32NF_Sleep_STOP ClkXtal32FilterModePart
#define XTAL32NF_OFF        ClkXtal32FilterModeNone

#define PLL_CLK_SOURCE_HRC  ClkPllSrcHRC
#define PLL_CLK_SOURCE_XTAL ClkPllSrcXTAL

#define PLL_DIV2    1u
#define PLL_DIV3    2u
#define PLL_DIV4    3u
#define PLL_DIV5    4u
#define PLL_DIV6    5u
#define PLL_DIV7    6u
#define PLL_DIV8    7u
#define PLL_DIV9    8u
#define PLL_DIV10   9u
#define PLL_DIV11   10u
#define PLL_DIV12   11u
#define PLL_DIV13   12u
#define PLL_DIV14   13u
#define PLL_DIV15   14u
#define PLL_DIV16   15u
//----------以下分频只适用于输入分频---------------//
#define PLL_DIV17   16u
#define PLL_DIV18   17u
#define PLL_DIV19   18u
#define PLL_DIV20   19u
#define PLL_DIV21   20u
#define PLL_DIV22   21u
#define PLL_DIV23   22u
#define PLL_DIV24   23u

#define SYSCLK_HRC      ClkSysSrcHRC
#define SYSCLK_MRC      ClkSysSrcMRC
#define SYSCLK_LRC      ClkSysSrcLRC
#define SYSCLK_XTAL     ClkSysSrcXTAL
#define SYSCLK_XTAL32   ClkSysSrcXTAL32
#define SYSCLK_MPLL     CLKSysSrcMPLL

#define SYSCLK_DIV1     ClkSysclkDiv1
#define SYSCLK_DIV2     ClkSysclkDiv2
#define SYSCLK_DIV4     ClkSysclkDiv4
#define SYSCLK_DIV8     ClkSysclkDiv8
#define SYSCLK_DIV16    ClkSysclkDiv16
#define SYSCLK_DIV32    ClkSysclkDiv32
#define SYSCLK_DIV64    ClkSysclkDiv64

#define USBCLK_SYS_DIV2 ClkUsbSrcSysDiv2
#define USBCLK_SYS_DIV3 ClkUsbSrcSysDiv3
#define USBCLK_SYS_DIV4 ClkUsbSrcSysDiv4
#define USBCLK_MPLL_P   ClkUsbSrcMpllp
#define USBCLK_MPLL_Q   ClkUsbSrcMpllq
#define USBCLK_MPLL_R   ClkUsbSrcMpllr
#define USBCLK_UPLL_P   ClkUsbSrcUpllp
#define USBCLK_UPLL_Q   ClkUsbSrcUpllq
#define USBCLK_UPLL_R   ClkUsbSrcUpllr

#define PERICLK_Default 0u
#define PERICLK_MPLL_P  8u
#define PERICLK_MPLL_Q  9u
#define PERICLK_MPLL_R  10u
#define PERICLK_UPLL_P  11u
#define PERICLK_UPLL_Q  12u
#define PERICLK_UPLL_R  13u

//------------------Configure XTAL---------------------------//
#define XTAL_ENABLE         ENABLE
#define XTAL_SUPDRV_ENABLE  Enable
#define XTAL_MODE           XTAL_OSCMODE
#define XTAL_DRV            XTALDRV_H
#define XTAL_STB            XTALSTB_2147CLK
#define XTALSTDE_ENABLE     DISABLE
#define XTALSTDRIS_ENABLE   DISABLE
#define XTALSTDRE_ENABLE    DISABLE
#define XTALSTDIE_ENABLE    DISABLE

//------------------Configure XTAL32K-------------------------//
#define XTAL32_ENABLE           ENABLE
#define XTAL32_SUPDRV_ENABLE    Disable
#define XTAL32_DRV              XTAL32DRV_H
#define XTAL32_NF_MODE          XTAL32NF_ALWAYS_ON

//---------------------Configure HRC-------------------------//
#define HRC_ENABLE              DISABLE
//--------------------Configure MRC--------------------------//
#define MRC_ENABLE              ENABLE
//--------------------Configure LRC--------------------------//
#define LRC_ENABLE              ENABLE
//-------------------Configure MainCLK-----------------------//
#define SYSTEMCLKSOURCE         SYSCLK_MPLL
//HCLK for Core, SRAM, GPIO, MPU, DCU, QSPI, INTC
#define HCLK_DIV                SYSCLK_DIV1
//EXCKS MAX 84MHZ   for SDIO, CAN
#define EXCKS_DIV               SYSCLK_DIV4
//PCLK4 MAX 84MHZ for ADC Logic, TRNG
#define PCLK4S_DIV              SYSCLK_DIV2
//PCLK3 MAX 42MHZ for I2C, RTC, CMP, WDT, SWDT, 
#define PCLK3S_DIV              SYSCLK_DIV4 
//PCLK2 MAX 60MHZ for ADC convert
#define PCLK2S_DIV              SYSCLK_DIV4
//PCLK1 MAX 84MHz for SPI, USART, Timer0, TimerA, Timer4, Timer6 control Clock, EMB, CRC, HASH, AES, I2S
#define PCLK1S_DIV              SYSCLK_DIV2
//PCLK0 for Timer6 counter clock MAX 168MHZ
#define PCLK0S_DIV              SYSCLK_DIV1
//---------------------Configure PLL---------------------------//
//Configure PLL Clock Source
#define PLL_CLK_SOURCE  PLL_CLK_SOURCE_HRC
//------------------Configure MPLL CLK-------------------------//
//Enable MPLL
#define MPLL_CLK_ENABLE  ENABLE
//MPLL Source Clock Divider HRC 16MHZ DIV2 8MHZ
#define MPLL_CLK_M_DIV   2//PLL_DIV2+1
//----------------倍频系数，最低倍频数20，最高80-----------------//
//MPLL 倍频系数 21x8MHZ = 168MHZ
#define MPLL_CLK_NUM        42u//75u//
//MPLL_P,Q,R分频
#define MPLL_CLK_P_DIV      PLL_DIV2+1//MPLL 预分频数
#define MPLL_CLK_Q_DIV      PLL_DIV7+1
#define MPLL_CLK_R_DIV      PLL_DIV15+1


//------------------Configure UPLL CLK------------------------//
#define UPLL_CLK_ENABLE  ENABLE
#define UPLL_CLK_SOURCE  PLL_CLK_SOURCE//必须与MPLL相同
#define UPLL_CLK_M_DIV   8//PLL_DIV2+1
//倍频系数，最低倍频数20，最高80/
#define UPLL_CLK_NUM      384u

#define UPLL_CLK_P_DIV      1//UPLL 预分频数
#define UPLL_CLK_Q_DIV      PLL_DIV2+1
#define UPLL_CLK_R_DIV      PLL_DIV2+1

//-----------------Configure USB CLK------------------------//
#define USB_CLK_SOURCE      USBCLK_MPLL_Q

//-----------------Configure AD & PERIPHERAL TRNG CLK---------------------//
#define PERI_CLK_SOURCE     PERICLK_MPLL_Q
#endif
