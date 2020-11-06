#include "hc32_ddl.h"
#include "sd_card.h"

/* SDIOC Port/Pin definition */
#define SDIOC_CD_PORT                   PortE
#define SDIOC_CD_PIN                    Pin14

#define SDIOC_CK_PORT                   PortC
#define SDIOC_CK_PIN                    Pin12

#define SDIOC_CMD_PORT                  PortD
#define SDIOC_CMD_PIN                   Pin02

#define SDIOC_D0_PORT                   PortC
#define SDIOC_D0_PIN                    Pin08

#define SDIOC_D1_PORT                   PortC
#define SDIOC_D1_PIN                    Pin09

#define SDIOC_D2_PORT                   PortC
#define SDIOC_D2_PIN                    Pin10

#define SDIOC_D3_PORT                   PortC
#define SDIOC_D3_PIN                    Pin11

/* SD sector && count */
#define SD_SECTOR_START                 (0u)
#define SD_SECTOR_COUNT                 (4u)

static en_result_t SdiocInitPins(void);
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_sdcard_init_t m_stcCardInitCfg =
{
    SdiocBusWidth4Bit,
    SdiocClk50M,
    SdiocHighSpeedMode,
};

static stc_sdcard_dma_init_t m_stcDmaInitCfg =
{
    M4_DMA1,
    DmaCh0,
};

stc_sd_handle_t m_stcSdhandle =
{
    M4_SDIOC1,
    SdCardDmaMode,
    &m_stcDmaInitCfg,
};

static uint32_t m_u32ReadBlocks[512];
static uint32_t m_u32WriteBlocks[512];
 /******************************************************************************
 ** \brief Initialize SDIO pins
 **
 ** \param [in] None
 **
 ** \retval Ok  SDIO pins initialized successfully
 ** 
 ******************************************************************************/
static en_result_t SdiocInitPins(void)
{
    PORT_SetFunc(SDIOC_D0_PORT, SDIOC_D0_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D1_PORT, SDIOC_D1_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D2_PORT, SDIOC_D2_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D3_PORT, SDIOC_D3_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CD_PORT, SDIOC_CD_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CK_PORT, SDIOC_CK_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CMD_PORT, SDIOC_CMD_PIN, Func_Sdio, Disable);

    return Ok;
}

en_result_t SD_CARD_Init(void)
{
    en_result_t enTestResult = Ok;
    SdiocInitPins();
    /* Initialilze SD card */
    if(Ok != SDCARD_Init(&m_stcSdhandle, &m_stcCardInitCfg))
    {
        enTestResult = Error;
    }
        return enTestResult;
}
en_result_t FS_SD_ReadBlocks(const uint8_t* buff, uint16_t BlockAddr, uint8_t BlockCnt)
{
    return SDCARD_ReadBlocks(&m_stcSdhandle, BlockAddr,BlockCnt,(uint8_t *)buff,2000);
}
en_result_t FS_SD_WriteBlocks(const uint8_t* buff, uint16_t BlockAddr, uint8_t BlockCnt)
{
    return SDCARD_WriteBlocks(&m_stcSdhandle, BlockAddr,BlockCnt,(uint8_t *)buff,2000);
}  
void SD_CARD_TEST(void)
{
    en_result_t enTestResult = Ok;
     /* Erase SD card */
    if(Ok != SDCARD_Erase(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, 20000))
    {
        enTestResult = Error;
    }

    /* Read SD card */
    if(Ok != SDCARD_ReadBlocks(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)m_u32ReadBlocks, 2000))
    {
        enTestResult = Error;
    }

    /* Check whether data value is OxFFFFFFFF after erase SD card */
    for(uint32_t i = 0;  i < 512; i++)
    {
        if(m_u32ReadBlocks[i] != 0xFFFFFFFF)
        {
            enTestResult = Error;
            break;
        }
    }

    /* Write SD card */
    if(Ok != SDCARD_WriteBlocks(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)m_u32WriteBlocks, 2000))
    {
        enTestResult = Error;
    }

    /* Read SD card */
    if(Ok != SDCARD_ReadBlocks(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)m_u32ReadBlocks, 20000))
    {
        enTestResult = Error;
    }

    /* Compare read/write data */
    if(0 != memcmp(m_u32WriteBlocks, m_u32ReadBlocks, sizeof(m_u32ReadBlocks)))
    {
        enTestResult = Error;
    }
}    
void hd_sdio_hw_init()
{
	;
}
void enable_sdio_hw(bool enable)
{
	;
}
en_result_t SD_ReadDisk(uint8_t* buf, uint32_t blk_addr, uint16_t blk_len)
{
	;
}
en_result_t SD_WriteDisk(uint8_t* buf, uint32_t blk_addr, uint16_t blk_len)
{
	;
}