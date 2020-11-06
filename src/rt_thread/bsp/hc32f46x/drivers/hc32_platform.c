/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co., Ltd. ("HDSC").
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
 * REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
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
/** \file hc32_platform.c
 **
 **   - 2019-02-15  1.0 Hongjh First version
 **
 ******************************************************************************/
#include <rtdevice.h>
#include <string.h>

#include "hc32_platform.h"

#ifdef RT_USING_I2C
#include "drv_i2c.h"
#endif

#ifdef RT_USING_SPI
#endif

#ifdef RT_USING_USART3
#include "drv_usart.h"
#endif

#ifdef RT_USING_SDIO
//#include "drv_sdio.h"
#endif

typedef struct hc32_platform_device
{
    char *name;
    void *private_data;
} hc32_platform_device_t;

#ifdef RT_USING_I2C

#ifdef RT_USING_I2C2
static struct hc32_i2c_instance i2c2_instance = {
    .id = 2,
    .handle = M4_I2C2,
    .config = {
        .scl_port = PortD,
        .scl_pin  = Pin00,
        .sda_port = PortD,
        .sda_pin  = Pin01,
        .init = {
            .enI2cMode = I2cMaster,
            .u32Baudrate = 100000,
        }
    },
};

struct hc32_platform_device plat_i2c2 = {
    .name = "i2c",
    .private_data = &i2c2_instance,
};
#endif

#endif

#ifdef RT_USING_SDIO
#endif

const static struct hc32_platform_device *platform_device[] = {
#ifdef RT_USING_I2C
#ifdef RT_USING_I2C2
    &plat_i2c2,
#endif
#endif
};

int hc32_platform_driver_init(struct hc32_platform_driver *plat_drv)
{
    int i,ret = RT_EOK;
    int device_cnt = sizeof(platform_device) / sizeof(platform_device[0]);

    if(!plat_drv || !plat_drv->name || !plat_drv->probe)
        return -RT_EINVAL;

    for (i = 0; i < device_cnt; i++)
    {
        if (!strcmp(plat_drv->name, platform_device[i]->name))
        {
            ret = plat_drv->probe(platform_device[i]->private_data);
        }
    }

    return ret;
}

int hc32_platform_driver_uninit(struct hc32_platform_driver *plat_drv)
{
    int i, ret = RT_EOK;
    int device_cnt = sizeof(platform_device) / sizeof(platform_device[0]);

    if(!plat_drv || !plat_drv->name || !plat_drv->probe)
        return -RT_EINVAL;

    for (i = 0; i < device_cnt; i++)
    {
        if (!strcmp(plat_drv->name, platform_device[i]->name))
        {
            ret = plat_drv->remove(platform_device[i]->private_data);
        }
    }

    return ret;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
