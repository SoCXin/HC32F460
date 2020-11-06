/******************************************************************************
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
/** \file drv_i2c.c
 **
 ** History:
 **
 **   - 2019-02-14  1.0  Hongjh      First version.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_platform.h"

#include "drv_irq_cfg.h"
#include "drv_i2c.h"

#if defined(RT_USING_I2C)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

//#define HC32_I2C_DEBUG
#ifndef HC32_I2C_DEBUG
#define I2C_PRINT_DBG(fmt, args...)
#define I2C_PRINT_ERR(fmt, args...) rt_kprintf(fmt, ##args);
#else
#define I2C_PRINT_DBG(fmt, args...) rt_kprintf(fmt, ##args);
#define I2C_PRINT_ERR(fmt, args...) rt_kprintf(fmt, ##args);
#endif

#define I2C_TIMEOUT                 ((uint32_t)0x10000)
#define E2_ADDRESS_W                0x00
#define E2_ADDRESS_R                0x01

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

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief  Send start condition
 **
 ** \param  I2Cx   Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 **
 ** \retval Process result
 **         - RT_ERROR  Send start failed
 **         - RT_EOK     Send start success
 **
 ******************************************************************************/
static int hc32_hw_i2c_start(M4_I2C_TypeDef* I2Cx)
{
    volatile rt_uint32_t u32TimeOut = I2C_TIMEOUT;

    RT_ASSERT((M4_I2C1 == I2Cx) || (M4_I2C2 == I2Cx) || (M4_I2C3 == I2Cx));

    /* generate start signal */
    while(Set == I2C_GetStatus(I2Cx, I2C_SR_BUSY)) /* Wait I2C bus idle */
    {
        if(0 == (u32TimeOut--))
        {
            return RT_ERROR;
        }
    }

    I2C_GenerateStart(I2Cx , Enable);

    /* Judge if start success*/
    u32TimeOut = I2C_TIMEOUT;
    while((Reset == I2C_GetStatus(I2Cx, I2C_SR_BUSY)) ||
            (Reset == I2C_GetStatus(I2Cx, I2C_SR_STARTF)))
    {
        if(0 == (u32TimeOut--))
        {
            return RT_ERROR;
        }
    }

    return RT_EOK;
}

/**
 ******************************************************************************
 ** \brief  Send restart condition
 **
 ** \param  I2Cx   Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 **
 ** \retval Process result
 **         - RT_ERROR  Send start failed
 **         - RT_EOK     Send start success
 **
 ******************************************************************************/
//static int hc32_hw_i2c_restart(M4_I2C_TypeDef* I2Cx)
//{
//    volatile rt_uint32_t u32TimeOut;
//
//    RT_ASSERT((M4_I2C1 == I2Cx) || (M4_I2C2 == I2Cx) || (M4_I2C3 == I2Cx));
//
//    /* generate restart signal */
//    I2C_ClearStatus(I2Cx, I2C_CLR_STARTFCLR); /* Clear start status flag */
//    I2C_GenerateReStart(I2Cx , Enable); /* Send restart condition */
//
//    /* Judge if start success*/
//    u32TimeOut = I2C_TIMEOUT;
//    while((Reset == I2C_GetStatus(I2Cx, I2C_SR_BUSY)) ||
//            (Reset == I2C_GetStatus(I2Cx, I2C_SR_STARTF)))
//    {
//        if(0 == (u32TimeOut--))
//        {
//            return RT_ERROR;
//        }
//    }
//
//    return RT_EOK;
//}

/**
 ******************************************************************************
 ** \brief  Send e2prom device address
 **
 ** \param  I2Cx   Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param  u16Adr  The slave address
 **
 ** \retval Process result
 **         - RT_ERROR  Send start failed
 **         - RT_EOK     Send start success
 **
 ******************************************************************************/
static int hc32_hw_i2c_send_addr(M4_I2C_TypeDef* I2Cx, uint8_t u8Adr)
{
    volatile rt_uint32_t u32TimeOut;

    RT_ASSERT((M4_I2C1 == I2Cx) || (M4_I2C2 == I2Cx) || (M4_I2C3 == I2Cx));

    /* Wait tx buffer empty */
    u32TimeOut = I2C_TIMEOUT;
    while (Reset == I2C_GetStatus(I2Cx, I2C_SR_TEMPTYF))
    {
        if(0 == (u32TimeOut--))
        {
            return RT_ERROR;
        }
    }

    /* Send I2C address */
    u32TimeOut = I2C_TIMEOUT;
    I2C_SendData(I2Cx, u8Adr);

    if (E2_ADDRESS_W == (u8Adr & 0x01))
    {
        /* If in master transfer process, Need wait transfer end*/
        while (Reset == I2C_GetStatus(I2Cx, I2C_SR_TENDF))
        {
            if (0 == (u32TimeOut--))
            {
                return RT_ERROR;
            }
        }

        /* Check ACK */
        u32TimeOut = I2C_TIMEOUT;
        while (Set == I2C_GetStatus(I2Cx, I2C_SR_NACKDETECTF))
        {
            if (0 == (u32TimeOut--))
            {
                return RT_ERROR;
            }
        }
    }

    return RT_EOK;
}

/**
 ******************************************************************************
 ** \brief  Send data to e2prom
 **
 ** \param  I2Cx   Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 ** \param  pTxData  Pointer to the data buffer
 ** \param  u32Size  Data size
 **
 ** \retval Process result
 **         - RT_ERROR  Send start failed
 **         - RT_EOK     Send start success
 **
 ******************************************************************************/
static int hc32_hw_i2c_write_data(M4_I2C_TypeDef* I2Cx,
                                        uint8_t *pTxData,
                                        uint32_t u32Size)
{
    volatile rt_uint32_t u32TimeOut;

    RT_ASSERT(0 != u32Size);
    RT_ASSERT(RT_NULL != pTxData);
    RT_ASSERT((M4_I2C1 == I2Cx) || (M4_I2C2 == I2Cx) || (M4_I2C3 == I2Cx));

    while(u32Size--)
    {
        /* Wait tx buffer empty */
        u32TimeOut = I2C_TIMEOUT;
        while(Reset == I2C_GetStatus(I2Cx, I2C_SR_TEMPTYF))
        {
            if(0 == (u32TimeOut--))
            {
                return RT_ERROR;
            }
        }

        /* Send one byte data */
        I2C_SendData(I2Cx, *pTxData++);

        /* Wait transfer end*/
        u32TimeOut = I2C_TIMEOUT;
        while(Reset == I2C_GetStatus(I2Cx, I2C_SR_TENDF))
        {
            if(0 == (u32TimeOut--))
            {
                return RT_ERROR;
            }
        }

        /* Check ACK */
        u32TimeOut = I2C_TIMEOUT;
        while(Set == I2C_GetStatus(I2Cx, I2C_SR_NACKDETECTF))
        {
            if(0 == (u32TimeOut--))
            {
                return RT_ERROR;
            }
        }
    }

    return RT_EOK;
}

/**
 ******************************************************************************
 ** \brief  Write address and receive data from slave
 **
 ** \param  u8Adr    Device address and R/W bit
 ** \param  pTxData  Pointer to the data buffer
 ** \param  u32Size  Data size
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Process failed
 **         - I2C_RET_OK     Process success
 ******************************************************************************/
static int hc32_hw_i2c_read_data(M4_I2C_TypeDef* I2Cx,
                                        uint8_t *pRxData,
                                        uint32_t u32Size)
{
    volatile rt_uint32_t u32TimeOut;

    /* Wait tx buffer empty */
    while(Reset == I2C_GetStatus(I2Cx, I2C_SR_TEMPTYF))
    {
        if(0 == (u32TimeOut--))
        {
            return RT_ERROR;
        }
    }

    for(uint32_t i=0; i<u32Size; i++)
    {
        /* if the last byte receive, need config NACK*/
        if(i == (u32Size - 1))
        {
            I2C_NackConfig(I2Cx, Enable);
        }

        /* Wait receive full flag*/
        u32TimeOut = I2C_TIMEOUT;
        while(Reset == I2C_GetStatus(I2Cx, I2C_SR_RFULLF))
        {
            if(0 == (u32TimeOut--))
            {
                return RT_ERROR;
            }
        }

        /* read data from register*/
        *pRxData++ = I2C_ReadData(I2Cx);
    }

    return RT_EOK;
}

/**
 ******************************************************************************
 ** \brief  General stop condition to e2prom
 **
 ** \param  I2Cx   Pointer to the I2C peripheral register, can
 **                                be M4_I2C1,M4_I2C2 or M4_I2C3.
 **
 ** \retval Process result
 **         - RT_ERROR  Send start failed
 **         - RT_EOK     Send start success
 **
 ******************************************************************************/
static int hc32_hw_i2c_stop(M4_I2C_TypeDef* I2Cx)
{
    volatile uint32_t u32TimeOut;

    RT_ASSERT((M4_I2C1 == I2Cx) || (M4_I2C2 == I2Cx) || (M4_I2C3 == I2Cx));

    /* Wait I2C bus busy */
    u32TimeOut = I2C_TIMEOUT;
    while(Reset == I2C_GetStatus(I2Cx, I2C_SR_BUSY))
    {
        if(0 == (u32TimeOut--))
        {
            return RT_ERROR;
        }
    }

    I2C_GenerateStop(I2Cx, Enable);

    /* Wait STOPF */
    u32TimeOut = I2C_TIMEOUT;
    while(Reset == I2C_GetStatus(I2Cx, I2C_SR_STOPF))
    {
        if(0 == (u32TimeOut--))
        {
            return RT_ERROR;
        }
    }

    return RT_EOK;
}

static int hc32_i2c_init(struct hc32_i2c_instance *i2c_instance)
{
    int ret = RT_EOK;
    rt_uint32_t pwc_periph_clk;
    M4_I2C_TypeDef *I2Cx = (M4_I2C_TypeDef*)i2c_instance->handle;

    /* Initialize i2c port */
    PORT_SetFunc(i2c_instance->config.scl_port , i2c_instance->config.scl_pin, Func_I2c2_Scl, Disable);
    PORT_SetFunc(i2c_instance->config.sda_port , i2c_instance->config.sda_pin, Func_I2c2_Sda, Disable);

    /* Enable I2C Peripheral*/
    if (M4_I2C1 == I2Cx)
    {
        pwc_periph_clk = PWC_FCG1_PERIPH_I2C1;
    }
    else if (M4_I2C2 == I2Cx)
    {
        pwc_periph_clk = PWC_FCG1_PERIPH_I2C2;
    }
    else if (M4_I2C3 == I2Cx)
    {
        pwc_periph_clk = PWC_FCG1_PERIPH_I2C3;
    }
    else
    {
        return RT_ERROR;
    }

    PWC_Fcg1PeriphClockCmd(pwc_periph_clk, Enable);

    I2C_DeInit(I2Cx);

    I2C_Init(I2Cx, &i2c_instance->config.init);

    I2C_Cmd(I2Cx, Enable);

    return ret;
}

int hc32_i2c_write(M4_I2C_TypeDef* I2Cx,
                        rt_uint8_t address,
                        rt_uint8_t* buffer,
                        rt_uint32_t bytes)
{
    int ret;

    ret = hc32_hw_i2c_start(I2Cx);
    if (RT_EOK != ret)
    {
        goto out;
    }

    ret = hc32_hw_i2c_send_addr(I2Cx, (address << 1) | E2_ADDRESS_W);
    if (RT_EOK != ret)
    {
        goto out;
    }

    ret = hc32_hw_i2c_write_data(I2Cx, buffer, bytes);
    if (RT_EOK != ret)
    {
        goto out;
    }

    ret = hc32_hw_i2c_stop(I2Cx);
    if (RT_EOK != ret)
    {
        goto out;
    }

    return RT_EOK;
out:
    return ret;
}

int hc32_i2c_read(M4_I2C_TypeDef* I2Cx,
                        rt_uint8_t address,
                        rt_uint8_t* buffer,
                        rt_uint32_t bytes)
{
    int ret;

    ret = hc32_hw_i2c_start(I2Cx);
    if (RT_EOK != ret)
    {
        goto out;
    }

    ret = hc32_hw_i2c_send_addr(I2Cx, (address << 1) | E2_ADDRESS_W);
    if (RT_EOK != ret)
    {
        goto out;
    }

    ret = hc32_hw_i2c_read_data(I2Cx, buffer, bytes);
    if (RT_EOK != ret)
    {
        goto out;
    }

    ret = hc32_hw_i2c_stop(I2Cx);
    if (RT_EOK != ret)
    {
        goto out;
    }

    return RT_EOK;
out:
    return ret;
}

static rt_size_t hc32_i2c_xfer(struct rt_i2c_bus_device *dev,
                                struct rt_i2c_msg msgs[],
                                rt_uint32_t num)
{
    int ret;
    rt_uint32_t i;
    struct rt_i2c_msg *msg;

    struct hc32_i2c_instance *i2c_instance = (struct hc32_i2c_instance *)dev->priv;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags == RT_I2C_WR)
        {
            ret = hc32_i2c_write((M4_I2C_TypeDef *)i2c_instance->handle,
                                    msg->addr,
                                    msg->buf,
                                    msg->len);
            if (ret != RT_EOK)
            {
                I2C_PRINT_ERR("[%s:%d]I2C Write error!\n", __func__, __LINE__);
                goto out;
            }
        }
        else if (msg->flags == RT_I2C_RD)
        {
            ret = hc32_i2c_read((M4_I2C_TypeDef *)i2c_instance->handle,
                                msg->addr,
                                msg->buf,
                                msg->len);
            if (ret != RT_EOK)
            {
                I2C_PRINT_ERR("[%s:%d]I2C Read error!\n", __func__, __LINE__);
                goto out;
            }
        }
        else
        {
            I2C_PRINT_ERR("[%s:%d]I2C msg flag error!\n", __func__, __LINE__);
            goto out;
        }
    }

    ret = i;

out:
    i2c_dbg("send stop condition\n");
    return ret;
}

static const struct rt_i2c_bus_device_ops hc32_i2c_ops = {
    .master_xfer = hc32_i2c_xfer,
};

int hc32_i2c_probe(void *priv_data)
{
    int ret;
    char i2c_dev_name[5]       = {0};
    struct rt_i2c_bus_device *i2c_bus_dev;
    struct hc32_i2c_instance *i2c_instance = (struct hc32_i2c_instance *)priv_data;
    stc_clk_freq_t sysclk;
    CLK_GetClockFreq(&sysclk);
    if (priv_data == RT_NULL)
    {
        return -RT_EINVAL;
    }

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_malloc(sizeof(struct rt_i2c_bus_device));
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    rt_memset(i2c_bus_dev, 0, sizeof(struct rt_i2c_bus_device));
    i2c_bus_dev->ops = &hc32_i2c_ops;
    i2c_bus_dev->priv = i2c_instance;

    rt_sprintf(i2c_dev_name, "%s%d", "i2c", i2c_instance->id);
    ret = rt_i2c_bus_device_register(i2c_bus_dev, i2c_dev_name);
    if (ret != RT_EOK)
    {
        I2C_PRINT_ERR("ERROR:rt_spi_bus_register failed, ret=%d\n", ret);
        return -RT_ENOMEM;
    }
    i2c_instance->config.init.u32Baudrate = 400000;
    i2c_instance->config.init.u32Pclk3 = sysclk.pclk3Freq;
    i2c_instance->config.init.enI2cMode = I2cMaster;
    hc32_i2c_init(i2c_instance);

    return ret;
}

int hc32_i2c_exit(void *priv_data)
{
    rt_uint32_t pwc_periph_clk;
    M4_I2C_TypeDef *I2Cx = RT_NULL;
    struct hc32_i2c_instance *i2c_instance = (struct hc32_i2c_instance *)priv_data;

    if ((RT_NULL == priv_data) || (RT_NULL == i2c_instance))
    {
        return RT_ERROR;
    }

    I2Cx = (M4_I2C_TypeDef*)i2c_instance->handle;

    /* Disable I2C Peripheral*/
    if (M4_I2C2 == I2Cx)
    {
        pwc_periph_clk = PWC_FCG1_PERIPH_I2C1;
    }
    else if (M4_I2C2 == I2Cx)
    {
        pwc_periph_clk = PWC_FCG1_PERIPH_I2C2;
    }
    else if (M4_I2C2 == I2Cx)
    {
        pwc_periph_clk = PWC_FCG1_PERIPH_I2C3;
    }
    else if (M4_I2C2 == I2Cx)
    {
        return RT_ERROR;
    }

    PWC_Fcg1PeriphClockCmd(pwc_periph_clk, Disable);

    I2C_DeInit(I2Cx);

    return RT_EOK;
}

struct hc32_platform_driver i2c_driver_ops = {
    .name = "i2c",
    .probe = hc32_i2c_probe,
    .remove = hc32_i2c_exit,
};

void i2c_test(void);

int hc32_hw_i2c_init(void)
{
    I2C_PRINT_DBG("%s start\n", __func__);
    hc32_platform_driver_init(&i2c_driver_ops);
    I2C_PRINT_DBG("%s end\n", __func__);

    return 0;
}

INIT_BOARD_EXPORT(hc32_hw_i2c_init);

#ifdef I2C_TEST
void i2c_test(void)
{
    struct rt_i2c_bus_device *hc32_i2c = RT_NULL;

    hc32_i2c = rt_i2c_bus_device_find("i2c2");
    RT_ASSERT(hc32_i2c != RT_NULL);
    
#define WM8731_ADDRESS             (0x1A)       // WM8731 chip address on I2C bus
#define WM8731_REG_SAMPLING_CTRL   (0x08)       // Sampling Control Register

    struct rt_i2c_msg msg;
    rt_uint8_t send_buf[3];


    uint8_t u8RegAddr = WM8731_REG_SAMPLING_CTRL;
    uint16_t u16Cmd = 4;

    send_buf[0] =  (uint8_t)((u8RegAddr << 1) | ((u16Cmd>>8) & 0x1));
    send_buf[1] = (uint8_t)(u16Cmd & 0xFF);

    msg.addr  = WM8731_ADDRESS;
    msg.flags = RT_I2C_WR;
    msg.len   = 2;
    msg.buf   = send_buf;

    rt_i2c_transfer(hc32_i2c, &msg, 1);

    I2C_PRINT_DBG("data read from 0x3038 is 0x%x\r\n", data[0]);
    I2C_PRINT_DBG("%s end\n", __func__);
}
#endif

#endif

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
