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
/** \file drv_usart.c
 **
 ** History:
 **
 **   - 2019-01-29  1.0  Hongjh      First version.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <rtdevice.h>
#include "hc32_ddl.h"
#include "drv_irq_cfg.h"
#include "drv_usart.h"

#if defined(RT_USING_UART1) || defined(RT_USING_UART2) || defined(RT_USING_UART3) || defined(RT_USING_UART4)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
struct hc32_usart
{
    M4_USART_TypeDef                *usart_device;
    IRQn_Type                       irq_rx;
    IRQn_Type                       irq_tx;
};

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define USART3_RX_PORT                  PortE
#define USART3_RX_PIN                   Pin04
#define USART3_RX_FUNC                  Func_Usart3_Rx

#define USART3_TX_PORT                  PortE
#define USART3_TX_PIN                   Pin05
#define USART3_TX_FUNC                  Func_Usart3_Tx

/* USART interrupt number  */
#define USART3_RI_NUM                   INT_USART3_RI

#define USART3_EI_NUM                   INT_USART3_EI

#define USART3_TI_NUM                   INT_USART3_TI

#define USART3_TCI_NUM                  INT_USART3_TCI

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static rt_err_t hc32_configure(struct rt_serial_device *serial, struct serial_configure *cfg);
static rt_err_t hc32_control(struct rt_serial_device *serial, int cmd, void *arg);
static int hc32_putc(struct rt_serial_device *serial, char c);
static int hc32_getc(struct rt_serial_device *serial);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
#if defined(RT_USING_UART3)
static struct hc32_usart usart3 =
{
    M4_USART3,
    USART3_RI_IRQn,
    USART3_TCI_IRQn,
};
static struct rt_serial_device serial3;
#endif

static const struct rt_uart_ops hc32_usart_ops =
{
    hc32_configure,
    hc32_control,
    hc32_putc,
    hc32_getc,
};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static rt_err_t hc32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct hc32_usart* usart = serial->parent.user_data;
    stc_usart_uart_init_t stc_UartCfg;

    RT_ASSERT(RT_NULL != cfg);
    RT_ASSERT(RT_NULL != usart);
    RT_ASSERT(RT_NULL != serial);

    MEM_ZERO_STRUCT(stc_UartCfg);

    if(BIT_ORDER_LSB == cfg->bit_order)
    {
        stc_UartCfg.enDirection = UsartDataLsbFirst;
    }
    else
    {
        stc_UartCfg.enDirection = UsartDataMsbFirst;
    }

    switch(cfg->stop_bits)
    {
        case STOP_BITS_1:
            stc_UartCfg.enStopBit = UsartOneStopBit;
            break;
        case STOP_BITS_2:
            stc_UartCfg.enStopBit = UsartTwoStopBit;
            break;
        default:
            RT_ASSERT(0);
            break;
    }

    switch(cfg->parity)
    {
        case PARITY_NONE:
            stc_UartCfg.enParity = UsartParityNone;
            break;
        case PARITY_EVEN:
            stc_UartCfg.enParity = UsartParityEven;
            break;
        default:
            stc_UartCfg.enParity = UsartParityOdd;
            break;
    }

    switch(cfg->data_bits)
    {
        case DATA_BITS_8:
            stc_UartCfg.enDataLength = UsartDataBits8;
            break;
        case DATA_BITS_9:
            stc_UartCfg.enDataLength = UsartDataBits9;
            break;
        default:
            RT_ASSERT(0);
            break;
    }

    USART_UART_Init(usart->usart_device, &stc_UartCfg);

    USART_SetBaudrate(usart->usart_device, cfg->baud_rate);

    if(serial->parent.flag & RT_DEVICE_FLAG_RDWR || serial->parent.flag & RT_DEVICE_FLAG_RDONLY)
    {
        USART_FuncCmd(usart->usart_device, UsartRx, Enable);
    }

    if(serial->parent.flag & RT_DEVICE_FLAG_RDWR || serial->parent.flag & RT_DEVICE_FLAG_WRONLY)
    {
        USART_FuncCmd(usart->usart_device, UsartTx, Enable);
    }

    return RT_EOK;
}

static rt_err_t hc32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);
    struct hc32_usart* usart = (struct hc32_usart *)serial->parent.user_data;

    RT_ASSERT(RT_NULL != usart);
    RT_ASSERT(RT_NULL != serial);

    switch (cmd)
    {
        case RT_DEVICE_CTRL_CLR_INT:

            if(RT_DEVICE_FLAG_INT_RX == ctrl_arg)
            {
                /* Disable RX irq */
                NVIC_DisableIRQ(usart->irq_rx);
                USART_FuncCmd(usart->usart_device, UsartRxInt, Disable);
            }
            else
            {
                /* Disable TX complete irq */
                NVIC_DisableIRQ(usart->irq_tx);
                USART_FuncCmd(usart->usart_device, UsartTxCmpltInt, Disable);
            }

            break;
        case RT_DEVICE_CTRL_SET_INT:

            if(RT_DEVICE_FLAG_INT_RX == ctrl_arg)
            {
                /* Enable RX irq */
                NVIC_EnableIRQ(usart->irq_rx);
                USART_FuncCmd(usart->usart_device, UsartRxInt, Enable);
            }
            else
            {
                /* Enable TX complete irq */
                NVIC_EnableIRQ(usart->irq_tx);

                /* Do not enable tx interrupt here! */
            }

            break;
        case RT_DEVICE_CTRL_CONFIG:
            //TBD
            break;
    }

    return RT_EOK;
}

static int hc32_putc(struct rt_serial_device *serial, char c)
{
    struct hc32_usart* usart= (struct hc32_usart*)serial->parent.user_data;

    RT_ASSERT(RT_NULL != usart);
    RT_ASSERT(RT_NULL != serial);

    if(serial->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        /* Interrupt mode. */
        if(c == '\n')
        {
            c = '\r';
        }

        USART_SendData(usart->usart_device, c);
        USART_FuncCmd(usart->usart_device, UsartTxCmpltInt, Enable);

        /* return -1 to wait TC. */
        return -1;
    }
    else
    {
        /* Polling mode. */
        USART_SendData(usart->usart_device, c);
        while(USART_GetStatus(usart->usart_device, UsartTxEmpty) != Set);
    }

    return 1;
}

static int hc32_getc(struct rt_serial_device *serial)
{
    int ch= -1;
    struct hc32_usart* usart = (struct hc32_usart*)serial->parent.user_data;;

    RT_ASSERT(RT_NULL != usart);
    RT_ASSERT(RT_NULL != serial);

    if(Set == USART_GetStatus(usart->usart_device, UsartRxNoEmpty))
    {
        ch = (rt_uint8_t)USART_RecData(usart->usart_device);
    }

    return ch;
}

static void uart3_rx_irq_handler(void)
{
    struct hc32_usart* usart = (struct hc32_usart*)serial3.parent.user_data;

    RT_ASSERT(RT_NULL != usart);

    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_IND);

    /* Do not support clear RXFF flag.Recv data will clear it. */

    /* leave interrupt */
    rt_interrupt_leave();
}

static void uart3_rx_err_irq_handler(void)
{
    struct hc32_usart* usart = (struct hc32_usart*)serial3.parent.user_data;

    RT_ASSERT(RT_NULL != usart);

    /* enter interrupt */
    rt_interrupt_enter();

    if (Set == USART_GetStatus(usart->usart_device, UsartFrameErr))
    {
        USART_ClearStatus(usart->usart_device, UsartFrameErr);
    }

    if (Set == USART_GetStatus(usart->usart_device, UsartParityErr))
    {
        USART_ClearStatus(usart->usart_device, UsartParityErr);
    }

    if (Set == USART_GetStatus(usart->usart_device, UsartOverrunErr))
    {
        USART_ClearStatus(usart->usart_device, UsartOverrunErr);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

static void uart3_tx_irq_handler(void)
{
    struct hc32_usart* usart = (struct hc32_usart*)serial3.parent.user_data;

    RT_ASSERT(RT_NULL != usart);

    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_TX_DONE);

    /* Disable TC. Do not support clear TC flag.Send data will clear it. */
    USART_FuncCmd(usart->usart_device, UsartTxCmpltInt, Enable);

    /* leave interrupt */
    rt_interrupt_leave();
}

int hc32_hw_usart_init(void)
{
    struct hc32_usart *usart;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART3
    /* Enable peripheral UART3 clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART3_RX_PORT, USART3_RX_PIN, USART3_RX_FUNC, Disable);
    PORT_SetFunc(USART3_TX_PORT, USART3_TX_PIN, USART3_TX_FUNC, Disable);

    usart = &usart3;

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = USART3_RI_IRQn;
    stcIrqRegiCfg.pfnCallback = uart3_rx_irq_handler;
    stcIrqRegiCfg.enIntSrc = USART3_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    //NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = USART3_EI_IRQn;
    stcIrqRegiCfg.pfnCallback = uart3_rx_err_irq_handler;
    stcIrqRegiCfg.enIntSrc = USART3_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = USART3_TCI_IRQn;
    stcIrqRegiCfg.pfnCallback = uart3_tx_irq_handler;
    stcIrqRegiCfg.enIntSrc = USART3_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    //NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    serial3.ops = &hc32_usart_ops;
    serial3.config = config;

    rt_hw_serial_register(&serial3,
                          RT_CONSOLE_DEVICE_NAME,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          usart);
#endif

    return 0;
}

INIT_BOARD_EXPORT(hc32_hw_usart_init);

#endif

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
