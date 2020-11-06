#include "bsp_interrupt.h"
void bsp_interrupt_callback_regist(en_int_src_t enIntSrc, IRQn_Type enIRQn, void *callback)
{
    stc_irq_regi_conf_t stcIrqRegiConf;
     /* Register Int to Vect.No.*/
    stcIrqRegiConf.enIRQn = enIRQn;
    /* Select I2C receive full interrupt function */
    stcIrqRegiConf.enIntSrc = enIntSrc;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = (func_ptr_t)callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

void bsp_interrupt_enable(IRQn_Type enIRQn, bool value)
{
    if(value == Enable)
    {
        NVIC_EnableIRQ(enIRQn);
    }
    else
    {
        NVIC_DisableIRQ(enIRQn);
    }
}

