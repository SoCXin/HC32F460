#ifndef BSP_INTERRUPT_H
#define BSP_INTERRUPT_H
#include "hc32_ddl.h"

#ifdef __cplusplus
extern "C" {
#endif
void bsp_interrupt_callback_regist(en_int_src_t enIntSrc, IRQn_Type enIRQn, void *callback);
void bsp_interrupt_enable(IRQn_Type enIRQn, bool value);
#ifdef __cplusplus
};
#endif    
    
#endif


