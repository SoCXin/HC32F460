#include "hc32_ddl.h"

void User_TRNG_Init(void)
{
    stc_trng_init_t stcTrngInit;
    stcTrngInit.enLoadCtrl   = TrngLoadNewInitValue_Enable;
    stcTrngInit.enShiftCount = TrngShiftCount_64;
    TRNG_Init(&stcTrngInit);
}

void Get_TRNG_Number(uint32_t *p_data)
{
    TRNG_Generate(p_data, 10, 100);
}

uint32_t TRNG_GetRandData(void)
{
    uint32_t Data[2];
    Get_TRNG_Number(Data);
    return Data[0];
}
