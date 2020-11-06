#ifndef USER_TRNG_H
#define USER_TRNG_H
#include "hc32_ddl.h"

void User_TRNG_Init(void);
void Get_TRNG_Number(uint32_t *p_data);
uint32_t TRNG_GetRandData(void);

#endif
