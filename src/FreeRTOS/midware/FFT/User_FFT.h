#ifndef USER_FFT_H
#define USER_FFT_H
#include "hc32_ddl.h"
void InitBufInArray();
void fft_test(float *sourcedata, float *Result);
void User_FFT_Test();
extern float32_t f_testOutput[4096];
#endif


