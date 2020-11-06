#include "hc32_ddl.h"
#include <math.h>
#include <arm_math.h>
#include "OLED.h"
#include "User_Gpio.h"
#define RFFT 0
#define RFFT_FAST 1
#define CFFT_RADIX4 2
#define RFFT_MODE RFFT_FAST

//#define Fs 10240//采样频率,分辩率10HZ,
#define FS  12800//采样频率 50x256 = 12.8K,分辨率12800/256 = 50HZ，每格数据是50HZ

#define PI2 6.28318530717959f
#define FFT_SIZE_4096           4096
#define FFT_SIZE_1024           1024
#define FFT_SIZE_256            256
#define FFT_SIZE_2048			2048
#define NPT FFT_SIZE_2048//1024点FFT
#define Sameples NPT
#if     RFFT_MODE == RFFT_FAST  //实域FFT
float32_t f_testInput[NPT];      
#elif   RFFT_MODE == CFFT_RADIX4 || RFFT_MODE == RFFT                    //复杂FFT
float32_t f_testInput[NPT*2];        
#endif
float32_t f_testOutput[NPT];
/** Magnitude buffer converted in float */
float32_t mag_in_buffer[NPT];
arm_cfft_radix4_instance_f32 *f_fft_data;

void InitBufInArray(void)
{
     unsigned short i;
//     static float fx;
     for(i=0; i<NPT; i++)
     {        
#if     RFFT_MODE == RFFT_FAST
         f_testInput[i] = 100*arm_sin_f32(PI2*i*500.0/FS) + 300*arm_sin_f32(PI2*i*300.0/FS)  + 1000*arm_sin_f32(PI2*i*1000/FS);//1000*arm_sin_f32(PI2*i*500.0/Fs) + 2000*arm_sin_f32(PI2*i*1250.0/Fs)  + 3000*arm_sin_f32(PI2*i*1450.0/Fs);
#elif   RFFT_MODE == CFFT_RADIX4 || RFFT_MODE == RFFT
         f_testInput[2*i] = 1000*arm_sin_f32(PI2*i*500.0/FS) + 2000*arm_sin_f32(PI2*i*1250.0/FS)  + 3000*arm_sin_f32(PI2*i*1450.0/FS);
         f_testInput[2*i+1] = 0;
#endif                  
         f_testOutput[i] = 0;
     }
}

void fft_test(float *sourcedata, float *Result)
{
//    uint16_t i;
#if RFFT_MODE == RFFT    
    arm_rfft_instance_f32 S;
    arm_cfft_radix4_instance_f32 S_CFFT;
    arm_cfft_radix4_init_f32(&S_CFFT,NPT,0,1);
    arm_rfft_init_f32(&S,&S_CFFT,NPT,0,1);
    arm_rfft_f32(&S,sourcedata,Result);
#elif RFFT_MODE == RFFT_FAST    
    arm_rfft_fast_instance_f32 S;
    arm_rfft_fast_init_f32(&S,NPT);
    arm_rfft_fast_f32(&S,sourcedata,Result,0);
#elif RFFT_MODE == CFFT_RADIX4
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S,NPT,0,1);
    arm_cfft_radix4_f32(&S,sourcedata);
#endif    
    arm_cmplx_mag_f32(sourcedata,Result,NPT);
}
void User_FFT_Test()
{
    static int i;//,cnt,j;
    InitBufInArray();
    printf("fft_inpurt:");
//    for(i = 0;i < NPT; i++)
//    {
//        printf("%f ",f_testInput[i]);
////        printf("\r\n");
//    }
    printf("\r\n");
    PORT_SetBits(LED3_PORT,LED3_Pin);
    Ddl_Delay1ms(1);
    PORT_ResetBits(LED3_PORT,LED3_Pin);
    fft_test(f_testInput,f_testOutput);
     PORT_SetBits(LED3_PORT,LED3_Pin);
    printf("fft_outpurt:");
//    cnt = 0;
//    disp_pos = 0;
    for(i = 0;i < NPT/2; i++)
    {
        printf("%f ",f_testOutput[i]/NPT);
        printf("\r\n");
    }
    printf("\r\n");
}

