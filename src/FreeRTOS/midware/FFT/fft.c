/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "hc32_ddl.h"
#include "fft.h"
#define BOARD_MCK 168000000
#define TC_Start() TIMER0_Cmd(M4_TMR01, Tim0_ChannelA, Enable)
#define RESET_CYCLE_COUNTER() TIMER0_WriteCntReg(M4_TMR01,Tim0_ChannelA,0x0000)
#define Get_TMR0CHA_CNT()  TIMER0_GetCntReg(M4_TMR01,Tim0_ChannelA)
#define TC_Stop() TIMER0_Cmd(M4_TMR01, Tim0_ChannelA, Disable)
/** macro to control whether to use deprecated functions in CMSIS-DSP v1.4.5  */
#define USING_DEPRECATED 1

#if (USING_DEPRECATED == 0)
	#include "arm_const_structs.h"
	extern const arm_cfft_instance_q15 arm_cfft_sR_q15_len256;
	extern const arm_cfft_instance_q15 arm_cfft_sR_q15_len1024;
	extern const arm_cfft_instance_q15 arm_cfft_sR_q15_len4096;
	extern const arm_cfft_instance_q31 arm_cfft_sR_q31_len256;
	extern const arm_cfft_instance_q31 arm_cfft_sR_q31_len1024;
	extern const arm_cfft_instance_q31 arm_cfft_sR_q31_len4096;
#endif

/*----------------------------------------------------------------------------
 *        Imported variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/
volatile uint32_t time_total_us;
volatile int32_t time_fft_us;
volatile uint32_t CycleCounter[3];

/** wave buffer in float */
float32_t wav_in_buffer[MAX_FFT_SIZE * 2];

/** Magnitude buffer converted in float */
float32_t mag_in_buffer[MAX_FFT_SIZE];

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** cFFT buffer declaration */
static union {
	/** q15 cFFT buffer declaration */
	q15_t q15[MAX_FFT_SIZE * 2];
	/** q31 cFFT buffer declaration, F31 cFFT use the same buffer */
	q31_t q31[MAX_FFT_SIZE * 2];
	/** q31 cFFT buffer declaration, F31 cFFT use the same buffer */
	q31_t f32[MAX_FFT_SIZE * 2];
}cfft_buffer;

/** cFFT instance declaration */
#if (USING_DEPRECATED == 1)
static union {
	/** cFFT configuration instance declaration Q15 */
	arm_cfft_radix4_instance_q15 q15;
	/** cFFT configuration instance declaration Q31 */
	arm_cfft_radix4_instance_q31 q31;
	/** cFFT configuration instance declaration F32 */
	arm_cfft_radix4_instance_f32 f32;
#else
static struct {
	/** cFFT configuration instance declaration Q15 */
	const arm_cfft_instance_q15 * q15;
	/** cFFT configuration instance declaration Q31 */
	const arm_cfft_instance_q31 * q31;
	/** cFFT configuration instance declaration F32 */
	const arm_cfft_instance_f32 * f32;
#endif
}cfft_instance;

/** Magnitude buffer declaration */
static union {
	/** q15 Magnitude buffer declaration */
	q15_t q15[MAX_FFT_SIZE];
	/** q31 Magnitude buffer declaration */
	q31_t q31[MAX_FFT_SIZE];
	/* f32 not needed */
}magnitude_buffer;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief FFT main process routine
 *
 *  \param fft_size   size of FFT.
 *  \param data_type  data type.
 *  \return void
 */
void fft_process(uint32_t fft_size, EN_DATA_TYPE data_type)
{
	uint32_t i;
	float32_t timer_factor;
	uint32_t CycleCounterOffset;
	
//	RESET_CYCLE_COUNTER();
    TIMER0_WriteCntReg(M4_TMR01,Tim0_ChannelA,0x0000);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//	CycleCounterOffset = DWT->CYCCNT;
//	CycleCounterOffset -=10;
	
	if(EN_DATA_TYPE_F32 != data_type){
//		TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK3);
		timer_factor = BOARD_MCK/1000/1000/32.0;
	} else {
//		TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK2);
		timer_factor = BOARD_MCK/1000/1000/8.0;
	}

#if (USING_DEPRECATED == 0)
	switch(fft_size){
		case 256:
			cfft_instance.q15 = &arm_cfft_sR_q15_len256;
			cfft_instance.q31 = &arm_cfft_sR_q31_len256;
			cfft_instance.f32 = &arm_cfft_sR_f32_len256;
			break;
		case 1024:
			cfft_instance.q15 = &arm_cfft_sR_q15_len1024;
			cfft_instance.q31 = &arm_cfft_sR_q31_len1024;
			cfft_instance.f32 = &arm_cfft_sR_f32_len1024;
			break;
		case 4096:
			cfft_instance.q15 = &arm_cfft_sR_q15_len4096;
			cfft_instance.q31 = &arm_cfft_sR_q31_len4096;
			cfft_instance.f32 = &arm_cfft_sR_f32_len4096;
			break;
		default:
			break;
	}
#endif

	/* Perform FFT and bin Magnitude calculation */
	if(EN_DATA_TYPE_Q15 == data_type){
			TC_Start();
			RESET_CYCLE_COUNTER();
		arm_float_to_q15(wav_in_buffer, cfft_buffer.q15, fft_size * 2);
#if (USING_DEPRECATED == 1)
		arm_cfft_radix4_init_q15(&cfft_instance.q15, fft_size, 0, 1);
			CycleCounter[0] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[0]);  
			time_fft_us = Get_TMR0CHA_CNT();
		arm_cfft_radix4_q15(&cfft_instance.q15, cfft_buffer.q15);
#else
			CycleCounter[0] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[0]);  
			time_fft_us = Get_TMR0CHA_CNT();
		arm_cfft_q15(cfft_instance.q15, cfft_buffer.q15, 0, 1);
#endif
			CycleCounter[2] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[2]);  
			time_fft_us = Get_TMR0CHA_CNT() - time_fft_us;
		arm_cmplx_mag_q15(cfft_buffer.q15, magnitude_buffer.q15, fft_size);
		arm_q15_to_float(magnitude_buffer.q15, mag_in_buffer, fft_size);
			CycleCounter[1] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[1]);
			TC_Stop();
	}
	else if(EN_DATA_TYPE_Q31 == data_type){
			TC_Start();
			RESET_CYCLE_COUNTER();
		arm_float_to_q31(wav_in_buffer, cfft_buffer.q31, fft_size * 2);
#if (USING_DEPRECATED == 1)
		arm_cfft_radix4_init_q31(&cfft_instance.q31, fft_size, 0, 1);
			CycleCounter[0] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[0]);  
			time_fft_us = Get_TMR0CHA_CNT();
		arm_cfft_radix4_q31(&cfft_instance.q31, cfft_buffer.q31);
#else
			CycleCounter[0] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[0]);  
			time_fft_us = Get_TMR0CHA_CNT();
		arm_cfft_q31(cfft_instance.q31, cfft_buffer.q31, 0, 1);
#endif
			CycleCounter[2] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[2]);  
			time_fft_us = Get_TMR0CHA_CNT() - time_fft_us;
		arm_cmplx_mag_q31(cfft_buffer.q31, magnitude_buffer.q31, fft_size);
		arm_q31_to_float(magnitude_buffer.q31, mag_in_buffer, fft_size);
			CycleCounter[1] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[1]);
			TC_Stop();
	}
	else if(EN_DATA_TYPE_F32 == data_type){
		float32_t *pTmp = (float32_t *)cfft_buffer.q31;
		for (i = 0; i < fft_size * 2; i++)
			pTmp[i] = wav_in_buffer[i];
			TC_Start();
			RESET_CYCLE_COUNTER();
#if (USING_DEPRECATED == 1)
		arm_cfft_radix4_init_f32(&cfft_instance.f32, fft_size, 0, 1);
			CycleCounter[0] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[0]);  
			time_fft_us = Get_TMR0CHA_CNT();
		arm_cfft_radix4_f32(&cfft_instance.f32, pTmp);
#else
			CycleCounter[0] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[0]);  
			time_fft_us = Get_TMR0CHA_CNT();
		arm_cfft_f32(cfft_instance.f32, pTmp, 0, 1);
#endif
			CycleCounter[2] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[2]);  
			time_fft_us = Get_TMR0CHA_CNT() - time_fft_us;
		arm_cmplx_mag_f32(pTmp, mag_in_buffer, fft_size);
			CycleCounter[1] = Get_TMR0CHA_CNT();//GET_CYCLE_COUNTER(CycleCounter[1]);
			TC_Stop();
	}
	time_total_us = (uint32_t)(Get_TMR0CHA_CNT()/ timer_factor);
	time_fft_us /= timer_factor;
	CycleCounter[0] = CycleCounter[2] - CycleCounter[0];
	CycleCounter[1] -= CycleCounterOffset;
}
