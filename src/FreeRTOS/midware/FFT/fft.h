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

#ifndef _FFT_H_
#define _FFT_H_

#include "arm_math.h"

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/* Numbers Definitions related to the FFT calculation */
#define MAX_FFT_SIZE            4096UL

#define FFT_SIZE_4096           4096
#define FFT_SIZE_1024           1024
#define FFT_SIZE_256            256

typedef enum EN_SIGNAL_TYPE {
	EN_SIGNAL_SINUS = 0,
	EN_SIGNAL_SOUND = 1,
	EN_SIGNAL_SQUARE,
	EN_SIGNAL_TRIANGLE,
	EN_SIGNAL_WHITE_NOISE,
	EN_SIGNAL_PINK_NOISE,
	EN_SIGNAL_BUTT,
}EN_SIGNAL_TYPE;

typedef enum EN_DATA_TYPE{
	EN_DATA_TYPE_Q15 = 0,
	EN_DATA_TYPE_Q31 = 1,
	EN_DATA_TYPE_F32 = 2,
	EN_DATA_TYPE_BUTT,
}EN_DATA_TYPE;


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
extern void fft_process(uint32_t fft_size, EN_DATA_TYPE data_type);

#endif // _FFT_H_

