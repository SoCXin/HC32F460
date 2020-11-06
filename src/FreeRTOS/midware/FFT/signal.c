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
#include "sound.h"
#include "signal.h"
#include "User_TRNG.h"
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

 /** Wave offset in bytes, Normal wave offset should be 44. But the next 4 
		bytes are meaningless. */
#define WAVE_OFFSET                 (44 + 4)

/*----------------------------------------------------------------------------
 *        Imported variables
 *----------------------------------------------------------------------------*/
extern float32_t wav_in_buffer[];

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief get signal of the audio signal
 *
 */
void get_real_time_audio(uint32_t sample_number, uint32_t fft_size)
{
	uint32_t i, j;
	float32_t tmp;
	uint32_t offset_tmp;

	/* Pointer to the wave file. */
	static uint32_t offset = WAVE_OFFSET;

	/* Copy dsp_sfx into wav_in_buffer and prepare Q15 format. */
	for (i = 0, j = 0; i < fft_size * 2; ++j) {
		tmp = (((dsp_sfx[offset] - (float) 128)) / 100);

		// Store Audio sample real part in memory.
		// ADC is 12 bits, meaning that values range from 0 to 4096.
		// -2048 is done to get the actual 0 at the middle of the sinusoidal signal.
		// /200 is the maximum amplitude of the sin from the mic. We divide 
		// here to get
		// in the ]-1;1[ range for Q15 conversion.
		wav_in_buffer[i++] = tmp;

		// Store Audio sample imaginary part in memory.
		wav_in_buffer[i++] = 0;

		// Update the wave file offset pointer.
		if(offset < dsp_sfx_size - 1)
			offset++;
		else
			offset = WAVE_OFFSET;
	}
	if(fft_size < sample_number){
		/* additional samples only for shown at the most right side in the
		wave chart */
		for ( offset_tmp = offset;
			  i < sample_number * 2;
			)
		{
			tmp = (((dsp_sfx[offset_tmp] - (float) 128)) / 100);
			wav_in_buffer[i++] = tmp;
			wav_in_buffer[i++] = 0;
			if(offset_tmp < dsp_sfx_size - 1)
				offset_tmp++;
			else
				offset_tmp = WAVE_OFFSET;
		}
	}
}

/**
 * \brief Generate a sinus input signal.
 *
 * \param freq Sinus frequency.
 */
void generate_sinus_wave(uint32_t sample_number, float32_t freq)
{
	uint32_t i;
	uint32_t y;
	float32_t ratio;

	ratio = freq / SAMPLING_FREQUENCY;

	for (i = 0, y = 0; i < sample_number; i++) {
		/* Amplification is set to 1. */
		wav_in_buffer[y++] = arm_sin_f32(2 * PI * i * ratio);
		wav_in_buffer[y++] = 0;
	}
}

/**
 * \brief Generate a square wave signal.
 *
 * \param freq       frequency.
 * \param duty_ratio duty ratio.
 */
void generate_square_wave(uint32_t sample_number, float32_t freq, 
		float32_t duty_ratio)
{
	/* i / Fs = integer * 1/f + fraction
		 Fs - Sampling frequency
		 f  - frequency of square wave
	*/
	uint32_t i;
	uint32_t y;
	float32_t period_sampling = 1.0/SAMPLING_FREQUENCY;
	float32_t period_signal = (0 == (uint32_t)freq) ? 1.0 : (1.0/freq);
	float32_t period_low = period_signal * duty_ratio;
	float32_t tmp_sample = 0;   /* i / Fs */
	float32_t tmp_wave = 0;     /* integer * 1/f */
	float32_t fraction;

	for (i = 0, y = 0; i < sample_number; i++) {
		if(tmp_sample - tmp_wave - period_signal > 0){
			tmp_wave += period_signal;
		}
		fraction = tmp_sample - tmp_wave;
		tmp_sample += period_sampling;
		if ( fraction < period_low )
			wav_in_buffer[y++] = -1;
		else
			wav_in_buffer[y++] = 1;
		wav_in_buffer[y++] = 0;
	}
}

/**
 * \brief Generate a triangle wave signal.
 *
 * \param freq       frequency.
 * \param duty_ratio duty ratio.
 */
void generate_triangle_wave(uint32_t sample_number, float32_t freq,
		float32_t duty_ratio)
{
	uint32_t i;
	uint32_t y;
	float32_t period_sampling = 1.0/SAMPLING_FREQUENCY;
	float32_t period_signal = (0 == (uint32_t)freq) ? 1.0 : (1.0/freq);
	float32_t period_raise = period_signal * duty_ratio;
	float32_t slope_raise = 2.0 / period_raise;
	float32_t slope_descend = 2.0 / (period_signal - period_raise);
	float32_t tmp_sample = 0;   /* i / Fs */
	float32_t tmp_wave = 0;     /* integer * 1/f */
	float32_t fraction;

	for (i = 0, y = 0; i < sample_number; i++) {
		if(tmp_sample - tmp_wave - period_signal > 0){
			tmp_wave += period_signal;
		}
		fraction = tmp_sample - tmp_wave;
		tmp_sample += period_sampling;
		if ( fraction < period_raise )
			wav_in_buffer[y++] = -1.0 + fraction * slope_raise;
		else{
			wav_in_buffer[y++] = 1.0 - (fraction - period_raise) * slope_descend;
		}
		wav_in_buffer[y++] = 0;
	}
}

/**
 * \brief Generate a white noise signal.
 *
 * \param sample_number  number of samples.
 */
void generate_white_noise(uint32_t sample_number)
{
	uint32_t i;
	uint32_t y;
	uint32_t random;
	float32_t tmp;

	for (i = 0, y = 0; i < sample_number/4; i++) {
		/* the RNG provides one 32-bit value every 84 clock cycles,
		 * use one random number to get 4 samples once  */
		random = TRNG_GetRandData();
		tmp = random & 0xFF;
		wav_in_buffer[y++] = 1.0 - 2.0 * tmp / 256.0;
		wav_in_buffer[y++] = 0;

		random >>= 8;
		tmp = random & 0xFF;
		wav_in_buffer[y++] = 1.0 - 2.0 * tmp / 256.0;
		wav_in_buffer[y++] = 0;

		random >>= 8;
		tmp = random & 0xFF;
		wav_in_buffer[y++] = 1.0 - 2.0 * tmp / 256.0;
		wav_in_buffer[y++] = 0;

		random >>= 8;
		tmp = random & 0xFF;
		wav_in_buffer[y++] = 1.0 - 2.0 * tmp / 256.0;
		wav_in_buffer[y++] = 0;
	}
}

/**
 * \brief Generate a pink noise signal.
 *
 * \param sample_number  number of samples.
 */
void generate_pink_noise(uint32_t sample_number)
{
	uint32_t j;
	uint32_t y;
	int i;

	int32_t max_key;
	int32_t key;
	uint32_t white_values[5];
	uint32_t range = 128;

	max_key = 0x1f; // Five bits set
	key = 0;
	for (i = 0; i < 5; i++)
		white_values[i] = TRNG_GetRandData() % (range/5);

	for (j = 0, y = 0; j < sample_number; j++) {
		int32_t last_key = key;
		uint32_t sum;

		key++;
		if (key > max_key)
			key = 0;
		// Exclusive-Or previous value with current value. This gives
		// a list of bits that have changed.
		int32_t diff = last_key ^ key;
		sum = 0;
		for (i = 0; i < 5; i++){
			// If bit changed get new random number for corresponding
			// white_value
			if (diff & (1 << i))
				white_values[i] = TRNG_GetRandData() % (range/5);
			sum += white_values[i];
		}
		wav_in_buffer[y++] = 1.0 - 2.0 * sum / 128.0;
		wav_in_buffer[y++] = 0;
	}
}
