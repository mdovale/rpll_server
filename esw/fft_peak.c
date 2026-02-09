/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Miguel Dovale (University of Arizona)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fft_peak.h"
#include <math.h>
#include <stdint.h>

int argmax(double* data, int length)
{
    double current_max = data[0];
    int current_index = 0;
    for (int i = 1; i < length; i++) {
        if (data[i] > current_max) {
            current_max = data[i];
            current_index = i;
        }
    }
    return current_index;
}

/**
 * Run real FFT, fill magnitude array (FFT_SIZE), return relative peak frequency.
 * Real FFT yields REAL_FFT_NBINS (513) complex bins; we fill abs[0..512] only.
 */
static double do_fft(float* tbuf, double* abs)
{
    real_fft_cpx_t kout[REAL_FFT_NBINS];

    real_fft_1024_forward(tbuf, kout);

    for (int i = 0; i < REAL_FFT_NBINS; i++) {
        abs[i] = sqrt((double)(kout[i].r * kout[i].r + kout[i].i * kout[i].i)) / (double)FFT_SIZE;
    }
    abs[0] = 0.0;

    /*
     * Peak bin index k corresponds to frequency f = k * Fs / N, where N is the
     * real FFT length (REAL_FFT_SIZE=1024). FFT_SIZE here is the *number of bins*
     * (REAL_FFT_NBINS=513), so dividing by FFT_SIZE would scale frequencies ~2x.
     */
    const int k = argmax(abs, FFT_SIZE);
    return ((double)k) / (double)REAL_FFT_SIZE;
}

void calc_fft_peak(void* memory, double sampling_rate, double* abs_fft, double* peak)
{
    static float tbuf[REAL_FFT_SIZE];
    static double abs[FFT_SIZE];

    for (int i = 0; i < REAL_FFT_SIZE; i++) {
        tbuf[i] = (float)*((int16_t*)((char*)memory + FFT_MEMORY_STEP * i));
    }

    *peak = sampling_rate * do_fft(tbuf, abs);

    for (int i = 0; i < FFT_SIZE; i++) {
        abs_fft[i] = abs[i];
    }
}
