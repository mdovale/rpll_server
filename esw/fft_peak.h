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
#ifndef FFT_PEAK_H
#define FFT_PEAK_H

#include "real_fft_1024.h"

#define FFT_MEMORY_STEP 4
#define FFT_SIZE REAL_FFT_NBINS

/**
 * Perform FFT on data in an application specific format optimized for speed on repetitive calls.
 * The memory format of the input data is assumed to be 16 bit signed integers every
 * <FFT_MEMORY_STEP> bytes. This allows for interleaved data from multiple time series channels.
 * @param memory The memory region containing the time space input data.
 * @param sampling_rate Sampling rate (used to calculate peak frequency)
 * @param abs_fft The frequency space FFT amplitude output (FFT_SIZE values)
 * @param peak The peak frequency
 */
void calc_fft_peak(void* memory, double sampling_rate, double* abs_fft, double* peak);

/**
 * Find index of the maximum
 * @param data Input
 * @param length Length of data
 * @return Index of the maximum
 */
int argmax(double* data, int length);

#endif
