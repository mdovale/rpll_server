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
#include "rand.h"
#include <math.h>
#include <stdlib.h>

double rand_uniform(void)
{
    int i = rand();
    return ((double)i + 1.0) / ((double)RAND_MAX + 2.0);
}

double rand_gaussian(double floor, double fs)
{
    // floor: white noise floor
    // fs: data rate
    double a = 2.0; // a*sigma is considered as the peak value of noise
    double sigma = floor * sqrt(2.0) * sqrt(fs) / a;
    return 0.0 + sigma * sqrt(-2.0 * log(rand_uniform())) * sin(2.0 * M_PI * rand_uniform());
}

void reset_window_length(double* array, int length)
{
    double* array_tmp;
    array_tmp = realloc(array, sizeof(double) * length);
    array = array_tmp;
    for (int i = 0; i < length; ++i) {
        array[i] = 0.0;
    }
}

void simple_average_filter(double* val, double* array, double rand, int index, int size)
{
    *val -= array[index] / (double)size;
    array[index] = rand;
    *val += array[index] / (double)size;
}
