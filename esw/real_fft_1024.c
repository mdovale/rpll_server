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
 *
 * Real FFT 1024: forward only, float. Inspired by KISS FFT (real FFT via
 * two-real packed complex FFT and super-twiddle unpack).
 */

#include "real_fft_1024.h"
#include <math.h>
#include <string.h>

#define CFFT_N 512
#define PI 3.141592653589793238462643383279502884197169399375105820974944

/* Bit-reverse index for 9 bits (0..511). */
static unsigned bitrev9(unsigned i)
{
    unsigned r = 0;
    for (int b = 0; b < 9; b++) {
        r = (r << 1) | (i & 1u);
        i >>= 1;
    }
    return r;
}

/* Twiddles for 512-point complex FFT: W_512^k = exp(-2*pi*i*k/512), k=0..255. */
static void init_twiddles(real_fft_cpx_t *tw)
{
    for (int k = 0; k <= CFFT_N / 2; k++) {
        double phase = -2.0 * PI * (double)k / (double)CFFT_N;
        tw[k].r = (float)cos(phase);
        tw[k].i = (float)sin(phase);
    }
}

/* Radix-2 decimation-in-time complex FFT, N=512, forward. Out-of-place. */
static void cfft_512_forward(const real_fft_cpx_t *in, real_fft_cpx_t *out,
                             const real_fft_cpx_t *twiddles)
{
    real_fft_cpx_t tmp[CFFT_N];
    /* Bit-reverse copy into tmp. */
    for (int i = 0; i < CFFT_N; i++) {
        unsigned j = bitrev9((unsigned)i);
        tmp[j].r = in[i].r;
        tmp[j].i = in[i].i;
    }
    /* Radix-2 stages: L = 2, 4, 8, ..., 512. */
    for (int L = 2; L <= CFFT_N; L *= 2) {
        int half = L / 2;
        int step = CFFT_N / L;
        for (int b = 0; b < CFFT_N; b += L) {
            for (int j = 0; j < half; j++) {
                int k = j * step;
                float wr = twiddles[k].r;
                float wi = twiddles[k].i;
                float ar = tmp[b + j].r;
                float ai = tmp[b + j].i;
                float br = tmp[b + j + half].r;
                float bi = tmp[b + j + half].i;
                float tr = br * wr - bi * wi;
                float ti = br * wi + bi * wr;
                tmp[b + j].r = ar + tr;
                tmp[b + j].i = ai + ti;
                tmp[b + j + half].r = ar - tr;
                tmp[b + j + half].i = ai - ti;
            }
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

/* Super-twiddles for real unpack: exp(-j*pi*((k+1)/512 + 0.5)), k=0..255. */
static void init_super_twiddles(real_fft_cpx_t *tw)
{
    for (int k = 0; k < CFFT_N / 2; k++) {
        double phase = -PI * ((double)(k + 1) / (double)CFFT_N + 0.5);
        tw[k].r = (float)cos(phase);
        tw[k].i = (float)sin(phase);
    }
}

void real_fft_1024_forward(const float *timedata, real_fft_cpx_t *freqdata)
{
    static int init = 0;
    static real_fft_cpx_t twiddles[CFFT_N / 2 + 1];
    static real_fft_cpx_t super_twiddles[CFFT_N / 2];
    real_fft_cpx_t packed[CFFT_N];
    real_fft_cpx_t tmpbuf[CFFT_N];

    if (!init) {
        init = 1;
        init_twiddles(twiddles);
        init_super_twiddles(super_twiddles);
    }

    /* Pack even/odd reals into complex: z[n] = timedata[2n] + i*timedata[2n+1]. */
    for (int n = 0; n < CFFT_N; n++) {
        packed[n].r = timedata[2 * n];
        packed[n].i = timedata[2 * n + 1];
    }

    cfft_512_forward(packed, tmpbuf, twiddles);

    /* Unpack to real spectrum (KISS-style): DC and Nyquist real; rest from symmetry. */
    {
        float tdc_r = tmpbuf[0].r;
        float tdc_i = tmpbuf[0].i;
        freqdata[0].r = (tdc_r + tdc_i) * 0.5f;
        freqdata[0].i = 0.0f;
        freqdata[CFFT_N].r = (tdc_r - tdc_i) * 0.5f;
        freqdata[CFFT_N].i = 0.0f;
    }
    for (int k = 1; k <= CFFT_N / 2; k++) {
        float fpk_r = tmpbuf[k].r;
        float fpk_i = tmpbuf[k].i;
        float fpnk_r = tmpbuf[CFFT_N - k].r;
        float fpnk_i = -tmpbuf[CFFT_N - k].i;
        float f1k_r = (fpk_r + fpnk_r) * 0.5f;
        float f1k_i = (fpk_i + fpnk_i) * 0.5f;
        float f2k_r = (fpk_r - fpnk_r) * 0.5f;
        float f2k_i = (fpk_i - fpnk_i) * 0.5f;
        float tw_r = super_twiddles[k - 1].r;
        float tw_i = super_twiddles[k - 1].i;
        float tw_re = f2k_r * tw_r - f2k_i * tw_i;
        float tw_im = f2k_r * tw_i + f2k_i * tw_r;
        freqdata[k].r = (f1k_r + tw_re) * 0.5f;
        freqdata[k].i = (f1k_i + tw_im) * 0.5f;
        freqdata[CFFT_N - k].r = (f1k_r - tw_re) * 0.5f;
        freqdata[CFFT_N - k].i = (tw_im - f1k_i) * 0.5f;
    }
}
