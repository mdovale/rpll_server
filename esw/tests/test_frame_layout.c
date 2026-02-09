/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Miguel Dovale (University of Arizona)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Verify frame layout constants match client expectations.
 * Must stay in sync with client/frame_schema.py and server/esw/memory_map.h.
 */
#include "../memory_map.h"
#include "../fft_peak.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    /* FFT_SIZE must match client and real_fft_1024.h */
    assert(FFT_SIZE == 513);

    /* Frame layout: counter, 2*FFT ch1, 2*FFT ch2, 16 tail doubles */
    assert(FRAME_SIZE == (2 * FFT_SIZE + 16));
    assert(FRAME_SIZE_BYTES == (2 * FFT_SIZE + 16) * 8);

    /* Offsets must match client frame_schema.py */
    assert(FRAME_CONTENT_ADDRESS_OFFSET[FRAME_COUNTER] == 0);
    assert(FRAME_CONTENT_ADDRESS_OFFSET[FFT_RESULT_CHAN1_START] == 1);
    assert(FRAME_CONTENT_ADDRESS_OFFSET[FFT_RESULT_CHAN2_START] == 1 + FFT_SIZE);
    assert(FRAME_CONTENT_ADDRESS_OFFSET[PLL0PIR] == 2 * FFT_SIZE + 2);
    assert(FRAME_CONTENT_ADDRESS_OFFSET[PLL1PIR] == 2 * FFT_SIZE + 3);
    assert(FRAME_CONTENT_ADDRESS_OFFSET[MAX_ABS_FREQ0] == 2 * FFT_SIZE + 14);
    assert(FRAME_CONTENT_ADDRESS_OFFSET[MAX_ABS_FREQ1] == 2 * FFT_SIZE + 15);

    printf("test_frame_layout: all checks passed\n");
    return 0;
}
