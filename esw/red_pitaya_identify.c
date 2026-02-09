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
 
#include "red_pitaya_identify.h"
#include <unistd.h>

enum red_pitaya_model get_rp_model(void)
{
    char soc_id[3] = {0, 0, 0};
    int soc_id_file = open(SOC_IDENTIFIER_FILE, O_RDONLY);
    if (soc_id_file < 0) {
        // Fall back to default model if SOC identifier is unavailable.
        return RP_125_14;
    }

    ssize_t len = read(soc_id_file, soc_id, (ssize_t)(sizeof(soc_id)));
    close(soc_id_file);
    if (len < (ssize_t)sizeof(soc_id)) {
        // Avoid using uninitialized bytes if the file is shorter than expected.
        return RP_125_14;
    }

    switch (soc_id[2]) {
    case '2':
        return RP_125_14;
    case '7':
        return RP_250_12;
    default:
        return RP_125_14;
    }
};
