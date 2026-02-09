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
#include "memory_map.h"
#include "red_pitaya_identify.h"
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

void* mmap_default(int fd, int address, int size_multiplier)
{
    return mmap(NULL, sysconf(_SC_PAGESIZE)*size_multiplier, PROT_READ | PROT_WRITE, MAP_SHARED, fd, address);
}

#ifdef RP_VARIANT_PHASEMETER
static int is_servo_region(int i)
{
    return i == CFG_SERVOS_0 || i == CFG_SERVOS_1 || i == BRAM_SERVO_0 || i == BRAM_SERVO_1;
}
#endif

void* mmap_all(const char* file_name)
{
    int fd;
    if ((fd = open(file_name, O_RDWR)) < 0) {
        return NULL;
    }
    int model = get_rp_model();
    if (model < 0 || model >= red_pitaya_model_count) {
        model = RP_125_14;
    }

    int** memory_map = malloc(sizeof(int*) * MEMORY_MAP_COUNT);
    if (!memory_map) {
        close(fd);
        errno = ENOMEM;
        return NULL;
    }

    for (int i = 0; i < MEMORY_MAP_COUNT; i++) {
#ifdef RP_VARIANT_PHASEMETER
        if (is_servo_region(i)) {
            memory_map[i] = NULL;
            continue;
        }
#endif
        memory_map[i] = (int*)mmap_default(fd, MEMORY_MAP_ADDRESS[model][i], MEMORY_MAP_SIZE[model][i]);
        if (memory_map[i] == MAP_FAILED) {
            for (int j = 0; j < i; j++) {
                if (memory_map[j] && memory_map[j] != MAP_FAILED) {
                    munmap(memory_map[j], sysconf(_SC_PAGESIZE) * MEMORY_MAP_SIZE[model][j]);
                }
            }
            free(memory_map);
            close(fd);
            return NULL;
        }
    }

    close(fd);
    return memory_map;
}

void reset(void** memory_map)
{
    *(uint32_t*)MMAP_BYTE(memory_map, RD_FLG_SC, 0) = 0;

    for (int i = 0; i < PLL_COUNT; i++) {
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_RST, PLL_ADDRESS_OFFSET[i]) = 0;
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_FREQ_INIT, PLL_ADDRESS_OFFSET[i]) = 0;
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_PLL_P_GAINS, PLL_ADDRESS_OFFSET[i]) = 0;
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_PLL_I_GAINS, PLL_ADDRESS_OFFSET[i]) = 0;
    }

#ifndef RP_VARIANT_PHASEMETER
    for (int i = 0; i < SERVO_SETTINGS_COUNT; i++) {
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_SERVOS_0, SERVO_SETTINGS_ADDRESS_OFFSET[i]) = 0;
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_SERVOS_1, SERVO_SETTINGS_ADDRESS_OFFSET[i]) = 0;
    }
#endif

    *(uint32_t*)MMAP_BYTE(memory_map, CFG_RST, 0) = 1;
}
