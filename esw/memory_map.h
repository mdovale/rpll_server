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
#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

#define SHARED_MEMORY_FILE "/dev/mem"
#include "red_pitaya_identify.h"
#include "fft_peak.h"

/* Byte-offset helper: memory_map[i] + N adds N*sizeof(int) bytes. Use this for register offsets. */
#define MMAP_BYTE(m, idx, off) ((void*)((char*)(m)[idx] + (off)))

/**
 * Map shared memory with some default settings. See mmap
 * @param fd The file descriptor
 * @param address The relative memory address in <fd>
 * @return The mapped shared memory
 */
void* mmap_default(int fd, int address, int multiplier);

/**
 * Map the application specific memory addresses of the automatically identified RedPitaya model.
 * The returned pointers are used by both the CPU and the FPGA application and are used for communication between them.
 * @param file_name The file name of the shared memory
 * @return An array of pointers with indexes as defined in the enum <memory_map>.
 */
void* mmap_all(const char* name);

/**
 * Send a RESET to the FPGA.
 * @param memory_map The memory shared between FPGA and CPU.
 */
void reset(void** memory_map);

/**
 * Used to index the mapped memory
 */
enum memory_map {
    CFG_RST,
    CFG_FREQ_INIT,
    CFG_PLL_P_GAINS,
    CFG_PLL_I_GAINS,
    CFG_SERVOS_0,
    CFG_SERVOS_1,
    BRAM_PLLS_PIR,
    BRAM_PLLS_QI,
    BRAM_SCOPE,
    BRAM_SERVO_0,
    BRAM_SERVO_1,
    RD_FLG_SC,
    MEMORY_MAP_COUNT
};

/**
 * All RedPitaya specific addresses
 */
static const int MEMORY_MAP_ADDRESS[][MEMORY_MAP_COUNT] = {
    [RP_125_14] = {
        [CFG_RST] = 0x42000000,
        [CFG_FREQ_INIT] = 0x41200000,
        [CFG_PLL_P_GAINS] = 0x41210000,
        [CFG_PLL_I_GAINS] = 0x41230000,
        [CFG_SERVOS_0] = 0x60000000,
        [CFG_SERVOS_1] = 0x50000000,
        [BRAM_PLLS_PIR] = 0x40000000,
        [BRAM_PLLS_QI] = 0x44000000,
        [BRAM_SCOPE] = 0x4C000000,
        [BRAM_SERVO_0] = 0x46000000,
        [BRAM_SERVO_1] = 0x48000000,
        [RD_FLG_SC] = 0x41220000  
    },
    [RP_250_12] = {
        [CFG_RST] = 0x82000000,
        [CFG_FREQ_INIT] = 0x81200000,
        [CFG_PLL_P_GAINS] = 0x81210000,
        [CFG_PLL_I_GAINS] = 0x81230000,
        [CFG_SERVOS_0] = 0xA0000000,
        [CFG_SERVOS_1] = 0x90000000,
        [BRAM_PLLS_PIR] = 0x80000000,
        [BRAM_PLLS_QI] = 0x84000000,
        [BRAM_SCOPE] = 0x8C000000,
        [BRAM_SERVO_0] = 0x86000000,
        [BRAM_SERVO_1] = 0x88000000,
        [RD_FLG_SC] = 0x81220000
    }
};

static const int MEMORY_MAP_SIZE[][MEMORY_MAP_COUNT] = {
    [RP_125_14] = {
        [CFG_RST] = 1,
        [CFG_FREQ_INIT] = 1,
        [CFG_PLL_P_GAINS] = 1,
        [CFG_PLL_I_GAINS] = 1,
        [CFG_SERVOS_0] = 1,
        [CFG_SERVOS_1] = 1,
        [BRAM_PLLS_PIR] = 1,
        [BRAM_PLLS_QI] = 1,
        [BRAM_SCOPE] = 1,
        [BRAM_SERVO_0] = 1,
        [BRAM_SERVO_1] = 1,
        [RD_FLG_SC] = 1
    },
    [RP_250_12] = {
        [CFG_RST] = 1,
        [CFG_FREQ_INIT] = 1,
        [CFG_PLL_P_GAINS] = 1,
        [CFG_PLL_I_GAINS] = 1,
        [CFG_SERVOS_0] = 1,
        [CFG_SERVOS_1] = 1,
        [BRAM_PLLS_PIR] = 1,
        [BRAM_PLLS_QI] = 1,
        [BRAM_SCOPE] = 1,
        [BRAM_SERVO_0] = 1,
        [BRAM_SERVO_1] = 1,
        [RD_FLG_SC] = 1
    }
};

enum servo_setting {
    FREQ_REF_LOOP,
    PIEZO_SWITCH,
    TEMP_SWITCH,
    PIEZO_SIGN,
    TEMP_SIGN,
    PIEZO_OFFSET,
    TEMP_OFFSET,
    PIEZO_GAIN_I,
    PIEZO_GAIN_II,
    TEMP_GAIN_P,
    TEMP_GAIN_I,
    SERVO_SETTINGS_COUNT
};

enum frame_content{
  FRAME_COUNTER,
  FFT_RESULT_CHAN1_START,
  FFT_RESULT_CHAN2_START,
  PLL0PIR,
  PLL1PIR,
  PLL0Q,
  PLL1Q,  
  PLL0I,
  PLL1I,    
  PIEZO_ACT0,
  PIEZO_ACT1,  
  TEMP_ACT0,
  TEMP_ACT1,  
  FREQ_ERR0,
  FREQ_ERR1,
  MAX_ABS_FREQ0,
  MAX_ABS_FREQ1,
  FRAME_CONTENT_COUNT 
};

/**
 * Offsets inside the mapped memory used for the pdh functionality
 */
 
 static const int FRAME_CONTENT_ADDRESS_OFFSET[FRAME_CONTENT_COUNT] = {
     [FRAME_COUNTER] = 0,
     [FFT_RESULT_CHAN1_START] = 1,
     [FFT_RESULT_CHAN2_START] = 1 + FFT_SIZE,
     [PLL0PIR] = 2 * FFT_SIZE + 2,
     [PLL1PIR] = 2 * FFT_SIZE + 3,
     [PLL0Q] = 2 * FFT_SIZE + 4,
     [PLL1Q] = 2 * FFT_SIZE + 5,
     [PLL0I] = 2 * FFT_SIZE + 6,
     [PLL1I] = 2 * FFT_SIZE + 7,
     [PIEZO_ACT0] = 2 * FFT_SIZE + 8,
     [PIEZO_ACT1] = 2 * FFT_SIZE + 9,
     [TEMP_ACT0] = 2 * FFT_SIZE + 10,
     [TEMP_ACT1] = 2 * FFT_SIZE + 11,
     [FREQ_ERR0] = 2 * FFT_SIZE + 12,
     [FREQ_ERR1] = 2 * FFT_SIZE + 13,
     [MAX_ABS_FREQ0] = 2 * FFT_SIZE + 14,
     [MAX_ABS_FREQ1] = 2 * FFT_SIZE + 15
 };
 
 // frane elements are double values with 8 bytes
#define FRAME_SIZE_BYTES  (2 * FFT_SIZE + 16) * 8
#define FRAME_SIZE (2 * FFT_SIZE + 16)

/**
 * Offsets inside the mapped memory used for the servo functionality
 */
static const int SERVO_SETTINGS_ADDRESS_OFFSET[SERVO_SETTINGS_COUNT] = {
    [FREQ_REF_LOOP] = 0,
    [PIEZO_SWITCH] = 4,
    [TEMP_SWITCH] = 8,
    [PIEZO_SIGN] = 12,
    [TEMP_SIGN] = 16,
    [PIEZO_OFFSET] = 20,
    [TEMP_OFFSET] = 24,
    [PIEZO_GAIN_I] = 28,
    [PIEZO_GAIN_II] = 32,
    [TEMP_GAIN_P] = 36,
    [TEMP_GAIN_I] = 40
};

enum pll {
    PLL0,
    PLL1,
    PLL_COUNT
};

/* AXI GPIO dual-channel register map (Xilinx xgpio_l.h): 0x0=Data1, 0x4=Tri1, 0x8=Data2, 0xC=Tri2. */
static const int PLL_ADDRESS_OFFSET[PLL_COUNT] = {
    [PLL0] = 0,
    [PLL1] = 8
};

#endif