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

#ifndef SERVER_H
#define SERVER_H

/**
 * \file server.h
 * \brief RedPitaya server API: frame building, commands, PLL logging, and main loop.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* White-noise / averaging: scale and data rate (Hz) for reference freq filter. */
#define MAGIC_RAND_SCALE 200000.0
#define MAGIC_DATA_RATE 200000.0
#define MAGIC_FILTER_FC 100

#define SAMPLING_RATE 125e6
/* DDS/frequency scaling: FPGA register value from Hz (ref freq) and DAC ±1 Vpp. */
#define FREQUENCY_FACTOR_A 34.359738368
#define FREQUENCY_FACTOR_B 1 / (SAMPLING_RATE / (1 << 25)) / (SAMPLING_RATE / (1 << 24) / (1 << 8))
#define dac_cal 2

/** \brief Reference channel state (noise filter, window, frequency). */
typedef struct {
    double freq_now;
    double freq_noise_floor;  /**< White noise floor for reference frequency */
    double freq_noise_corner; /**< Corner frequency (Hz), minimum 1 */
    int freq_index;
    int window_length;
    double freq_ave;
} t_ref;

/** \brief Application configuration: measuring flag and two reference channels. */
typedef struct {
    int start_measuring;
    t_ref ref1;
    t_ref ref2;
} t_config;

/** \brief PLL CSV logger state (file handle). */
typedef struct pll_logger {
    FILE* fd;
} pll_logger;

/**
 * \brief Open a CSV file for PLL state logging.
 * \param log Logger to initialize (unchanged if filename is NULL or open fails).
 * \param filename Path for the CSV file; NULL to leave logger closed.
 */
void pll_logger_init(pll_logger* log, const char* filename);

/**
 * \brief Append one row (timestamp + 6 PLL values) to the logger file.
 * \param log Logger (ignored if NULL or not open).
 * \param memory_map FPGA shared memory (BRAM_PLLS_PIR, BRAM_PLLS_QI).
 */
void pll_logger_write_row(pll_logger* log, void** memory_map);

/**
 * \brief Close the logger file and set fd to NULL.
 * \param log Logger to close (safe if NULL or already closed).
 */
void pll_logger_close(pll_logger* log);

/**
 * \brief Read ADC time series from FPGA, compute FFT, and write result into the frame.
 * \param memory_map FPGA shared memory (BRAM_SCOPE).
 * \param frame Output buffer (FRAME_SIZE doubles); FFT and peak indices are written.
 */
void add_fft_to_frame(void** memory_map, double* frame);

/**
 * \brief Read PLL state from FPGA and write PIR/Q/I into the frame.
 * \param memory_map FPGA shared memory (BRAM_PLLS_PIR, BRAM_PLLS_QI).
 * \param frame Output buffer (FRAME_SIZE doubles); PLL fields are written.
 */
void add_pll_to_frame(void** memory_map, double* frame);

/**
 * \brief Read servo (laser lock) state from FPGA and write piezo/temp/freq err into the frame.
 * \param memory_map FPGA shared memory (BRAM_SERVO_0, BRAM_SERVO_1).
 * \param frame Output buffer (FRAME_SIZE doubles); servo fields are written.
 */
void add_servo_to_frame(void** memory_map, double* frame);

/**
 * \brief Trigger scope read, build one frame, optionally log PLL to CSV, and send frame to client.
 * \param memory_map FPGA shared memory.
 * \param sock_client Client socket (negative to skip send).
 * \param logger Optional PLL CSV logger; NULL to skip logging.
 */
void read_and_send(void** memory_map, int sock_client, pll_logger* logger);

/**
 * \brief Dispatch one binary command to config, PLL, or servo registers.
 * \param memory_map FPGA shared memory (config and BRAM regions).
 * \param command Two words: legacy (addr in high byte, value in low 24) or extended (addr, value).
 * \param config Application config (start_measuring, ref1, ref2); updated for addr 0, 9, 20, 31--34.
 */
void process_command(void** memory_map, uint32_t* command, t_config* config);

/**
 * \brief Create a TCP server on RP_DEFAULT_PORT and start listening.
 * \return Server socket on success, -1 on socket/bind/listen error.
 */
int start_server(void);

/**
 * \brief Add filtered white noise to reference frequencies and write to FPGA servo registers.
 * \param memory_map FPGA shared memory (CFG_SERVOS_0, CFG_SERVOS_1).
 * \param config Config with ref1/ref2 noise and window state; updated in place.
 */
void add_white_noise(void** memory_map, t_config* config);

/**
 * \brief Server entry point.
 * \param argc Argument count.
 * \param argv argv[1] optional phasemeter log file; argv[2] "persist" for non-blocking accept.
 * \return 0 on normal exit, non-zero on error.
 */
int main(int argc, char** argv);

#endif