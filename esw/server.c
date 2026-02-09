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

#include "server.h"
#include "cmd_parse.h"
#include "rp_protocol.h"
#include "fft_peak.h"
#include "memory_map.h"
#include "rand.h"

#ifdef RP_VARIANT_PHASEMETER
#define RP_SERVER_VARIANT RP_CAP_PHASEMETER
#else
#define RP_SERVER_VARIANT RP_CAP_LASER_LOCK
#endif
#include <arpa/inet.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define SERVER_LISTEN_BACKLOG 1024
#define SCOPE_READ_USLEEP_US 5000
#define CMD_BUF_SIZE 8


void add_fft_to_frame(void** memory_map, double* frame)
{
    calc_fft_peak(memory_map[BRAM_SCOPE], SAMPLING_RATE, &frame[FRAME_CONTENT_ADDRESS_OFFSET[FFT_RESULT_CHAN1_START]], &frame[FRAME_CONTENT_ADDRESS_OFFSET[MAX_ABS_FREQ0]]);
    calc_fft_peak(memory_map[BRAM_SCOPE] + FFT_MEMORY_STEP/2, SAMPLING_RATE, &frame[FRAME_CONTENT_ADDRESS_OFFSET[FFT_RESULT_CHAN2_START]], &frame[FRAME_CONTENT_ADDRESS_OFFSET[MAX_ABS_FREQ1]]);
}

/**
 * Read PLL state from FPGA shared memory and scale to physical units.
 * out[0]=PLL0 PIR (Hz), out[1]=PLL1 PIR (Hz), out[2]=PLL0 Q, out[3]=PLL1 Q, out[4]=PLL0 I, out[5]=PLL1 I.
 */
static void read_pll_from_fpga(void** memory_map, double out[6])
{
    /* PIR is signed 64-bit fixed-point; uint64_t would misinterpret negatives as ~125 MHz.
     * Read both PIRs in one 16-byte memcpy for coherence (FPGA writes 128 bits atomically). */
    int64_t raw0, raw8;
    if (memory_map[BRAM_PLLS_PIR]) {
        int64_t buf[2];
        memcpy(buf, memory_map[BRAM_PLLS_PIR], 16);
        raw0 = buf[0];
        raw8 = buf[1];
    } else {
        raw0 = 0;
        raw8 = 0;
    }
    out[0] = (double)raw0 / pow(2.0, 64.0) * SAMPLING_RATE;
    out[1] = (double)raw8 / pow(2.0, 64.0) * SAMPLING_RATE;
    out[2] = *(int32_t*)MMAP_BYTE(memory_map, BRAM_PLLS_QI, 0) / pow(2.0, 32.0);
    out[3] = *(int32_t*)MMAP_BYTE(memory_map, BRAM_PLLS_QI, 8) / pow(2.0, 32.0);
    out[4] = *(int32_t*)MMAP_BYTE(memory_map, BRAM_PLLS_QI, 4) / pow(2.0, 32.0);
    out[5] = *(int32_t*)MMAP_BYTE(memory_map, BRAM_PLLS_QI, 12) / pow(2.0, 32.0);
}

void add_pll_to_frame(void** memory_map, double* frame)
{
    double pll[6];
    read_pll_from_fpga(memory_map, pll);
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PLL0PIR]] = pll[0];
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PLL1PIR]] = pll[1];
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PLL0Q]] = pll[2];
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PLL1Q]] = pll[3];
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PLL0I]] = pll[4];
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PLL1I]] = pll[5];
}

void add_servo_to_frame(void** memory_map, double* frame)
{
#ifdef RP_VARIANT_PHASEMETER
    /* Phasemeter: send PIR (frequency in Hz) in FREQ_ERR slots; no servo data. PIR is signed. */
    double pir0, pir1;
    if (memory_map[BRAM_PLLS_PIR]) {
        int64_t buf[2];
        memcpy(buf, memory_map[BRAM_PLLS_PIR], 16);
        pir0 = (double)buf[0] / pow(2.0, 64.0) * SAMPLING_RATE;
        pir1 = (double)buf[1] / pow(2.0, 64.0) * SAMPLING_RATE;
    } else {
        pir0 = pir1 = 0.0;
    }
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PIEZO_ACT0]] = 0.0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PIEZO_ACT1]] = 0.0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[TEMP_ACT0]] = 0.0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[TEMP_ACT1]] = 0.0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[FREQ_ERR0]] = pir0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[FREQ_ERR1]] = pir1;
#else
    double freq_err_0, freq_err_1, piezo_act_0, piezo_act_1, temp_act_0, temp_act_1;

    piezo_act_0 = *(int32_t*)MMAP_BYTE(memory_map, BRAM_SERVO_0, 0) / pow(2.0, 32.0) * dac_cal;
    temp_act_0 = *(int32_t*)MMAP_BYTE(memory_map, BRAM_SERVO_0, 4) / pow(2.0, 32.0) * dac_cal;
    freq_err_0 = *(int64_t*)MMAP_BYTE(memory_map, BRAM_SERVO_0, 8) / pow(2.0, 64.0) * SAMPLING_RATE;
    piezo_act_1 = *(int32_t*)MMAP_BYTE(memory_map, BRAM_SERVO_1, 0) / pow(2.0, 32.0) * dac_cal;
    temp_act_1 = *(int32_t*)MMAP_BYTE(memory_map, BRAM_SERVO_1, 4) / pow(2.0, 32.0) * dac_cal;
    freq_err_1 = *(int64_t*)MMAP_BYTE(memory_map, BRAM_SERVO_1, 8) / pow(2.0, 64.0) * SAMPLING_RATE;

    frame[FRAME_CONTENT_ADDRESS_OFFSET[PIEZO_ACT0]] = (double)piezo_act_0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[PIEZO_ACT1]] = (double)piezo_act_1;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[TEMP_ACT0]] = (double)temp_act_0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[TEMP_ACT1]] = (double)temp_act_1;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[FREQ_ERR0]] = (double)freq_err_0;
    frame[FRAME_CONTENT_ADDRESS_OFFSET[FREQ_ERR1]] = (double)freq_err_1;
#endif
}

void pll_logger_init(pll_logger* log, const char* filename)
{
    log->fd = NULL;
    if (filename) {
        log->fd = fopen(filename, "w");
        if (log->fd)
            setlinebuf(log->fd);
    }
}

void pll_logger_write_row(pll_logger* log, void** memory_map)
{
    double pll[6];
    if (!log || !log->fd)
        return;
    read_pll_from_fpga(memory_map, pll);
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    fprintf(log->fd, "%ld.%09ld %.17e %.17e %.17e %.17e %.17e %.17e\n", spec.tv_sec, spec.tv_nsec, pll[0], pll[2], pll[4], pll[1], pll[3], pll[5]);
}

void pll_logger_close(pll_logger* log)
{
    if (log && log->fd) {
        fclose(log->fd);
        log->fd = NULL;
    }
}

static void trigger_scope_read(void** memory_map)
{
    *(uint32_t*)MMAP_BYTE(memory_map, RD_FLG_SC, 8) = 1;
    usleep(SCOPE_READ_USLEEP_US);
    *(uint32_t*)MMAP_BYTE(memory_map, RD_FLG_SC, 8) = 0;
}

/**
 * Build one frame from FPGA state into frame[FRAME_SIZE]. Optionally append PLL row to logger.
 */
static void build_frame(void** memory_map, double* frame, pll_logger* logger)
{
    static int count = 0;
    frame[0] = (double)(++count);
    add_fft_to_frame(memory_map, frame);
    add_servo_to_frame(memory_map, frame);
    add_pll_to_frame(memory_map, frame);
    pll_logger_write_row(logger, memory_map);
}

void read_and_send(void** memory_map, int sock_client, pll_logger* logger)
{
    static double frame[FRAME_SIZE];
    trigger_scope_read(memory_map);
    build_frame(memory_map, frame, logger);
    if (sock_client >= 0) {
        send(sock_client, &frame[0], FRAME_SIZE_BYTES, MSG_NOSIGNAL);
    }
}

void process_command(void** memory_map, uint32_t* commandTwo, t_config* config)
{
    //printf("Packet received: %i %i\n", commandTwo[0], commandTwo[1]);
    int value = commandTwo[1]; //separate command-value from command-addr
    int adr = (commandTwo[0] & 0xff000000) >> 24; //separate command-value from command-addr
    //printf("addr: %i ---  valueA B: %i %i\n\n", adr, value, valueB);
    switch (adr) {
    case 0:
        config->start_measuring = value;
        break;
    case 1:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_RST, PLL_ADDRESS_OFFSET[PLL0]) = value;
        break;
    case 2:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_RST, PLL_ADDRESS_OFFSET[PLL1]) = value;
        break;
    case 3:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_FREQ_INIT, PLL_ADDRESS_OFFSET[PLL0]) = ((uint32_t)(value) << 20);
        break;
    case 4:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_FREQ_INIT, PLL_ADDRESS_OFFSET[PLL1]) = ((uint32_t)(value) << 20);
        break;
    case 5:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_PLL_P_GAINS, PLL_ADDRESS_OFFSET[PLL0]) = value;
        break;
    case 6:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_PLL_P_GAINS, PLL_ADDRESS_OFFSET[PLL1]) = value;
        break;
    case 7:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_PLL_I_GAINS, PLL_ADDRESS_OFFSET[PLL0]) = value;
        break;
    case 8:
        *(uint32_t*)MMAP_BYTE(memory_map, CFG_PLL_I_GAINS, PLL_ADDRESS_OFFSET[PLL1]) = value;
        break;
    case 9: // reference frequency
#ifdef RP_VARIANT_PHASEMETER
        (void)memory_map; /* servo registers not mapped */
        config->ref1.freq_now = value / 0.268435456;
        break;
#else
        config->ref1.freq_now = value / 0.268435456;
        value = ((uint32_t)(value * FREQUENCY_FACTOR_B));
        *((uint32_t*)(memory_map[CFG_SERVOS_0] + SERVO_SETTINGS_ADDRESS_OFFSET[FREQ_REF_LOOP])) = value;
        break;
#endif
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
#ifdef RP_VARIANT_PHASEMETER
        break;
#else
        *((uint32_t*)(memory_map[CFG_SERVOS_0] + SERVO_SETTINGS_ADDRESS_OFFSET[adr - 9])) = value;
        break;
#endif
    case 20: // reference frequency
#ifdef RP_VARIANT_PHASEMETER
        (void)memory_map;
        config->ref2.freq_now = value / 0.268435456;
        break;
#else
        config->ref2.freq_now = value / 0.268435456;
        value = ((uint32_t)(value * FREQUENCY_FACTOR_B));
        *((uint32_t*)(memory_map[CFG_SERVOS_1] + SERVO_SETTINGS_ADDRESS_OFFSET[FREQ_REF_LOOP])) = value;
        break;
#endif
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
#ifdef RP_VARIANT_PHASEMETER
        break;
#else
        *((uint32_t*)(memory_map[CFG_SERVOS_1] + SERVO_SETTINGS_ADDRESS_OFFSET[adr - 20])) = value;
        break;
#endif
    case 31:
        config->ref1.freq_noise_floor = value; // white noise floor for the reference frequency
        break;
    case 32:
        config->ref2.freq_noise_floor = value; // white noise floor for the reference frequency
        break;
    case 33:
        // White noise corner freq. for the reference frequency (Hz).
        // Guard against invalid/zero values that can lead to huge/invalid window lengths and crashes.
        if (value < 1) {
            value = 1;
        }
        config->ref1.freq_noise_corner = value;
        config->ref1.window_length = ceil(0.443 * MAGIC_DATA_RATE / config->ref1.freq_noise_corner);
        if (config->ref1.window_length < 1) {
            config->ref1.window_length = 1;
        }
        config->ref1.freq_index %= config->ref1.window_length;
        break;
    case 34:
        if (value < 1) {
            value = 1;
        }
        config->ref2.freq_noise_corner = value;
        config->ref2.window_length = ceil(0.443 * MAGIC_DATA_RATE / config->ref2.freq_noise_corner);
        if (config->ref2.window_length < 1) {
            config->ref2.window_length = 1;
        }
        config->ref2.freq_index %= config->ref2.window_length;
        break;
    }
}

int start_server(void)
{
    int sock_server;

    int yes = 1;
    struct sockaddr_in addr;
    //Setup sockat, listen for start, receive commands (not yet) and send data
    if ((sock_server = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket");
        return EXIT_FAILURE;
    }
    setsockopt(sock_server, SOL_SOCKET, SO_REUSEADDR, (void*)&yes, sizeof(yes));

    /* setup listening address */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(RP_DEFAULT_PORT);

    if (bind(sock_server, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return EXIT_FAILURE;
    }

    listen(sock_server, SERVER_LISTEN_BACKLOG);
    printf("Listening on port %d ...\n", RP_DEFAULT_PORT);

    return sock_server;
}

void add_white_noise(void** memory_map, t_config* config)
{
#ifdef RP_VARIANT_PHASEMETER
    (void)memory_map;
#endif
    static int init = 0;
    static double* window_array_1 = NULL;
    static double* window_array_2 = NULL;
    static int warned_alloc = 0;

    if (!init) {
        init = 1;
        if (config->ref1.window_length < 1) {
            config->ref1.window_length = 1;
        }
        if (config->ref2.window_length < 1) {
            config->ref2.window_length = 1;
        }
        window_array_1 = malloc(sizeof(double) * (size_t)config->ref1.window_length);
        window_array_2 = malloc(sizeof(double) * (size_t)config->ref2.window_length);
        if (!window_array_1 || !window_array_2) {
            if (!warned_alloc) {
                warned_alloc = 1;
                fprintf(stderr, "Warning: add_white_noise allocation failed (wl1=%d wl2=%d). Disabling noise.\n",
                        config->ref1.window_length, config->ref2.window_length);
                fflush(stderr);
            }
            // Don't crash; just disable the noise feature.
            return;
        }
        for (int i = 0; i < config->ref1.window_length; ++i) {
            window_array_1[i] = 0.0;
        }
        for (int i = 0; i < config->ref2.window_length; ++i) {
            window_array_2[i] = 0.0;
        }
    }

    if (!window_array_1 || !window_array_2) {
        return;
    }

    double ref_freq_tmp_1 = rand_gaussian(config->ref1.freq_noise_floor, MAGIC_RAND_SCALE);
    double ref_freq_tmp_2 = rand_gaussian(config->ref2.freq_noise_floor, MAGIC_RAND_SCALE);
    simple_average_filter(&config->ref1.freq_ave, window_array_1, ref_freq_tmp_1, config->ref1.freq_index, config->ref1.window_length);
    simple_average_filter(&config->ref2.freq_ave, window_array_2, ref_freq_tmp_2, config->ref2.freq_index, config->ref2.window_length);
#ifndef RP_VARIANT_PHASEMETER
    *((uint32_t*)(memory_map[CFG_SERVOS_0])) = (uint32_t)((config->ref1.freq_now + config->ref1.freq_ave) * FREQUENCY_FACTOR_A);
    *((uint32_t*)(memory_map[CFG_SERVOS_1])) = (uint32_t)((config->ref2.freq_now + config->ref2.freq_ave) * FREQUENCY_FACTOR_A);
#endif
    config->ref1.freq_index = (config->ref1.freq_index + 1) % config->ref1.window_length;
    config->ref2.freq_index = (config->ref2.freq_index + 1) % config->ref2.window_length;
}

int main(int argc, char** argv)
{
    int persist = 0;
    int skip_cap_handshake = 0;
    pll_logger pll_log = { 0 };
    int log_file_set = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "persist") == 0)
            persist = 1;
        else if (strcmp(argv[i], "--no-cap") == 0)
            skip_cap_handshake = 1;
        else if (argv[i][0] != '-' && !log_file_set) {
            pll_logger_init(&pll_log, argv[i]);
            log_file_set = 1;
        }
    }

    // parameters: white noise for reference frequencies
    srand((unsigned)time(NULL));
    t_config config = {
        .start_measuring = 0,
        .ref1 = {
            .freq_now = 0.0,
            .freq_noise_floor = 0.0,  // white noise floor
            .freq_noise_corner = 1.0, // corner freq. (initially 1Hz, which is minimum)
            .freq_index = 0,
            .window_length = ceil(0.443 * MAGIC_DATA_RATE), // window length
            .freq_ave = 0.0 },
        .ref2 = { .freq_now = 0.0,
            .freq_noise_floor = 0.0,  // white noise floor
            .freq_noise_corner = 1.0, // corner freq. (initially 1Hz, which is minimum)
            .freq_index = 0,
            .window_length = ceil(0.443 * MAGIC_DATA_RATE), // window length
            .freq_ave = 0.0 }
    };

    // Map memory shared between FPGA and CPI
    void** memory_map;
    if (!(memory_map = mmap_all(SHARED_MEMORY_FILE))) {
        perror("mmap_all");
        return 1;
    }
    reset(memory_map);

    int sock_client = -1;
    int need_warmup_before_first_send = 0;
    uint32_t command[2] = { 0, 0 };
    uint8_t cmd_buf[CMD_BUF_SIZE];
    size_t cmd_have = 0;
    int cmd_need_8 = 0;
    size_t cmd_need = 4;
    size_t cmd_consumed;

    int sock_server = start_server();
    // Helpful startup marker for remote debugging.
    fprintf(stderr, "Server initialized (persist=%d, skip_cap=%d). Waiting for client...\n", persist, skip_cap_handshake);
    fflush(stderr);
    if (persist) {
        // In persist mode keep the control loop running even when no client is connected.
        // Make accept() non-blocking so we don't stall while waiting for a reconnect.
        int flags = fcntl(sock_server, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(sock_server, F_SETFL, flags | O_NONBLOCK);
        }
    }

    while (1) {
        // Accept a new client if none connected.
        if (sock_client < 0) {
            sock_client = accept(sock_server, NULL, NULL);
            if (sock_client < 0) {
                if (!(errno == EAGAIN || errno == EWOULDBLOCK)) {
                    perror("accept");
                    pll_logger_close(&pll_log);
                    return EXIT_FAILURE;
                }
            } else {
                fprintf(stderr, "Client connected (fd=%d)\n", sock_client);
                fflush(stderr);
                config.start_measuring = 0;
                if (!skip_cap_handshake) {
                    char cap_line[64];
                    int n = snprintf(cap_line, sizeof(cap_line), RP_CAP_PREFIX "%s\n", RP_SERVER_VARIANT);
                    if (n > 0 && (size_t)n < sizeof(cap_line)) {
                        send(sock_client, cap_line, (size_t)n, MSG_NOSIGNAL);
                        fprintf(stderr, "Capability handshake sent: %s", cap_line);
                        fflush(stderr);
                    }
                } else {
                    fprintf(stderr, "Capability handshake skipped (--no-cap).\n");
                    fflush(stderr);
                }
                /* Warmup scope: run a few trigger/read cycles to flush stale data
                 * after reconnect. Prevents corrupted FFT (huge peak at high freq,
                 * shifted peak) on first frames. */
                for (int w = 0; w < 5; w++) {
                    read_and_send(memory_map, -1, &pll_log);
                }
                need_warmup_before_first_send = 1;
                cmd_have = 0;
                cmd_need_8 = 0;
                cmd_need = 4;
            }
        }

        // --- add white noise to reference frequency -------------------
        add_white_noise(memory_map, &config);

        // --- receive data from client -------------------
        int have_command = 0;
        if (sock_client >= 0) {
            ssize_t r = recv(sock_client, cmd_buf + cmd_have, cmd_need - cmd_have, MSG_DONTWAIT);
            if (r == 0) {
                close(sock_client);
                sock_client = -1;
                config.start_measuring = 0;
                cmd_have = 0;
                cmd_need_8 = 0;
                cmd_need = 4;
            } else if (r < 0) {
                if (!(errno == EAGAIN || errno == EWOULDBLOCK)) {
                    perror("recv");
                    close(sock_client);
                    sock_client = -1;
                    config.start_measuring = 0;
                    cmd_have = 0;
                    cmd_need_8 = 0;
                    cmd_need = 4;
                }
            } else {
                cmd_have += (size_t)r;
                if (parse_one_command(cmd_buf, cmd_have, &cmd_need_8, command, &cmd_consumed)) {
                    have_command = 1;
                    cmd_have = 0;
                    cmd_need_8 = 0;
                }
                cmd_need = cmd_need_8 ? 8 : 4;
            }
        }

        if (have_command) {
            process_command(memory_map, command, &config);
        }

        if (sock_client >= 0 && config.start_measuring && *(int32_t*)(memory_map[RD_FLG_SC])) {
            if (need_warmup_before_first_send) {
                for (int w = 0; w < 5; w++) {
                    read_and_send(memory_map, -1, &pll_log);
                }
                need_warmup_before_first_send = 0;
            }
            read_and_send(memory_map, sock_client, &pll_log);
        }

        // Avoid spinning at 100% CPU when running persist mode without a client.
        if (persist && sock_client < 0) {
            usleep(1000);
        }
    }

    pll_logger_close(&pll_log);
    close(sock_client);
    close(sock_server);
    munmap(memory_map[CFG_RST], sysconf(_SC_PAGESIZE));
    // munmap(memory_map[CFG_F_G], sysconf(_SC_PAGESIZE));
    // munmap(memory_map[CFG_DDS], sysconf(_SC_PAGESIZE));
    // munmap(memory_map[RD_FLG_SC], sysconf(_SC_PAGESIZE));
    munmap(memory_map[BRAM_SCOPE], sysconf(_SC_PAGESIZE));
    munmap(memory_map[RD_FLG_SC], sysconf(_SC_PAGESIZE));
    return 0;
}
