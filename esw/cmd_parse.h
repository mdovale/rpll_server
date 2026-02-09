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

/**
 * \file cmd_parse.h
 * \brief Parse binary commands from the client (4- or 8-byte protocol).
 */

#ifndef CMD_PARSE_H
#define CMD_PARSE_H

#include <stddef.h>
#include <stdint.h>

/**
 * \brief Parse one command from the receive buffer (4 or 8 bytes).
 *
 * Legacy: single uint32 with addr in high byte, value in low 24 bits.
 * Extended: two uint32s (addr, value). Disambiguation: addr==1 or low24!=0 => legacy.
 *
 * \param buf Receive buffer.
 * \param len Number of bytes currently in buf.
 * \param need_8 In/out: 0 = expect 4 bytes first, 1 = expect 8 bytes total.
 * \param command[2] Output: parsed (command[0]=addr/word0, command[1]=value/word1).
 * \param consumed Output: bytes consumed (4 or 8) when returning 1.
 * \return 1 if a full command was parsed, 0 if more data needed.
 */
int parse_one_command(const uint8_t* buf, size_t len, int* need_8, uint32_t command[2], size_t* consumed);

#endif
