/*
* Slamtec SLAMKIT SDK
*
* sl_slamkit_protocol.h
*
* Copyright (c) 2020 Shanghai Slamtec Co., Ltd.
*/

/*
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include "sl_types.h"

#define SL_SLAMKIT_CMD_SYNC_BYTE                        (0x50)


#define STATUS_CODE_SYNC                                (0x00)
#define STATUS_CODE_ECHO                                (0x01)
#define STATUS_CODE_ANS                                 (0x02)
#define STATUS_CODE_ANS_RXERR                           (0x03)



#define PKT_ERRORCODE_NULL                              (0)
#define PKT_ERRORCODE_CHECKSUM_FAIL                     (0x40)
#define PKT_ERRORCODE_SIZE_OVERFLOW                     (0x20)
#define PKT_ERRORCODE_NOT_SYNC_PKT                      (0x10)
#define PKT_ERRORCODE_NOT_SUPPORT                       (0x8000)
#define PKT_ERRORCODE_BAD_CMD                           (0x8001)
#define PKT_ERRORCODE_OPERATION_FAIL                    (0x8002)

#define PKT_SIZE_EX_PAYLOAD                             (5)



#if defined(_WIN32)
#pragma pack(1)
#endif

typedef struct sl_slamkit_cmd_packet_t
{
    sl_u8   sync_byte; //must be SL_SLAMKIT_CMD_SYNC_BYTE
    sl_u16  length;
    sl_u8   cmd;
    sl_u8   data[0];
} __attribute__((packed)) sl_slamkit_cmd_packet_t;



#if defined(_WIN32)
#pragma pack()
#endif
