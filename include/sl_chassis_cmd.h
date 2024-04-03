/*
* Slamtec CHASSIS SDK
*
* sl_chassis_cmd.h
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

#include "sl_chassis_protocol.h"


#define SLAMTEC_PROTOCOL_VERSION                                   (0x1)


//for slamkit cmd
#define CMD_CODE_SLAMTEC_SLAMKIT                                            (0xF1)

#define SL_SLAMKIT_CMD_CONNECT                                              (0x10)
#define SL_SLAMKIT_CMD_GET_DEVICE_INFO                                      (0x3f)
#define SL_SLAMKIT_CMD_READ_IMU_PROCESSED                                   (0x5d)
#define SL_SLAMKIT_CMD_READ_IMU_RAW                                         (0x5f)


#if defined(_WIN32)
#pragma pack(1)
#endif

typedef struct _sl_chassis_cmd_t
{
    sl_u8   cmd;
    sl_u8  payload[0];
} __attribute__((packed)) sl_chassis_cmd_t;


#define BASIC_INFO_TYPE     (0)
typedef struct _sl_slamkit_info_request_t
{
    sl_u8  request_type;
    sl_u32 request_key;
} __attribute__((packed)) sl_slamkit_info_request_t;


typedef struct _sl_slamkit_info_response_t
{
    sl_u16  model;
    sl_u16 firmware_version;
    sl_u16 hardware_version;
    sl_u32  serial_number[4];
    sl_u32 hw_features;
    sl_u32 licensed_features;
} __attribute__((packed)) sl_slamkit_info_response_t;


#define SLAMKIT_REQUEST_MOTION_HINT_BITMAP_MOTION_BIT       (0x1<<0)
#define SLAMKIT_REQUEST_MOTION_HINT_BITMAP_HAS_CONTROL_V    (0x1<<7)

typedef struct _sl_slamkit_read_imu_processed_request_t
{
    sl_u32 request_key;
    sl_u8  motion_hint_bitmap;
    sl_s32 ux_q16;
    sl_s32 uy_q16;
    sl_s32 uw_q16;
} __attribute__((packed)) sl_slamkit_read_imu_processed_request_t;


typedef struct acc_processed_data
{
    sl_u32     x_d4;
    sl_u32     y_d4;
    sl_u32     z_d4;
} __attribute__((packed)) acc_processed_data_t;

// pre-calibrated gyro data in the unit degressx10^4/sec (and degress x 100)
typedef struct gyro_processed_data
{
    sl_u32     wx_d4;
    sl_u32     wy_d4;
    sl_u32     wz_d4;

    sl_u32     sum_x_d4;
    sl_u32     sum_y_d4;
    sl_u32     sum_z_d4;
} __attribute__((packed)) gyro_processed_data_t;


#define IMU_BITMAP_AVAILABLE_BIT_LINEAR_ACC      (0x1<<0)
#define IMU_BITMAP_AVAILABLE_BIT_ROTATION_V      (0x1<<1)
#define IMU_BITMAP_AVAILABLE_BIT_ROTATION_DELTA  (0x1<<2)

#define IMU_BITMAP_AVAILABLE_BIT_HW_ERROR        (0x1<<7)

typedef struct _sl_slamkit_read_imu_processed_response_t
{
    sl_u32     timestamp;
    sl_u8      avail_bitmap; // 1st LSB for acc, 2nd for gyro ...
    acc_processed_data_t acc;
    gyro_processed_data_t gyro;
} __attribute__((packed)) sl_slamkit_read_imu_processed_response_t;


typedef struct _sl_slamkit_read_imu_raw_request_t
{
    sl_u32 request_key;
} __attribute__((packed)) _sl_slamkit_read_imu_raw_request_t;


typedef struct accelemeter_data
{
    sl_u16 acc_x;
    sl_u16 acc_y;
    sl_u16 acc_z;
} __attribute__((packed)) accelemeter_data_t;

typedef struct gyrosensor_data
{
    sl_u16 gyro_x;
    sl_u16 gyro_y;
    sl_u16 gyro_z;
} __attribute__((packed)) gyrosensor_data_t;

typedef struct inertia_sensor_data
{
    accelemeter_data_t acc;
    gyrosensor_data_t  gyro;
    sl_u16 ref_volt;
    sl_u32     timestamp;
}  __attribute__((packed)) inertia_sensor_data_t;

typedef struct compass_raw_data
{
    sl_s16 hmc_x;
    sl_s16 hmc_y;
    sl_s16 hmc_z;
    sl_u16 ref_volt;
} __attribute__((packed)) compass_raw_data_t;

typedef struct _sl_slamkit_read_imu_raw_response_t
{
    inertia_sensor_data_t     inertia_raw_data;
    compass_raw_data_t        compass_raw_data;
} __attribute__((packed)) sl_slamkit_read_imu_raw_response_t;



#if defined(_WIN32)
#pragma pack()
#endif
