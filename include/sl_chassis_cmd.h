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

//for base cmd
#define CMD_CODE_SLAMTEC_BASE                                           (0xF8)

#define SL_BASE_CMD_CONNECT                                             (0x10)
#define SL_BASE_CMD_GET_BAT_STATUS                                      (0x30)
#define SL_BASE_CMD_GET_ULTRASONIC_DATA                                 (0x32)
#define SL_BASE_CMD_GET_BUMPER_AND_CLIFF_DATA                           (0x33)
#define SL_BASE_CMD_SET_V_AND_GET_DEADRECKON                            (0x41)

#define SL_BASE_CMD_HEALTH_MGMT                                         (0x90)
#    define SL_BASE_CMD_HEALTH_GET_HEALTH                               (0x01)
#    define SL_BASE_CMD_HEALTH_GET_ERROR                                (0x02)
#    define SL_BASE_CMD_HEALTH_CLR_ERROR                                (0x03)

//for cp0 cmd
#define CMD_CODE_SLAMTEC_CP0                                            (0xF1)

#define SL_CP0_CMD_CONNECT                                              (0x10)
#define SL_CP0_CMD_GET_DEVICE_INFO                                      (0x3f)
#define SL_CP0_CMD_SET_LIDAR_POWER_ON                                   (0x4e)
#define SL_CP0_CMD_READ_IMU_PROCESSED                                   (0x5d)
#define SL_CP0_CMD_READ_IMU_RAW                                         (0x5f)
#define SL_CP0_CMD_SET_SL_LIDAR_BAUDRATE                                (0x6c)
#define SL_CP0_CMD_SET_SL_LIDARMOTOR_PWM                                (0x6e)



#if defined(_WIN32)
#pragma pack(1)
#endif




typedef struct _sl_chassis_cmd_t
{
    sl_u8   cmd;
    sl_u8  payload[0];
} __attribute__((packed)) sl_chassis_cmd_t;


typedef struct _sl_chassis_connect_request_t
{
    sl_u8  protocol_version;
} __attribute__((packed)) sl_chassis_connect_request_t;



typedef struct _sl_chassis_base_connect_response_t
{
    sl_u8  model[12];
    sl_u16 firmware_version;
    sl_u16 hardware_version;
    sl_u32 serial_number[3];
    sl_u8  firmware_version_suffix[11];
} __attribute__((packed)) sl_chassis_base_connect_response_t;

typedef sl_chassis_base_connect_response_t sl_chassis_base_device_info_t;

enum _sl_chassis_bat_status
{
    /* 
        // dock charging status
        1. PowerStatusNone:                                      when the robot is off the dock, the status is 0 which means OffDock.
        2. PowerStatusDockDetected:                              the IR communication detected the dock
        3. PowerStatusDockDetected + PowerStatusDockContacted:   the robot touches the dock
        4. PowerStatusDockDetected + PowerStatusDockContacted + PowerStatusDockNegociating: the robot sent the start charge command to the dock
        5. PowerStatusDockContacted + PowerStatusDockContacted + PowerStatusCharging    : charging
        6. PowerStatusDockLocked:                                the dock connect stable.

        // dc charging status
        1. PowerStatusNone:
        2. PowerStatusDcConnected:                               dc socket inserted.

        // charging status
        1. PowerStatusNone:
        2. PowerStatusCharging:                                  robot is charging.

    */
    PowerStatusNone = 0,
    PowerStatusCharging = 1,

    PowerStatusDcConnected = 2,

    PowerStatusDockLocked = 4,
    PowerStatusDockDetected = 8,
    PowerStatusDockContacted = 16,
    PowerStatusDockNegociating = 32
};


typedef struct _sl_chassis_bat_status_response_t
{
    sl_u8 battery_percentage;
    sl_u8 power_status;
} __attribute__((packed)) sl_chassis_bat_status_response_t;


typedef struct _sl_chassis_ultrasonic_data_response_t
{
    sl_u32  ultrasonic_data_mm_q16[16];
} __attribute__((packed)) sl_chassis_ultrasonic_data_response_t;


typedef struct _sl_chassis_bumper_and_cliff_data_response_t
{
    sl_u32  bumper_and_cliff;
} __attribute__((packed)) sl_chassis_bumper_and_cliff_data_response_t;


typedef struct _sl_chassis_set_velocity_request_t
{
    sl_s32 velocity_x_q16;
    sl_s32 velocity_y_q16;
    sl_s32 angular_velocity_q16;
} __attribute__((packed)) sl_chassis_set_velocity_request_t;



typedef struct _sl_chassis_deadreckon_response_t
{
    sl_s32 base_dx_mm_q16;
    sl_s32 base_dy_mm_q16;
    sl_s32 base_dtheta_degree_q16;
    sl_u8  status_bitmap;
    sl_u32 timestamp;                     //!< time ticks since base is powered up, in ms  
} __attribute__((packed)) sl_chassis_deadreckon_response_t;



#define SL_BASE_HEALTH_FLAG_HEALTHY        0
#define SL_BASE_HEALTH_FLAG_HAS_WARN       1
#define SL_BASE_HEALTH_FLAG_HAS_ERROR      2
#define SL_BASE_HEALTH_FLAG_HAS_FATAL      4

#define SL_BASE_HEALTH_ERROR_NONE          (0u << 24)
#define SL_BASE_HEALTH_ERROR_WARN          (1u << 24)
#define SL_BASE_HEALTH_ERROR_ERROR         (2u << 24)
#define SL_BASE_HEALTH_ERROR_FATAL         (4u << 24)

// components
#define BASE_COMPONENT_USER     (0u << 16)
#define BASE_COMPONENT_SYSTEM   (1u << 16)
#define BASE_COMPONENT_POWER    (2u << 16)
#define BASE_COMPONENT_MOTION   (3u << 16)
#define BASE_COMPONENT_SENSOR   (4u << 16)

#define BASE_SYSTEM_ERROR_EMERGENCY_STOP    (SL_BASE_HEALTH_ERROR_ERROR | BASE_COMPONENT_SYSTEM | 0x0100u)
#define BASE_POWER_WARN_POWER_LOW           (SL_BASE_HEALTH_ERROR_WARN | BASE_COMPONENT_POWER | 0x0100u)
#define BASE_SYSTEM_ERROR_BRAKE_RELEASED    (SL_BASE_HEALTH_ERROR_ERROR | BASE_COMPONENT_SYSTEM | 0x0700u)
#define BASE_MOTION_WARN_MOTOR_ALARM        (SL_BASE_HEALTH_ERROR_WARN  | BASE_COMPONENT_MOTION | 0x0100u) // the lowest 8bits is used to tell which motor down
#define BASE_SENSOR_ERROR_MAGTAPE_TRIGGERED     (SL_BASE_HEALTH_ERROR_ERROR | BASE_COMPONENT_SENSOR | 0x0500u)
#define BASE_SENSOR_FATAL_MAG_SELF_TEST_FAIL     (SL_BASE_HEALTH_ERROR_FATAL | BASE_COMPONENT_SENSOR | 0x0600u)
#define BASE_SENSOR_ERROR_BUMPER_DOWN       (SL_BASE_HEALTH_ERROR_ERROR | BASE_COMPONENT_SENSOR | 0x0100u) // the lowest 8bits is used to tell which sensor down


typedef struct _sl_chassis_health_mgmt_cmd_t
{
    sl_u8 cmd;
    sl_u8 payload[0]; 
} __attribute__((packed)) sl_chassis_health_mgmt_cmd_t;

typedef struct _sl_chassis_health_response_t
{
    sl_u8 health_flag;
    sl_u8 error_count;
} __attribute__((packed)) sl_chassis_health_response_t;

typedef struct _sl_chassis_health_error_request_t
{
    sl_u8 error_id; // should from 0 to error_count(not include)
} __attribute__((packed)) sl_chassis_health_error_request_t;

#define SL_BASE_HEALTH_ERROR_MSG_LENGTH  (32)
typedef struct _sl_chassis_health_error_response_t
{
    sl_u32 error_code;
    char error_msg[SL_BASE_HEALTH_ERROR_MSG_LENGTH];
} __attribute__((packed)) sl_chassis_health_error_response_t;

typedef struct _sl_chassis_health_clear_error_request_t
{
    sl_u32 error_code;
} __attribute__((packed)) sl_chassis_health_clear_error_request_t;



#define BASIC_INFO_TYPE     (0)
typedef struct _sl_cp0_info_request_t
{
    sl_u8  request_type;
    sl_u32 request_key;
} __attribute__((packed)) sl_cp0_info_request_t;


typedef struct _sl_cp0_info_response_t
{
    sl_u16  model;
    sl_u16 firmware_version;
    sl_u16 hardware_version;
    sl_u32  serial_number[4];
    sl_u32 hw_features;
    sl_u32 licensed_features;
} __attribute__((packed)) sl_cp0_info_response_t;


typedef enum _sl_cp0_gpio_operation_t
{
    GPIO_CLEAR = 0,
    GPIO_SET,
} sl_cp0_gpio_operation_t;

typedef enum _sl_cp0_gpio_group_t 
{
    GPIO_CTRL_BUS = 0,
} sl_cp0_gpio_group_t;

typedef enum _sl_cp0_gpio_value_t
{
    GPIO_CTRL_BUS_SIGNAL_CMD = 0,
    GPIO_CTRL_BUS_SIGNAL_BUSY,
    GPIO_LIDAR_BUS_SIGNAL_N5VEN
} sl_cp0_gpio_value_t;


typedef struct _sl_cp0_write_gpio_request_t
{
    sl_cp0_gpio_operation_t     gpio_operation;
    sl_cp0_gpio_group_t         gpio_group;
    sl_cp0_gpio_value_t         gpio_value;
    sl_u32                      request_key;
} __attribute__((packed)) sl_cp0_write_gpio_request_t;



#define CP0_REQUEST_MOTION_HINT_BITMAP_MOTION_BIT       (0x1<<0)
#define CP0_REQUEST_MOTION_HINT_BITMAP_HAS_CONTROL_V    (0x1<<7)

typedef struct _sl_cp0_read_imu_processed_request_t
{
    sl_u32 request_key;
    sl_u8  motion_hint_bitmap;
    sl_s32 ux_q16;
    sl_s32 uy_q16;
    sl_s32 uw_q16;
} __attribute__((packed)) sl_cp0_read_imu_processed_request_t;




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

typedef struct _sl_cp0_read_imu_processed_response_t
{
    sl_u32     timestamp;
    sl_u8      avail_bitmap; // 1st LSB for acc, 2nd for gyro ...
    acc_processed_data_t acc;
    gyro_processed_data_t gyro;
} __attribute__((packed)) sl_cp0_read_imu_processed_response_t;


typedef struct _sl_cp0_read_imu_raw_request_t
{
    sl_u32 request_key;
} __attribute__((packed)) _sl_cp0_read_imu_raw_request_t;


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

typedef struct _sl_cp0_read_imu_raw_response_t
{
    inertia_sensor_data_t     inertia_raw_data;
    compass_raw_data_t        compass_raw_data;
} __attribute__((packed)) sl_cp0_read_imu_raw_response_t;



typedef struct _sl_cp0_set_lidar_baudrate_request_t
{
    sl_u32 request_key;
    sl_u32 baudrate;
    sl_u8  databits;
    sl_u8  stopbits;
    sl_u8  parity;
} __attribute__((packed)) sl_cp0_set_lidar_baudrate_request_t;



typedef struct _sl_cp0_set_lidarmotor_pwm_request_t
{
    sl_u32 lidarmotor_pwm;
    sl_u32 request_key;
} __attribute__((packed)) sl_cp0_set_lidarmotor_pwm_request_t;


#if defined(_WIN32)
#pragma pack()
#endif
