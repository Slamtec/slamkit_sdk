/*
 *  Slamtec SLAMKIT SDK
 *
 *  Copyright (c) 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
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

#ifndef __cplusplus
#error "The Slamtec SLAMKIT SDK requires a C++ compiler to be built"
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>

#include "sl_slamkit_cmd.h"

namespace sl {

    typedef struct _sl_imu_raw_data_t
    {
        sl_s16     acc_x;
        sl_s16     acc_y;
        sl_s16     acc_z;

        sl_s16     gyro_x;
        sl_s16     gyro_y;
        sl_s16     gyro_z;

        sl_s16     mag_x;
        sl_s16     mag_y;
        sl_s16     mag_z;

        sl_u32     timestamp;
    } sl_imu_raw_data_t;




    /**
    * Abstract interface of communication channel
    */
    class IChannel
    {
    public:
        virtual ~IChannel() {}

    public:
        /**
        * Open communication channel (return true if succeed)
        */
        virtual bool open() = 0;

        /**
        * Close communication channel
        */
        virtual void close() = 0;

        /**
        * Flush all written data to remote endpoint
        */
        virtual void flush() = 0;

        /**
        * Wait for some data
        * \param size Bytes to wait
        * \param timeoutInMs Wait timeout (in microseconds, -1 for forever)
        * \param actualReady [out] actual ready bytes
        * \return true for data ready
        */
        virtual bool waitForData(size_t size, sl_u32 timeoutInMs = -1, size_t* actualReady = nullptr) = 0;

        /**
        * Send data to remote endpoint
        * \param data The data buffer
        * \param size The size of data buffer (in bytes)
        * \return Bytes written (negative for write failure)
        */
        virtual int write(const void* data, size_t size) = 0;

        /**
        * Read data from the chanel
        * \param buffer The buffer to receive data
        * \param size The size of the read buffer
        * \return Bytes read (negative for read failure)
        */
        virtual int read(void* buffer, size_t size) = 0;

        /**
        * Clear read cache
        */
        virtual void clearReadCache() = 0;

    private:

    };

    /**
    * Abstract interface of serial port channel
    */
    class ISerialPortChannel : public IChannel
    {
    public:
        virtual ~ISerialPortChannel() {}

    public:
        virtual void setDTR(bool dtr) = 0;
    };

    /**
    * Create a serial channel
    * \param device Serial port device
    *                   e.g. on Windows, it may be com3 or \\.\com10
    *                   on Unix-Like OS, it may be /dev/ttyS1, /dev/ttyUSB2, etc
    * \param baudrate Baudrate
    *                   Please refer to the datasheet for the baudrate (maybe 115200 or 256000)
    */
    std::shared_ptr<IChannel> createSerialPortChannel(const std::string& device, int baudrate);

    /**
    * Create a TCP channel
    * \param ip IP address of the device
    * \param port TCP port
    */
    std::shared_ptr<IChannel> createTcpChannel(const std::string& ip, int port);

    /**
    * Create a UDP channel
    * \param ip IP address of the device
    * \param port UDP port
    */
    std::shared_ptr<IChannel> createUdpChannel(const std::string& ip, int port);

    /**
    * Create a USB channel
    * \param venderId: usb vender id
    * \param productId: usb product id
    * \param interfaceId: usb interface id
    * \param txEndpoint: usb tx Endpoint
    * \param rxEndpoint: usb rx Endpoint
    */
    std::shared_ptr<IChannel> createUSBChannel(std::uint16_t venderId, std::uint16_t productId,
            std::uint16_t interfaceId, std::uint16_t txEndpoint, std::uint16_t rxEndpoint);

    enum ChannelType{
        CHANNEL_TYPE_SERIALPORT = 0x0,
        CHANNEL_TYPE_TCP = 0x1,
        CHANNEL_TYPE_UDP = 0x2,
        CHANNEL_TYPE_USB = 0x3,
    };

    enum
    {
        DEFAULT_TIMEOUT = 2000
    };

    class IConnect
    {
    public:
        virtual ~IConnect() {}

    public:
        /**
        * Connect to SLAMKIT via channel
        * \param channel The communication channel
        *                    Note: you should manage the lifecycle of the channel object, make sure it is alive during slamkit driver's lifecycle
        */
        virtual sl_result connect(std::shared_ptr<IChannel> &channel) = 0;

        /**
        * Disconnect from the SLAMKIT
        */
        virtual void disconnect() = 0;
        
        /**
        * Check if the connection is established
        */
        virtual bool isConnected() = 0;

    public:
        //enum
        //{
        //    DEFAULT_TIMEOUT = 2000
        //};
    };

    class ISlamkitDriver : public IConnect
    {
    public:
        virtual ~ISlamkitDriver() {}

    public:
        /// Retrieve the health status of the SL_SLAMKIT
        /// The host system can use this operation to check whether SL_SLAMKIT is in the self-protection mode.
        ///
        /// \param health        The health status info returned from the SL_SLAMKIT
        ///
        /// \param timeout       The operation timeout value (in millisecond) for port communication     
        //virtual sl_result getHealth(sl_slamkit_response_device_health_t& health, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Get the device information of the SL_SLAMKIT include number, firmware version, device model etc.
        /// 
        /// \param info          The device information returned from the SL_SLAMKIT
        /// \param timeout       The operation timeout value (in millisecond) for port communication  
        virtual sl_result getDeviceInfo(sl_slamkit_info_response_t& info, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Set motion hit and get imu processed data of the SL_SLAMKIT, should include motion hit bit map.
        /// 
        /// \param req              The motion hit set to the SL_SLAMKIT
        /// \param processed_data   The processed data returned from the SL_SLAMKIT
        /// \param timeout          The operation timeout value (in millisecond) for port communication  
        virtual sl_result set_motion_hit_and_get_imu_processed(const sl_slamkit_read_imu_processed_request_t& req, sl_slamkit_read_imu_processed_response_t& processed_data, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;
        
        /// Get imu raw data of the SL_SLAMKIT include number, firmware version, device model etc.
        /// 
        /// \param imu_raw_data  The imu raw data returned from the SL_SLAMKIT
        /// \param timeout       The operation timeout value (in millisecond) for port communication  
        virtual sl_result getImuRawData(sl_imu_raw_data_t& imu_raw_data, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;
};

    std::shared_ptr<ISlamkitDriver> createSlamkitDriver();
}
