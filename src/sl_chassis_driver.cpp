/*
 * Slamtec CHASSIS SDK
 *
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
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

#include "sdkcommon.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "hal/assert.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/event.h"
#include "sl_chassis_driver.h"
#include "sl_crc.h" 
#include <algorithm>
#include <chrono>


#ifdef _WIN32
#define NOMINMAX
#undef min
#undef max
#endif

#if defined(__cplusplus) && __cplusplus >= 201103L
#ifndef _GXX_NULLPTR_T
#define _GXX_NULLPTR_T
typedef decltype(nullptr) nullptr_t;
#endif
#endif /* C++11.  */

namespace sl {

    
    class SlamtecBaseDriver :public IBaseDriver
    {
    public:
        SlamtecBaseDriver()
            : _channel(nullptr)
            , _isConnected(false)
        {}

        sl_result connect(std::shared_ptr<IChannel> &channel)
        {
            //printf("9 \n");
            //sl_result ans = SL_RESULT_OK;

            if (!channel) return SL_RESULT_OPERATION_FAIL;
            if (isConnected()) return SL_RESULT_ALREADY_DONE;
            _channel = channel;
            //printf("7 \n");
            {
                rp::hal::AutoLocker l(_lock);
                auto ans = _channel->open();
                //printf("8 \n");

                if (!ans)
                {
                    return SL_RESULT_OPERATION_FAIL;
                }
                    
                _channel->flush();
            }
     
            _isConnected = true;

            return SL_RESULT_OK;
        }

        void disconnect()
        {
            if (_isConnected)
                _channel->close();
        }

        bool isConnected()
        {
            return _isConnected;
        }

        sl_result getDeviceInfo(sl_chassis_base_device_info_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_chassis_connect_request_t payload;
            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_CONNECT;
            tx_req.payload.protocol_version = SLAMTEC_PROTOCOL_VERSION;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[100] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_base_device_info_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            
            sl_chassis_base_device_info_t* ppayload = (sl_chassis_base_device_info_t*) (&data_buffer[4]);
            
            memcpy(&info, ppayload, sizeof(sl_chassis_base_device_info_t));
            return SL_RESULT_OK;
        }

        sl_result getBatteryStatus(sl_chassis_bat_status_response_t& status, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_GET_BAT_STATUS;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[100] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_bat_status_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            
            sl_chassis_bat_status_response_t* ppayload = (sl_chassis_bat_status_response_t*) (&data_buffer[4]);
            
            memcpy(&status, ppayload, sizeof(sl_chassis_bat_status_response_t));
            return SL_RESULT_OK;
        }

        sl_result getUltraData(sl_chassis_ultrasonic_data_response_t& data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_GET_ULTRASONIC_DATA;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[100] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_ultrasonic_data_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            
            sl_chassis_ultrasonic_data_response_t* ppayload = (sl_chassis_ultrasonic_data_response_t*) (&data_buffer[4]);
            
            memcpy(&data, ppayload, sizeof(sl_chassis_ultrasonic_data_response_t));
            return SL_RESULT_OK;
        }

        sl_result getBumperCliffData(sl_chassis_bumper_and_cliff_data_response_t& data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_GET_BUMPER_AND_CLIFF_DATA;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[100] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_bumper_and_cliff_data_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            
            sl_chassis_bumper_and_cliff_data_response_t* ppayload = (sl_chassis_bumper_and_cliff_data_response_t*) (&data_buffer[4]);
            
            memcpy(&data, ppayload, sizeof(sl_chassis_bumper_and_cliff_data_response_t));
            return SL_RESULT_OK;
        }

        sl_result set_v_and_get_deadreckon(const sl_chassis_set_velocity_request_t& set_v, sl_chassis_deadreckon_response_t& deadreckon, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            //
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_chassis_set_velocity_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_SET_V_AND_GET_DEADRECKON;
            memcpy(&tx_req.payload, &set_v, sizeof(set_v));

            //uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            //uint64_t now_us_send = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();



            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[1024] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_deadreckon_response_t), timeout);
            }

            //uint64_t now_rcv = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();


            //printf("send time delta: %lld , rev delt time: %lld \r\n",(now_us_send - now_us), now_rcv - now_us_send);


            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            //if (deadreckon != nullptr)
            {
                /* code */
            }
            
            sl_chassis_deadreckon_response_t* ppayload = (sl_chassis_deadreckon_response_t*) (&data_buffer[4]);
            
            memcpy(&deadreckon, ppayload, sizeof(sl_chassis_deadreckon_response_t));

            return SL_RESULT_OK;
        }

        sl_result getHealthInfo(sl_chassis_health_response_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            // struct _health_mgmt_t
            // {
            //     sl_chassis_health_mgmt_cmd_t
            // };


            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_chassis_health_mgmt_cmd_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_HEALTH_MGMT;
            tx_req.payload.cmd = SL_BASE_CMD_HEALTH_GET_HEALTH;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[10] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_health_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }
            
            sl_chassis_health_response_t* ppayload = (sl_chassis_health_response_t*) (&data_buffer[4]);
            
            memcpy(&info, ppayload, sizeof(sl_chassis_health_response_t));
            return SL_RESULT_OK;
        }

        sl_result getSingleHealthData(const sl_chassis_health_error_request_t& error_id, sl_chassis_health_error_response_t& error_data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            // struct _health_mgmt_t
            // {
            //     sl_chassis_health_mgmt_cmd_t
            // };


            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_chassis_health_mgmt_cmd_t payload;
                sl_chassis_health_error_request_t error_id;

            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_HEALTH_MGMT;
            tx_req.payload.cmd = SL_BASE_CMD_HEALTH_GET_ERROR;
            tx_req.error_id.error_id = error_id.error_id;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[100] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_chassis_health_error_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }
            
            sl_chassis_health_error_response_t* ppayload = (sl_chassis_health_error_response_t*) (&data_buffer[4]);
            
            memcpy(&error_data, ppayload, sizeof(sl_chassis_health_error_response_t));
            return SL_RESULT_OK;
        }

        sl_result clearSingleHealthData(const sl_chassis_health_clear_error_request_t& error_code, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_chassis_health_mgmt_cmd_t payload;
                sl_chassis_health_clear_error_request_t error_code;

            }tx_req;

            tx_req.cmd.cmd = SL_BASE_CMD_HEALTH_MGMT;
            tx_req.payload.cmd = SL_BASE_CMD_HEALTH_CLR_ERROR;
            tx_req.error_code.error_code = error_code.error_code;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_BASE, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[10] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, 0, timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            return SL_RESULT_OK;
        }

    private:
        
        sl_result  _sendCommand(sl_u8 cmd, const void * payload = nullptr, size_t payloadsize = 0 )
        {
            if (!_isConnected)
            {
                return SL_RESULT_OPERATION_FAIL;
            }
            

            sl_u8 checksum = 0;
            sl_u16 length = payloadsize + 1;
            sl_u8 length_l = (sl_u8)(length & 0x00ff);
            sl_u8 length_h = (sl_u8)((length & 0xff00)>>8);


            std::vector<sl_u8> cmd_packet;
            cmd_packet.clear();

			_channel->flush();
            cmd_packet.push_back(SL_CHASSIS_CMD_SYNC_BYTE);
            cmd_packet.push_back(length_l);
            cmd_packet.push_back(length_h);
            cmd_packet.push_back(cmd);

            checksum ^= SL_CHASSIS_CMD_SYNC_BYTE;
            checksum ^= length_l;
            checksum ^= length_h;
            checksum ^= cmd;

            for (size_t pos = 0; pos < payloadsize; ++pos) 
            {
                checksum ^= ((sl_u8 *)payload)[pos];
                cmd_packet.push_back(((sl_u8 *)payload)[pos]);
            }

            cmd_packet.push_back(checksum);


            sl_u8 packet[1024];
            for (sl_u32 pos = 0; pos < cmd_packet.size(); pos++) 
            {
                packet[pos] = cmd_packet[pos];
            }
            _channel->clearReadCache();
            _channel->write(packet, cmd_packet.size());

            //delay(1);
            return SL_RESULT_OK;
        }

        template <typename T>
        sl_result _waitResponse(T &payload, sl_u32 size, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            if (!_isConnected)
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            sl_u32 pkt_size = PKT_SIZE_EX_PAYLOAD + size;

            if (0 == size)
            {
                //pkt_size = PKT_SIZE_EX_PAYLOAD + 2;
                //timeout = 5;
            }
            

            if (!_channel->waitForData(pkt_size, timeout)) 
            {
                return SL_RESULT_OPERATION_TIMEOUT;
            }
            _channel->read(reinterpret_cast<sl_u8 *>(&payload), pkt_size);
            return SL_RESULT_OK;
        }
       

    private:
        std::shared_ptr<IChannel> _channel;
        bool _isConnected;
        rp::hal::Locker         _lock;
        rp::hal::Event          _dataEvt;
        //rp::hal::Thread         _cachethread;
    };

    std::shared_ptr<IBaseDriver> createBaseDriver()
    {
        return std::make_shared<SlamtecBaseDriver>();
    }


    class SlamtecCp0Driver :public ICp0Driver
    {
    public:
        SlamtecCp0Driver()
            : _channel(nullptr)
            , _isConnected(false)
        {}

        sl_result connect(std::shared_ptr<IChannel> &channel)
        {
            sl_result ans = SL_RESULT_OK;

            if (!channel) return SL_RESULT_OPERATION_FAIL;
            if (isConnected()) return SL_RESULT_ALREADY_DONE;
            _channel = channel;
            
            {
                rp::hal::AutoLocker l(_lock);
                ans = _channel->open();
                if (!ans)
                    return SL_RESULT_OPERATION_FAIL;

                _channel->flush();
            }
     
            _isConnected = true;

            return SL_RESULT_OK;
        }

        void disconnect()
        {
            if (_isConnected)
                _channel->close();
        }

        bool isConnected()
        {
            return _isConnected;
        }

        sl_result getDeviceInfo(sl_cp0_info_response_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO:
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_cp0_info_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_GET_DEVICE_INFO;
            memset(&tx_req.payload, 0, sizeof(sl_cp0_info_request_t));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[1024] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(info), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            } 

            sl_cp0_info_response_t* ppayload = (sl_cp0_info_response_t*) (&data_buffer[4]);
            
            memcpy(&info, ppayload, sizeof(sl_cp0_info_response_t));
            return SL_RESULT_OK;
        }

        sl_result setSLLidarPowerOn(sl_u32 timeout = DEFAULT_TIMEOUT)
        {
#if 1
            // 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_cp0_write_gpio_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_SET_LIDAR_POWER_ON;
            tx_req.payload.gpio_group = GPIO_CTRL_BUS;
            tx_req.payload.gpio_operation = GPIO_SET;
            tx_req.payload.gpio_value = GPIO_LIDAR_BUS_SIGNAL_N5VEN;
            tx_req.payload.request_key = 0;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[10] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, 0, timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

  #endif
            return SL_RESULT_OK;
        }

        sl_result setSLLidarPowerOff(sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_cp0_write_gpio_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_SET_LIDAR_POWER_ON;
            tx_req.payload.gpio_group = GPIO_CTRL_BUS;
            tx_req.payload.gpio_operation = GPIO_CLEAR;
            tx_req.payload.gpio_value = GPIO_LIDAR_BUS_SIGNAL_N5VEN;
            tx_req.payload.request_key = 0;

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[10] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, 0, timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }
            return SL_RESULT_OK;
        }

        sl_result set_motion_hit_and_get_imu_processed(const sl_cp0_read_imu_processed_request_t& req, sl_cp0_read_imu_processed_response_t& processed_data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            //
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_cp0_read_imu_processed_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_READ_IMU_PROCESSED;
            memcpy(&tx_req.payload, &req, sizeof(req));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[1024] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_cp0_read_imu_processed_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            //if (deadreckon != nullptr)
            {
                /* code */
            }
            
            sl_cp0_read_imu_processed_response_t* ppayload = (sl_cp0_read_imu_processed_response_t*) (&data_buffer[4]);
            
            memcpy(&processed_data, ppayload, sizeof(sl_cp0_read_imu_processed_response_t));
 
            return SL_RESULT_OK;
        }

        sl_result getImuRawData(sl_imu_raw_data_t& imu_raw_data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            //printf("test 0\r\n");
            // TODO: 
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                _sl_cp0_read_imu_raw_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_READ_IMU_RAW;
            tx_req.payload.request_key = 0;
            // memcpy(&tx_req.payload, &req, sizeof(req));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            //printf("test send ans = %08X\r\n", ans);
            if (SL_IS_FAIL(ans)) return ans;

            //printf("test send 0\r\n");

            sl_u8 data_buffer[1024] = {0};
            //sl_cp0_read_imu_raw_response_t raw_resp;
            //memset(&raw_resp, 0, sizeof(raw_resp));

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_cp0_read_imu_raw_response_t), timeout);
            }

            if (ans) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }
            
            sl_cp0_read_imu_raw_response_t* ppayload = (sl_cp0_read_imu_raw_response_t*) (&data_buffer[4]);
            
            imu_raw_data.acc_x = ppayload->inertia_raw_data.acc.acc_x;
            imu_raw_data.acc_y = ppayload->inertia_raw_data.acc.acc_y;
            imu_raw_data.acc_z = ppayload->inertia_raw_data.acc.acc_z;
            imu_raw_data.gyro_x = ppayload->inertia_raw_data.gyro.gyro_x;
            imu_raw_data.gyro_y = ppayload->inertia_raw_data.gyro.gyro_y;
            imu_raw_data.gyro_z = ppayload->inertia_raw_data.gyro.gyro_z;
            imu_raw_data.timestamp = ppayload->inertia_raw_data.timestamp;
            
            return SL_RESULT_OK;
        }

        sl_result setLidarBaudrate(const sl_cp0_set_lidar_baudrate_request_t& req, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO:
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_cp0_set_lidar_baudrate_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_SET_SL_LIDAR_BAUDRATE;
            memcpy(&tx_req.payload, &req, sizeof(req));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[1024] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, 0, timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            return SL_RESULT_OK;
        }

        sl_result setLidarPwm(const sl_cp0_set_lidarmotor_pwm_request_t& req, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO:
            struct _tx_req
            {
                sl_chassis_cmd_t cmd;
                sl_cp0_set_lidarmotor_pwm_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_CP0_CMD_SET_SL_LIDARMOTOR_PWM;
            memcpy(&tx_req.payload, &req, sizeof(req));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_CP0, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[10] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, 0, timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_CHASSIS_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }
 
            return SL_RESULT_OK;
        }


    private:
        
        sl_result  _sendCommand(sl_u8 cmd, const void * payload = nullptr, size_t payloadsize = 0 )
        {
            if (!_isConnected)
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            sl_u8 checksum = 0;
            sl_u16 length = payloadsize + 1;
            sl_u8 length_l = (sl_u8)(length & 0x00ff);
            sl_u8 length_h = (sl_u8)((length & 0xff00)>>8);


            std::vector<sl_u8> cmd_packet;
            cmd_packet.clear();

			_channel->flush();
            cmd_packet.push_back(SL_CHASSIS_CMD_SYNC_BYTE);
            cmd_packet.push_back(length_l);
            cmd_packet.push_back(length_h);
            cmd_packet.push_back(cmd);

            checksum ^= SL_CHASSIS_CMD_SYNC_BYTE;
            checksum ^= length_l;
            checksum ^= length_h;
            checksum ^= cmd;

            for (size_t pos = 0; pos < payloadsize; ++pos) 
            {
                checksum ^= ((sl_u8 *)payload)[pos];
                cmd_packet.push_back(((sl_u8 *)payload)[pos]);
            }

            cmd_packet.push_back(checksum);


            sl_u8 packet[1024];
            for (sl_u32 pos = 0; pos < cmd_packet.size(); pos++) 
            {
                packet[pos] = cmd_packet[pos];
            }
            _channel->clearReadCache();
            _channel->write(packet, cmd_packet.size());

            //delay(1);
            return SL_RESULT_OK;
        }

        template <typename T>
        sl_result _waitResponse(T &payload, sl_u32 size, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            if (!_isConnected)
            {
                return SL_RESULT_OPERATION_FAIL;
            }

            sl_u32 pkt_size = PKT_SIZE_EX_PAYLOAD + size;
            if (!_channel->waitForData(pkt_size, timeout)) 
            {
                return SL_RESULT_OPERATION_TIMEOUT;
            }
            _channel->read(reinterpret_cast<sl_u8 *>(&payload), pkt_size);
            return SL_RESULT_OK;
        }
       
       

    private:
        std::shared_ptr<IChannel> _channel;
        bool _isConnected;
        rp::hal::Locker         _lock;
        rp::hal::Event          _dataEvt;
        //rp::hal::Thread         _cachethread;
    };


    std::shared_ptr<ICp0Driver> createCp0Driver()
    {
        return std::make_shared<SlamtecCp0Driver>();
    }
}