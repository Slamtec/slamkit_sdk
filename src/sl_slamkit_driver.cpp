/*
 * Slamtec SLAMKIT SDK
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
#include "sl_slamkit_driver.h"
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

    class SlamtecSlamkitDriver :public ISlamkitDriver
    {
    public:
        SlamtecSlamkitDriver()
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

        sl_result getDeviceInfo(sl_slamkit_info_response_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            // TODO:
            struct _tx_req
            {
                sl_slamkit_cmd_t cmd;
                sl_slamkit_info_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_SLAMKIT_CMD_GET_DEVICE_INFO;
            memset(&tx_req.payload, 0, sizeof(sl_slamkit_info_request_t));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_SLAMKIT, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[1024] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(info), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_SLAMKIT_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            } 

            sl_slamkit_info_response_t* ppayload = (sl_slamkit_info_response_t*) (&data_buffer[4]);
            
            memcpy(&info, ppayload, sizeof(sl_slamkit_info_response_t));
            return SL_RESULT_OK;
        }

        sl_result set_motion_hit_and_get_imu_processed(const sl_slamkit_read_imu_processed_request_t& req, sl_slamkit_read_imu_processed_response_t& processed_data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            //
            struct _tx_req
            {
                sl_slamkit_cmd_t cmd;
                sl_slamkit_read_imu_processed_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_SLAMKIT_CMD_READ_IMU_PROCESSED;
            memcpy(&tx_req.payload, &req, sizeof(req));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_SLAMKIT, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            
            if (SL_IS_FAIL(ans)) return ans;

            sl_u8 data_buffer[1024] = {0};

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_slamkit_read_imu_processed_response_t), timeout);
            }

            if (SL_IS_FAIL(ans)) return ans;

            if ((data_buffer[0] != SL_SLAMKIT_CMD_SYNC_BYTE))
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
            
            sl_slamkit_read_imu_processed_response_t* ppayload = (sl_slamkit_read_imu_processed_response_t*) (&data_buffer[4]);
            
            memcpy(&processed_data, ppayload, sizeof(sl_slamkit_read_imu_processed_response_t));
 
            return SL_RESULT_OK;
        }

        sl_result getImuRawData(sl_imu_raw_data_t& imu_raw_data, sl_u32 timeout = DEFAULT_TIMEOUT)
        {
            //printf("test 0\r\n");
            // TODO: 
            struct _tx_req
            {
                sl_slamkit_cmd_t cmd;
                _sl_slamkit_read_imu_raw_request_t payload;

            }tx_req;

            tx_req.cmd.cmd = SL_SLAMKIT_CMD_READ_IMU_RAW;
            tx_req.payload.request_key = 0;
            // memcpy(&tx_req.payload, &req, sizeof(req));

            sl_result ans = SL_RESULT_OK;
            {
                rp::hal::AutoLocker l(_lock);
                ans = _sendCommand(CMD_CODE_SLAMTEC_SLAMKIT, 
                reinterpret_cast<const void *>(&tx_req), sizeof(tx_req));
            }
            //printf("test send ans = %08X\r\n", ans);
            if (SL_IS_FAIL(ans)) return ans;

            //printf("test send 0\r\n");

            sl_u8 data_buffer[1024] = {0};
            //sl_slamkit_read_imu_raw_response_t raw_resp;
            //memset(&raw_resp, 0, sizeof(raw_resp));

            {
                rp::hal::AutoLocker l(_lock);
                ans = _waitResponse(data_buffer, sizeof(sl_slamkit_read_imu_raw_response_t), timeout);
            }

            if (ans) return ans;

            if ((data_buffer[0] != SL_SLAMKIT_CMD_SYNC_BYTE))
            {
                return SL_RESULT_INVALID_DATA;
            }

            if ((data_buffer[3] == STATUS_CODE_ANS_RXERR))
            {
                return SL_RESULT_OPERATION_FAIL;
            }
            
            sl_slamkit_read_imu_raw_response_t* ppayload = (sl_slamkit_read_imu_raw_response_t*) (&data_buffer[4]);
            
            imu_raw_data.acc_x = ppayload->inertia_raw_data.acc.acc_x;
            imu_raw_data.acc_y = ppayload->inertia_raw_data.acc.acc_y;
            imu_raw_data.acc_z = ppayload->inertia_raw_data.acc.acc_z;
            imu_raw_data.gyro_x = ppayload->inertia_raw_data.gyro.gyro_x;
            imu_raw_data.gyro_y = ppayload->inertia_raw_data.gyro.gyro_y;
            imu_raw_data.gyro_z = ppayload->inertia_raw_data.gyro.gyro_z;
            imu_raw_data.mag_x = ppayload->compass_raw_data.hmc_x;
            imu_raw_data.mag_y = ppayload->compass_raw_data.hmc_y;
            imu_raw_data.mag_z = ppayload->compass_raw_data.hmc_z;
            imu_raw_data.timestamp = ppayload->inertia_raw_data.timestamp;
            
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
            cmd_packet.push_back(SL_SLAMKIT_CMD_SYNC_BYTE);
            cmd_packet.push_back(length_l);
            cmd_packet.push_back(length_h);
            cmd_packet.push_back(cmd);

            checksum ^= SL_SLAMKIT_CMD_SYNC_BYTE;
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


    std::shared_ptr<ISlamkitDriver> createSlamkitDriver()
    {
        return std::make_shared<SlamtecSlamkitDriver>();
    }
}