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

//#include "sl_chassis_driver.h"
#include "sdkcommon.h"
//#include "hal/usb/libusb/include/libusb.h"
#include <libusb-1.0/libusb.h>
#include "hal/locker.h"
#include "hal/event.h"
//#include "hal/thread.h"
#include <string.h>
#include <thread>

namespace sl {
 
    static void transfer_cb_proc(libusb_transfer * data);

    class USBChannel : public IChannel
    {
         enum
        {
            MAX_RX_BUFFER_SIZE = 1224
        };

        enum {
            RX_TICKET_SLEEP_TIME = 3000,
            TX_TICKET_TIMEOUT    = 3000,
        };

    public:
        rp::hal::Event _rx_completion_signal;
        rp::hal::Event _tx_completion_signal;


    public:
        USBChannel(std::uint16_t venderId, std::uint16_t productId,
            std::uint16_t interfaceId, std::uint16_t txEndpoint, std::uint16_t rxEndpoint) 
            : _dev_handle(NULL)
            , _is_interface_clamed(false)
            , _rx_size(0)
            , _ctx(NULL)
            , _tx_libusb_transfer(NULL)
            , _rx_libusb_transfer(NULL)
        {
            _venderId = venderId;
            _productId = productId;
            _interfaceId = interfaceId;
            _txEndpoint = txEndpoint;
            _rxEndpoint = rxEndpoint;
        }

        ~USBChannel()
        {
            close();            
        }

        bool open()
        {
            int err = libusb_init(&_ctx);
            if(err != LIBUSB_SUCCESS )
            {
                printf("libusb_init wrong %x:%x->%d , handle: %x error msg:  %s \n",
                     _venderId, _productId, _interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));
                return false;
            }

            _dev_handle = libusb_open_device_with_vid_pid(_ctx, _venderId, _productId);	
            if(!_dev_handle)
            {
                printf("open my_device %x:%x wrong\n", _venderId, _productId);
                //libusb_exit(_ctx);
                return false;
            }
            else
            {
                printf("open my_device %x:%x ok, handle: %x \n",  _venderId, _productId, _dev_handle);
            }

            err = libusb_set_auto_detach_kernel_driver(_dev_handle, 1);
            if(err != LIBUSB_SUCCESS )
            {
                printf("libusb_set_auto_detach_kernel_driver wrong %x:%x->%d , handle: %x error msg:  %s \n",
                     _venderId, _productId, _interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));
                libusb_exit(_ctx);
                return false;
            }

            err = libusb_claim_interface(_dev_handle, _interfaceId);
            if(err != LIBUSB_SUCCESS )
            {
                printf("libusb_claim_interface wrong %x:%x->%d , handle: %x error msg:  %s \n",
                     _venderId, _productId, _interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));
                libusb_exit(_ctx);
                return false;
            }
            _is_interface_clamed = true;


            _tx_libusb_transfer = libusb_alloc_transfer(0);
            if (!_tx_libusb_transfer)
            {
                printf("libusb_alloc_transfer tx wrong...\n");
                libusb_exit(_ctx);
                return false;
            }

            _tx_libusb_transfer->callback = transfer_cb_proc;
            _tx_libusb_transfer->dev_handle = _dev_handle;
            _tx_libusb_transfer->user_data = this;
            _tx_libusb_transfer->buffer = NULL;
            _tx_libusb_transfer->length = 0;
            _tx_libusb_transfer->endpoint = _txEndpoint & 0x0f;
            _tx_libusb_transfer->type = LIBUSB_TRANSFER_TYPE_BULK;
            _tx_libusb_transfer->timeout = TX_TICKET_TIMEOUT;


            _rx_libusb_transfer = libusb_alloc_transfer(0);
            if (!_rx_libusb_transfer)
            {
                printf("libusb_alloc_transfer tx wrong...\n");
                libusb_exit(_ctx);
                return false;
            }

            _rx_libusb_transfer->callback = transfer_cb_proc;
            _rx_libusb_transfer->dev_handle = _dev_handle;
            _rx_libusb_transfer->user_data = this;
            _rx_libusb_transfer->buffer = _rx_buffer;
            _rx_libusb_transfer->length = 0;
            _rx_libusb_transfer->endpoint = _rxEndpoint & 0x80;
            _rx_libusb_transfer->type = LIBUSB_TRANSFER_TYPE_BULK;
            _rx_libusb_transfer->timeout = RX_TICKET_SLEEP_TIME;

            start_pipe_line();
            return true;
        }

        void close()
        {
            if (_is_pipe_working)
            {
                rp::hal::AutoLocker lock(_pipeline_lock);
                _is_pipe_working = false;
                _pipe_thread.join();
            }

            
            if (_tx_libusb_transfer)
            {
                libusb_free_transfer(_tx_libusb_transfer);
                _tx_libusb_transfer = NULL;
            }

            if (_rx_libusb_transfer)
            {
                libusb_free_transfer(_rx_libusb_transfer);
                _rx_libusb_transfer = NULL;
            }


            if (_is_interface_clamed)
            {
                int err = libusb_release_interface(_dev_handle, _interfaceId);

                if(err != LIBUSB_SUCCESS )
                {
                    printf("libusb_release_interface wrong %x:%x->%d , handle: %x error msg:  %s \n",
                     _venderId, _productId,_interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));
                }
            }
            _is_interface_clamed = false;


            if (_dev_handle)
            {
                libusb_close(_dev_handle);
                libusb_exit(_ctx);
            }
            _dev_handle = NULL;
            _ctx = NULL;
        }

        void flush()
        {
            //_rx_size = 0;
        }

        bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (size > MAX_RX_BUFFER_SIZE)
            {
                printf("usb channel rx data too big, max %d\n", MAX_RX_BUFFER_SIZE);
                return false;
            }
            

            int ready_size = 0;
            int err = libusb_bulk_transfer(_dev_handle, 0x80 + _rxEndpoint, (unsigned char*)_rx_buffer, size, &ready_size, timeoutInMs);
            if (err != LIBUSB_SUCCESS)
            {
                printf("libusb_bulk_transfer read wrong %x:%x->%d , handle: %x error msg:  %s \n",
                     _venderId, _productId,_interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));

                if (actualReady)
                {
                   * actualReady = 0;
                }
                
               
                //return false;
            }

            int offset = 0, rsize = 0;
            if (ready_size != size) 
            {
                offset = size - ready_size;
                err = libusb_bulk_transfer(_dev_handle, 0x80 + _rxEndpoint,(unsigned char*) _rx_buffer + offset, offset, &rsize, timeoutInMs);
                
                if (actualReady)
                {
                    * actualReady = offset + rsize;
                   
                }
                 _rx_size = offset + rsize;
                
                if (err != LIBUSB_SUCCESS)
                {
                    printf("libusb_bulk_transfer read wrong 2: %x:%x->%d , handle: %x error msg:  %s \n",
                        _venderId, _productId,_interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));

                    
                    return false;
                }
            }
            else
            {
                if (actualReady)
                {
                    * actualReady = ready_size;
                }
                _rx_size = ready_size;
            }

            

            return true;
        }

        int write(const void* data, size_t size)
        {
            int actual_size = 0;

           do {
                rp::hal::AutoLocker locker(_op_lock);
                if (_tx_libusb_transfer)
                {
                    _tx_libusb_transfer->buffer = (unsigned char *)data;
                    _tx_libusb_transfer->length = size;


                    _tx_completion_signal.set(false);

                    int err = libusb_submit_transfer(_tx_libusb_transfer);
                    if (err != LIBUSB_SUCCESS)
                    {
                        printf("libusb_submit_transfer tx wrong, error msg:  %s \n", libusb_strerror((enum libusb_error)err));
                        libusb_cancel_transfer(_tx_libusb_transfer);
                        break;
                    }


                    if(rp::hal::Event::EVENT_OK != _tx_completion_signal.wait())
                    {
                        printf("libusb_submit_transfer tx wait completion wrong\r\n");
                        libusb_cancel_transfer(_tx_libusb_transfer);
                        break;
                    }

                    switch (_tx_libusb_transfer->status)
                    {
                        case LIBUSB_TRANSFER_COMPLETED:
                        {
                            actual_size = _tx_libusb_transfer->length;
                            break;
                        }
                        case LIBUSB_TRANSFER_STALL:
                        {
                            libusb_clear_halt(_dev_handle, _txEndpoint);
                            break;
                        }
                        default:
                        {
                            printf("libusb_submit_transfer tx wrong, status: \r\n", _tx_libusb_transfer->status);
                            break;
                        }
                    }
                }
            }while(0);
            
            

#if 0
            
            int err = libusb_bulk_transfer(_dev_handle, _txEndpoint, (unsigned char*)data, size, &ready_size, 1000);
            if (err != LIBUSB_SUCCESS)
            {
                printf("libusb_bulk_transfer write wrong %x:%x->%d , handle: %x error msg:  %s \n",
                     _venderId, _productId,_interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));                
               
                return SL_RESULT_OPERATION_FAIL;
            }

            int offset = 0, rsize = 0;
            if (ready_size != size) 
            {
                offset = size - ready_size;
                err = libusb_bulk_transfer(_dev_handle, _txEndpoint,(unsigned char*) data + offset, offset, &rsize, 1000);
                if (err != LIBUSB_SUCCESS)
                {
                    printf("libusb_bulk_transfer read wrong 2: %x:%x->%d , handle: %x error msg:  %s \n",
                        _venderId, _productId,_interfaceId, _dev_handle,  libusb_strerror((enum libusb_error)err));

                    return ready_size;
                }

                if ((offset + rsize) != size)
                {
                    return offset + rsize;
                }
                
            }
#endif
            return actual_size;
        }

        int read(void* buffer, size_t size)
        {
            //size_t lenRec = 0;
            if (size <= _rx_size)
            {
                memcpy(buffer, _rx_buffer, size);
                _rx_size -= size;

                return size;
            }
            else
            {
                memcpy(buffer, _rx_buffer, _rx_size);
                _rx_size = 0;

                return _rx_size;
            }
        }

        void clearReadCache() 
        {
            _rx_size = 0;
        }

        void setStatus(uint32_t flag){}

    private:
        int pipeline_handler() 
        {
            while (_is_pipe_working) 
            {
                timeval tv;
                tv.tv_sec = 1;
                tv.tv_usec = 00;
                if (libusb_handle_events_timeout(_ctx, &tv)) 
                {
                    // function failed, sleep for a while to prevent 100% cpu usage
                    delay(100);
                }
            }
            return 0;
        }

        void start_pipe_line()
        {
            rp::hal::AutoLocker lock(_pipeline_lock);
            if (_is_pipe_working) return;

            _pipe_thread = std::thread(std::bind(&USBChannel::pipeline_handler, this));


            //_pipe_thread = CLASS_THREAD(USBChannel, pipeline_handler);
            //_pipe_thread = rp::hal::Thread::create((std::bind(&USBChannel::pipeline_handler, this)), NULL);
            
            //_pipe_thread = rp::hal::Thread::create(std::move(std::bind(&USBChannel::pipeline_handler, this)));
            //_pipe_thread.setName("USBPipeline");
            //_pipe_thread.setPriority(rp::hal::Thread::PRIORITY_HIGH)


            _is_pipe_working = true;
        }



    private:
        libusb_device_handle *_dev_handle;
        libusb_context *	_ctx;

        bool    _is_interface_clamed;
        std::uint16_t _venderId;
        std::uint16_t _productId;
        std::uint16_t _interfaceId;
        std::uint16_t _txEndpoint;
        std::uint16_t _rxEndpoint;

        std::uint16_t _rx_size;
        std::uint8_t _rx_buffer[MAX_RX_BUFFER_SIZE];

        rp::hal::Locker _op_lock;
        //rp::hal::Event _rx_completion_signal;
        //rp::hal::Event _tx_completion_signal;

        libusb_transfer * _rx_libusb_transfer;
        libusb_transfer * _tx_libusb_transfer;

        rp::hal::Locker _pipeline_lock;
        bool _is_pipe_working;
        bool _is_rx_thread_working;
        std::thread _pipe_thread;
        std::thread _rx_thread;

    };

    static void transfer_cb_proc(libusb_transfer * data)
    {
        if(!data->user_data) return;


        USBChannel * chan = reinterpret_cast<USBChannel *>(data->user_data);

        if (data->endpoint & 0x80)
        {
            chan->_rx_completion_signal.set();
        }
        else
        {
            chan->_tx_completion_signal.set();
        }
    }


    std::shared_ptr<IChannel> createUSBChannel(std::uint16_t venderId, std::uint16_t productId,
            std::uint16_t interfaceId, std::uint16_t txEndpoint, std::uint16_t rxEndpoint)
    {
        return std::make_shared<USBChannel>(venderId, productId, interfaceId, txEndpoint, rxEndpoint);
    }
}