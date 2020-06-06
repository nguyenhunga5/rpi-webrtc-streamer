/*
Copyright (c) 2017, rpi-webrtc-streamer Lyu,KeunChang

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "raspi_httpImage.h"

#include <limits>
#include <string>
#include <fstream>

#include "common_types.h"
#include "config_motion.h"
#include "mmal_video.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/thread.h"

static const float kKushGaugeConstant = 0.07;
static const uint64_t kDrainProcessDelayMaximumInMicro = 32000;  // 32 ms
static const int kEventWaitPeriod = 5;

RaspiHttpImage::RaspiHttpImage(int width, int height)
    :
    width_(width),
    height_(height)
    {
    
}

RaspiHttpImage::~RaspiHttpImage() {
    RTC_LOG(INFO) << "Raspi Http Image Stopping";
}

template <typename T>
std::vector<T> operator+(const std::vector<T> &A, const std::vector<T> &B)
{
    std::vector<T> AB;
    AB.reserve( A.size() + B.size() );                // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );        // add A;
    AB.insert( AB.end(), B.begin(), B.end() );        // add B;
    return AB;
}

template <typename T>
std::vector<T> &operator+=(std::vector<T> &A, const std::vector<T> &B)
{
    A.reserve( A.size() + B.size() );                // preallocate memory without erase original data
    A.insert( A.end(), B.begin(), B.end() );         // add B;
    return A;                                        // here A could be named AB
}

void RaspiHttpImage::addBuffer(void *buf) {
    size_t length;
    
    MMAL_BUFFER_HEADER_T *dataBuff = (MMAL_BUFFER_HEADER_T *)buf;

    if (dataBuff && dataBuff->length > 0) {
        bool is_keyframe = dataBuff->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME;

        if (dataBuff->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
            // queuing motion vector for motion analysis
            
        } else if (dataBuff->flags & MMAL_BUFFER_HEADER_FLAG_FRAME) {
            if (is_keyframe) {
                size_t bytes = imageBuff_.size();
                if (bytes > 0) {
                    RTC_LOG(INFO) << "Have image, size " << bytes << " bytes, write to file";
                    uint8_t *buffer = imageBuff_.data();
                    const char *name = "image.frame";
                    FILE *pFile;
                    pFile = fopen(name, "wb");
                    fwrite(buffer, sizeof(uint8_t), bytes, pFile);
                    fclose(pFile);
                }

                RTC_LOG(INFO) << "Is Key frame, do clear";
                imageBuff_.clear();
            }
            
            std::vector<uint8_t> data(dataBuff->data, dataBuff->data + dataBuff->length);
            imageBuff_ += data;
            RTC_DCHECK(dataBuff->length == length);
        } else {
            RTC_LOG(LS_ERROR) << "**************************************";
            dump_mmal_buffer(0, dataBuff);
            RTC_LOG(LS_ERROR) << "**************************************";
        }
    }
}

void RaspiHttpImage::clear() {
    imageBuff_.clear();
}