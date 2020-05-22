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

#include "common_types.h"
#include "config_motion.h"
#include "mmal_video.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/thread.h"

static const uint64_t kDrainProcessDelayMaximumInMicro = 32000;  // 32 ms

RaspiHttpImage::RaspiHttpImage(int width, int height, int framerate, int bitrate)
    : Event(false, false),
    clock_(webrtc::Clock::GetRealTimeClock()) {
    
    mv_shared_buffer_.reset(
        new rtc::BufferQueue(queue_capacity_, mv_queue_size_));
}

bool RaspiHttpImage::IsActive() const { return isRunning; }

bool RaspiHttpImage::StartCapture() {
    RTC_LOG(INFO) << "Raspi Http Image Starting";

    // Get the instance of MMAL encoder wrapper
    if ((mmal_encoder_ = webrtc::MMALWrapper::Instance()) == nullptr) {
        RTC_LOG(LS_ERROR) << "Failed to get MMAL encoder wrapper";
        return false;
    }

    // Set media config params at first.
    mmal_encoder_->SetMediaConfigParams();

    //
    // Overriding media config params for Http Image Video
    //
    // Enable InlineMotionVectors
    mmal_encoder_->SetInlineMotionVectors(true);

    // Setting Intra Frame period
    mmal_encoder_->SetIntraPeriod(framerate_ * VIDEO_INTRAFRAME_PERIOD);

    RTC_LOG(INFO) << "Initial Http Image Video : " << width_ << " x " << height_
                  << "@" << framerate_ << ", " << bitrate_ << " kbps";
    if (mmal_encoder_->InitEncoder(width_, height_, framerate_, bitrate_) ==
        false) {
        return false;
    }

    // start capture in here
    mmal_encoder_->StartCapture();

    // start drain thread ;
    if (!drainThread_) {
        RTC_LOG(INFO) << "Frame drain thread initialized.";
        drainThread_.reset(new rtc::PlatformThread(
            RaspiHttpImage::DrainThread, this, "FrameDrain", rtc::kHighPriority));
        drainThread_->Start();
        drainThreadStarted_ = true;
    }

    isRunning = true;
    return true;
}

void RaspiHttpImage::StopCapture() {
    isRunning = false;
    if (drainThread_) {
        drainThreadStarted_ = false;
        drainThread_->Stop();
        drainThread_.reset();
    }

    if (mmal_encoder_) {
        mmal_encoder_->StopCapture();
        mmal_encoder_->UninitEncoder();
        mmal_encoder_ = nullptr;
    }
}

RaspiHttpImage::~RaspiHttpImage() {
    RTC_LOG(INFO) << "Raspi Http Image Stopping";
    StopCapture();
}

///////////////////////////////////////////////////////////////////////////////
//
// Raspi Encoder frame drain processing
//
///////////////////////////////////////////////////////////////////////////////
void RaspiHttpImage::DrainThread(void *obj) {
    RaspiHttpImage *raspi_motion = static_cast<RaspiHttpImage *>(obj);
    while (raspi_motion->DrainProcess()) {
    }
}

bool RaspiHttpImage::DrainProcess() {
    uint64_t current_timestamp, timestamp;
    MMAL_BUFFER_HEADER_T *buf = nullptr;
    size_t length;
    RTC_LOG(INFO) << "Have new frame";
    current_timestamp = clock_->TimeInMicroseconds();
    buf = mmal_encoder_->GetEncodedFrame();
    if (buf && buf->length > 0) {
        bool is_keyframe = buf->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME;

        if (buf->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
            
            Set();  // Event Set to wake up Http Image vector process
        } else if (buf->flags & MMAL_BUFFER_HEADER_FLAG_FRAME) {
            
            RTC_DCHECK(buf->length == length);
        } else {
            RTC_LOG(LS_ERROR) << "**************************************";
            dump_mmal_buffer(0, buf);
            RTC_LOG(LS_ERROR) << "**************************************";
        }
    }

    timestamp = clock_->TimeInMicroseconds();
    if (timestamp - current_timestamp > kDrainProcessDelayMaximumInMicro)
        RTC_LOG(LS_ERROR) << "Frame DrainProcess Time : "
                          << timestamp - current_timestamp;
    if (buf) mmal_encoder_->ReleaseFrame(buf);
    // TODO: if encoded_size is zero, we need to reset encoder itself
    return true;
}
