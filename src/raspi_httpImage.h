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

#ifndef RASPI_HTTPIMAGE_H_
#define RASPI_HTTPIMAGE_H_

#include <memory>
#include <vector>

#include "mmal_wrapper.h"
#include "raspi_httpnoti.h"
#include "raspi_motionfile.h"
#include "raspi_motionvector.h"
#include "rtc_base/buffer_queue.h"
#include "rtc_base/numerics/moving_average.h"
#include "rtc_base/platform_thread.h"
#include "system_wrappers/include/clock.h"

class RaspiHttpImage : public rtc::Event {
   public:
    explicit RaspiHttpImage(int width, int height, int framerate, int bitrate);
    explicit RaspiHttpImage();
    ~RaspiHttpImage();

    bool IsActive() const;

    // Motion Capture will use fixed resolution
    bool StartCapture();
    void StopCapture();

   private:
    static void DrainThread(void*);
    bool DrainProcess();
    bool isRunning;

    // Motion Capture params
    int width_, height_, framerate_, bitrate_;

    webrtc::MMALEncoderWrapper* mmal_encoder_;
    webrtc::Clock* const clock_;

    // Encoded frame process thread
    bool drainThreadStarted_;
    std::unique_ptr<rtc::PlatformThread> drainThread_;

    std::vector<uint8_t> imageBuff_;

    void GetMotionImage(uint8_t *buffer, int len);

    RTC_DISALLOW_COPY_AND_ASSIGN(RaspiHttpImage);
};

#endif  // RASPI_HTTPIMAGE_H_
