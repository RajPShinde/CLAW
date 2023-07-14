/*
BSD 3-Clause License

Copyright (c) 2022, Raj Shinde
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file manager.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * @copyright BSD 3-Clause License, Copyright (c) 2023
 * 
 */

#include <manager.hpp>

Manager::Manager(ros::NodeHandle &rootNH, std::shared_ptr<HardwareInterface> interface) : interface_(interface), controlThreadStatus_(true) {
    
    lastTime_ = std::chrono::high_resolution_clock::now();

    controlThread_ = std::thread([&]() {
        while (controlThreadStatus_) {
            control();
        }
    });

    sched_param sched{.sched_priority = threadPriority_};

    if (pthread_setschedparam(controlThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
        ROS_WARN_STREAM("Failed to set threads priority");
    }
}

Manager::~Manager(){
    controlThreadStatus_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}

void Manager::control(){
    const std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<double> timeDelay(1.0/frequency_);

    std::chrono::duration<double> timeSpent = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime_);
    period_ = ros::Duration(timeSpent.count());
    lastTime_ = currentTime;

    const double cycleTimeError = (period_ - ros::Duration(timeDelay.count())).toSec();
    if (cycleTimeError > cycleTimeErrorThreshold_) {
        ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycleTimeError - cycleTimeErrorThreshold_ << "s");
    }

    // Read
    interface_->read(ros::Time::now(), period_);

    // Control

    // Write
    interface_->write(ros::Time::now(), period_);

    const auto delay = currentTime + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(timeDelay);
    std::this_thread::sleep_until(delay);
}