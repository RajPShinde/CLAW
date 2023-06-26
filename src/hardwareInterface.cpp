/*
BSD 3-Clause License

Copyright (c) 2023, Raj Shinde
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
 * @file hardwareInterface.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * @copyright BSD 3-Clause License, Copyright (c) 2023
 * 
 */

#include <hardwareInterface.hpp>

HardwareInterface::HardwareInterface(){
   initialize();
}

HardwareInterface::~HardwareInterface(){
  
}

void HardwareInterface::initialize(){

   // Setup handlers
   setupJoints();
   setupImu();
   setupContactSensor();

   // Setup Odrive
   initializeOdrive();
}

void HardwareInterface::initializeOdrive(){
   if (!(master_.add_axis(0, "KF1") && master_.add_axis(1, "HF1") &&
         master_.add_axis(2, "HA2") && master_.add_axis(3, "HA1") &&
         master_.add_axis(4, "KF2") && master_.add_axis(5, "HF2") &&
         master_.add_axis(6, "KF3") && master_.add_axis(7, "HF3") &&
         master_.add_axis(8, "HA4") && master_.add_axis(9, "HA3") &&
         master_.add_axis(10, "KF4") && master_.add_axis(11, "HF4")))
   {
      ROS_ERROR_STREAM("Failed to create one or more axis. Aborting");
      return;
   }

   allAxis_ = {master_.axis("HA1"), master_.axis("HF1"), master_.axis("KF1"),
              master_.axis("HA2"), master_.axis("HF2"), master_.axis("KF2"),
              master_.axis("HA3"), master_.axis("HF3"), master_.axis("KF3"),
              master_.axis("HA4"), master_.axis("HF4"), master_.axis("KF4")};

   // Create Interface to SocketCAN 
   can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();
   if (!driver->init(Claw::ODRIVE_PORT, 0, can::NoSettings::create()))
   {
      ROS_ERROR_STREAM("Failed to initialize can device");
      return;
   }
   can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
      [&driver](const can::State& s) {
         std::string err;
         driver->translateError(s.internal_error, err);
         ROS_ERROR_STREAM("CAN Device error");
                     fprintf(stderr, "CAN Device error: %s, asio: %s.\n", 
               err.c_str(), s.error_code.message().c_str());
      }
   );

   master_.init(driver);

   // Start Torque Control on All Axis
   for(int i = 0; i<allAxis_.size(); i++){
      master_.set_axis_requested_state(allAxis_[i], odrive_can_ros::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
   }
}

void HardwareInterface::read(){

}

void HardwareInterface::write(){

}

void HardwareInterface::IMUCallback(){
   
}

void HardwareInterface::setupJoints(){

}

void HardwareInterface::setupImu(){
   imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("imu", "imu", imuData_.orientation, imuData_.oorientationCovariance,
                                                                           imuData_.angularVelocity, imuData_.angularVelocityCovarinace, 
                                                                           imuData_.linearAcceleration, imuData_.linearAccelerationCovariance));
}

void HardwareInterface::setupContactSensor(){

}

