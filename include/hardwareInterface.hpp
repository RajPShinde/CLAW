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
 * @file hardwareInterface.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * @copyright BSD 3-Clause License, Copyright (c) 2023
 * 
 */

#ifndef INCLUDE_HARDWAREINTERFACE_HPP_
#define INCLUDE_HARDWAREINTERFACE_HPP_

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <urdf/model.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <socketcan_interface/socketcan.h>
#include <sensor_msgs/Imu.h>

#include <contactSensorInterface.hpp>
#include <contactSensorHandle.hpp>
#include <hybridJointInterface.hpp>
#include <hybridJointHandle.hpp>
#include <odrive_can_ros/ODriveEnums.h>
#include <odrive_can_ros/can_simple.hpp>
#include <claw.hpp>
#include <data.hpp>
#include <contactSensor.hpp>
#include <status.hpp>

class HardwareInterface : public hardware_interface::RobotHW {

   public:
         HardwareInterface();
        
         ~HardwareInterface();

         bool init(ros::NodeHandle &rootNH, ros::NodeHandle &robotHardwareNH);

         void initializeOdrive();

         void read(const ros::Time& time, const ros::Duration& period);

         void write(const ros::Time& time, const ros::Duration& period);

         void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);

         void setupJoints();

         void setupImu();

         void setupContactSensor();

   private:
         hardware_interface::JointStateInterface jointStateInterface_;
         hardware_interface::ImuSensorInterface imuSensorInterface_;
         HybridJointInterface hybridJointInterface_;
         ContactSensorInterface contactSensorInterface_;

         ros::Subscriber imuSubscriber_;
         
         std::shared_ptr<urdf::Model> urdfModel_;
         odrive_can_ros::CANSimple master_;
         std::vector<odrive_can_ros::ODriveAxis> allAxis_;

         ContactSensor contacts_;
         Status lights_;

         ImuData imuData_;
         ActuatorData jointData_[12];
         bool contactState_[4];

         double batteryVoltage_;

};

#endif  //  INCLUDE_HARDWAREINTERFACE_HPP_