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

   // Acquire URDF
   // std::string urdfString;
   // if (urdfModel_ == nullptr) {
   //    urdfModel_ = std::make_shared<urdf::Model>();
   // }
   // nh.getParam("legged_robot_description", urdfString);
   // return !urdfString.empty() && urdfModel_->initString(urdfString);

   // if (!loadUrdf(root_nh)) {
   //    ROS_ERROR("Error occurred while setting up urdf");
   //    return false;
   // }  

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

   // Read Battery from Odrive
   master_.get_vbus_voltage(allAxis_[0]);
   batteryVoltage_ = allAxis_[0].vbus_voltage;
   // Read Joint Positions & Velocities from Odrive

   // Read Foot Contact Sensor
   contacts_.read();
}

void HardwareInterface::write(){
   // Write Torque Commands to Odrive
   for(int i = 0; i<allAxis_.size(); i++){
      double command = jointData_[i].kp * (jointData_[i].positionDesired - jointData_[i].position) + 
                       jointData_[i].kd * (jointData_[i].velocityDesired - jointData_[i].velocity) + 
                       jointData_[i].ff;
      master_.set_input_pos(allAxis_[i], command);
   }

   // Update Status LED's
   lights_.display();
}

void HardwareInterface::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imuData_.orientation[0] = msg->orientation.x;
  imuData_.orientation[1] = msg->orientation.y;
  imuData_.orientation[2] = msg->orientation.z;
  imuData_.orientation[3] = msg->orientation.w;
  imuData_.angularVelocity[0] = msg->angular_velocity.x;
  imuData_.angularVelocity[1] = msg->angular_velocity.y;
  imuData_.angularVelocity[2] = msg->angular_velocity.z;
  imuData_.linearAcceleration[0] = msg->linear_acceleration.x;
  imuData_.linearAcceleration[1] = msg->linear_acceleration.y;
  imuData_.linearAcceleration[2] = msg->linear_acceleration.z;
}

void HardwareInterface::setupJoints(){
  for (const auto& joint : urdfModel_->joints_) {

    int legIndex = 0;
    int jointIndex = 0;

    if (joint.first.find("RF") != std::string::npos)
      legIndex = Claw::FR;
    else if (joint.first.find("LF") != std::string::npos)
      legIndex = Claw::FL;
    else if (joint.first.find("RH") != std::string::npos)
      legIndex = Claw::RR;
    else if (joint.first.find("LH") != std::string::npos)
      legIndex = Claw::RL;
    else
      continue;

    if (joint.first.find("HAA") != std::string::npos)
      jointIndex = 0; 
    else if (joint.first.find("HFE") != std::string::npos)
      jointIndex = 1;
    else if (joint.first.find("KFE") != std::string::npos)
      jointIndex = 2;
    else
      continue;

    int index = legIndex * 3 + jointIndex;
    hardware_interface::JointStateHandle stateHandle(joint.first, &jointData_[index].position, &jointData_[index].velocity,
                                                      &jointData_[index].torque);
    jointStateInterface_.registerHandle(stateHandle);
    hybridJointInterface_.registerHandle(HybridJointHandle(&jointData_[index].positionDesired, &jointData_[index].velocityDesired,
                                                           &jointData_[index].kp, &jointData_[index].kd, &jointData_[index].ff));
  }
}

void HardwareInterface::setupImu(){
   imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("imu", "imu_link", imuData_.orientation, imuData_.oorientationCovariance,
                                                                           imuData_.angularVelocity, imuData_.angularVelocityCovarinace, 
                                                                           imuData_.linearAcceleration, imuData_.linearAccelerationCovariance));
}

void HardwareInterface::setupContactSensor(){
   for (size_t i = 0; i < 4; ++i) {
      contactSensorInterface_.registerHandle(ContactSensorHandle(Claw::contactSensors[i], &contactState_[i]));
   }
}

