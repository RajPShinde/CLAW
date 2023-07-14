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
}

HardwareInterface::~HardwareInterface(){
}

bool HardwareInterface::init(ros::NodeHandle &rootNH, ros::NodeHandle &robotHardwareNH){

   // Acquire URDF
   std::string urdfString;
   if (urdfModel_ == nullptr) {
      urdfModel_ = std::make_shared<urdf::Model>();
   }
   rootNH.getParam("legged_robot_description", urdfString);

   if (!(!urdfString.empty() && urdfModel_->initString(urdfString))) {
      ROS_ERROR("URDF Setup Error");
      return false;
   } 

  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

   // Setup handlers
   setupJoints();
   setupImu();
   setupContactSensor();

   // Setup Odrive
   initializeOdrive();

   imuSubscriber_ = rootNH.subscribe("/imu_data", 1, &HardwareInterface::IMUCallback, this);

   return true;
}

void HardwareInterface::initializeOdrive(){
   if (!(master_.add_axis(0, "RF_HAA") && master_.add_axis(1, "RF_HFE") && master_.add_axis(2, "RF_KFE") && 
         master_.add_axis(3, "LF_HAA") && master_.add_axis(4, "LF_HFE") && master_.add_axis(5, "LF_KFE") &&
         master_.add_axis(6, "RH_HAA") && master_.add_axis(7, "RH_HFE") && master_.add_axis(8, "RH_KFE") && 
         master_.add_axis(9, "LH_HAA") && master_.add_axis(10, "LH_HFE") && master_.add_axis(11, "LH_KFE")))
   {
      ROS_ERROR_STREAM("Failed to create one or more axis. Aborting");
      return;
   }

   allAxis_ = {"RF_HAA", "RF_HFE", "RF_KFE",
               "LF_HAA", "LF_HFE", "LF_KFE",
               "RH_HAA", "RH_HFE", "RH_KFE",
               "LH_HAA", "LH_HFE", "LH_KFE"};

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
      master_.set_axis_requested_state(master_.axis(allAxis_[i]), odrive_can_ros::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
   }
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period){

   // Read Battery from Odrive
   master_.get_vbus_voltage(master_.axis("RF_HAA"));
   batteryVoltage_ = master_.axis("RF_HAA").vbus_voltage;

   // Read Joint Positions, Velocities & Torques from Odrive
   for(int i = 0; i<allAxis_.size(); i++){
      double reduction = (i%3 == 0 ? Claw::reductionHAA : Claw::reductionHFE);
      int direction = Claw::encoderDirection[i/3][i%3];
      jointData_[i].position = (master_.axis(allAxis_[i]).pos_enc_estimate - Claw::encoderOffset[i/3][i%3]) * direction * 2 * M_PI / reduction;
      jointData_[i].velocity = master_.axis(allAxis_[i]).vel_enc_estimate * 2 * M_PI * direction / reduction;
      jointData_[i].torque = master_.axis(allAxis_[i]).idq_second * Claw::kt * direction * reduction;
   }

   // Read Foot Contact Sensor
   contacts_.read(contactState_);

   std::vector<std::string> names = hybridJointInterface_.getNames();
   for (const auto& name : names) {
      HybridJointHandle handle = hybridJointInterface_.getHandle(name);
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(3.);
   }
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period){
   // Write Torque Commands to Odrive
   for(int i = 0; i<allAxis_.size(); i++){
      double command = jointData_[i].kp * (jointData_[i].positionDesired - jointData_[i].position) + 
                       jointData_[i].kd * (jointData_[i].velocityDesired - jointData_[i].velocity) + 
                       jointData_[i].ff;
      master_.set_input_torque(master_.axis(allAxis_[i]), command);
   }

   // Update Status LED's
   lights_.displayContactState(contactState_);
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
      legIndex = Claw::RF;
    else if (joint.first.find("LF") != std::string::npos)
      legIndex = Claw::LF;
    else if (joint.first.find("RH") != std::string::npos)
      legIndex = Claw::RH;
    else if (joint.first.find("LH") != std::string::npos)
      legIndex = Claw::LH;
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
    hybridJointInterface_.registerHandle(HybridJointHandle(stateHandle, &jointData_[index].positionDesired, &jointData_[index].velocityDesired,
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

