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
 * @file manager.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_MANAGER_HPP_
#define INCLUDE_MANAGER_HPP_

#include <ros/ros.h>
#include <thread>
#include <string>
#include <iostream>
#include <vector>
#include <claw.hpp>
#include <inverseKinematics.hpp>
#include <forwardKinematics.hpp>
#include <trajectory.hpp>
#include <odrive_can_ros/ODriveEnums.h>
#include <odrive_can_ros/can_simple.hpp>
#include <socketcan_interface/socketcan.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class Manager {
    public:
        Manager(ros::NodeHandle &nh);

        ~Manager();

        int init();

        int begin();

        std::vector<int> anglesToPosition(std::vector<double> angle, int n);

        std::vector<double> positionToAngle(std::vector<int> position, int n);
        
        void move();

        void poseManipulation(odrive_can_ros::CANSimple &master);

        double offsetTime(double timeReference, int phaseReference, int &phasePair, double delay, Gait gait);

        void initializeOdrives(odrive_can_ros::CANSimple &master);

        void commandOdrives(odrive_can_ros::CANSimple &master);

        void idleOdrives(odrive_can_ros::CANSimple &master);

        void statePublisher(std::vector<double> l1, std::vector<double> l2, std::vector<double> l3, std::vector<double> l4);

    private:
        bool managerStatus = true;
        double commandValue_ = 1;
        double commandDirection_ = 1;
        double batteryVoltage_ = 0;
        std::string state_ = "MOVE_BASE";
        std::vector<std::vector<double>> jointAngles_;
        const std::vector<std::string> states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};
        std::vector<std::vector<int>> encoderShadowCount_;
        std::vector<odrive_can_ros::ODriveAxis> allAxis;

        ros::Publisher jointStatePublisher_;
        ForwardKinematics fk_;

        Pose base = {0, 0, 0};
};

#endif  //  INCLUDE_MANAGER_HPP_