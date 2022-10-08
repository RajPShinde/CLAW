/**
 * @file manager.hpp
 * @author Raj Shinde (rajprakashshinde07@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef INCLUDE_MANAGER_HPP_
#define INCLUDE_MANAGER_HPP_

#include <ros/ros.h>
#include <thread>
#include <string>
#include <iostream>
#include <vector>
#include <gait.hpp>
#include <claw.hpp>
#include <interface.hpp>
#include <inverseKinematics.hpp>
#include <forwardKinematics.hpp>
#include <trajectory.hpp>
#include <odrive_can_ros/can_simple.hpp>
#include <socketcan_interface/socketcan.h>
#include <sensor_msgs/JointState.h>

class Manager {
    public:
        Manager(ros::NodeHandle &nh);

        ~Manager();

        int begin();

        std::vector<int> anglesToPosition(std::vector<double> angle, int n);

        std::vector<double> positionToAngle(std::vector<int> position, int n);

        void statePublisher(std::vector<double> l1, std::vector<double> l2, std::vector<double> l3, std::vector<double> l4);

    private:
        bool managerStatus = true;
        double commandValue_ = 0;
        double commandDirection_ = 0;
        double batteryVoltage_ = 0;
        const std::string canDevice_ = "can0";
        std::string state_;
        std::vector<std::vector<double>> jointAngles_;
        const std::vector<std::string> states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};
        std::vector<std::string> axisNames = {"HA1", "HF1", "KF1", 
                                              "HA2", "HF2", "KF2", 
                                              "HA3", "HF3", "KF3", 
                                              "HA4", "HF4", "KF4"};
        std::vector<std::vector<int>> encoderShadowCount_;

        ros::Publisher jointStatePublisher;
};

#endif  //  INCLUDE_MANAGER_HPP_