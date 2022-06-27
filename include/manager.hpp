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


class Manager {
    public:
        Manager();

        ~Manager();

        int begin();

        std::vector<int> anglesToPosition(std::vector<double> angle, int n);

        std::vector<double> positionToAngle(std::vector<int> position, int n);

    private:
        Claw claw_;
        InverseKinematics ik_;
        ForwardKinematics fk_;

        double commandValue_ = 0;
        double commandDirection_ = 0;
        double batteryVoltage_ = 0;
        const std::string canDevice_ = "can0";
        std::string state_;
        std::vector<std::vector<double>> jointAngles_;
        const std::vector<std::string> states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};
};

#endif  //  INCLUDE_MANAGER_HPP_