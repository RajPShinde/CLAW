#ifndef INCLUDE_MANAGER_HPP_
#define INCLUDE_MANAGER_HPP_

#include <thread>
#include <string>
#include <iostream>
#include <vector>
#include <gait.hpp>

class Manager {
    public:
        Manager();

        ~Manager();

        void begin();

        std::vector<int> anglesToPosition(std::vector<double> angle, int n);

        std::vector<double> positionToAngle(std::vector<int> position, int n);

    private:
        Claw claw_;
        Gait gait;
        InverseKinematics ik_;
        ForwardKinematics fk_;

        double commandValue_ = 0;
        double commandDirection_ = 0;
        double batteryVoltage_ = 0;
        const double cprAngleRelation_ = claw_.countsPerRevolution * claw_.gearReduction / 360;
        const std::string canDevice_ = "can0";
        std::string state_;
        std::vector<double> jointAngles_;
        const std::vector<std::string> states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};
};

#endif  //  INCLUDE_MANAGER_HPP_