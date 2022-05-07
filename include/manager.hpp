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

        void updateJointAngles();

        std::vector<int> anglesToPosition(std::vector<double> angle, int n);

        std::vector<double> positionToAngle(std::vector<int> position, int n);

    private:
        const std::string canDevice_ = "can0";
        std::string state_;
        const std::vector<std::string> states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};

        Claw claw_;
        Gait gait;
        InverseKinematics ik_;
        ForwardKinematics fk_;

        double commandValue_ = 0;
        double commandDirection_ = 0;
        double batteryVoltage_ = 0;
        double cprAngleRelation_;

        std::vector<double> jointAngles_ = {{0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0},
                                            {0, 0, 0}};
};

#endif  //  INCLUDE_MANAGER_HPP_