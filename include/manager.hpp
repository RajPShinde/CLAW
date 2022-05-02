#ifndef INCLUDE_MANAGER_HPP_
#define INCLUDE_MANAGER_HPP_

#include <thread>
#include <string>
#include <iostream>
#include <gait.hpp>

class Manager {
    public:
        Manager();

        ~Manager();

        void begin();

        void walk();

        void reset();

        void updateJointAngles();

    private:
        std::string canDevice_ = "can0";
        std::string state_;
        std::string direction_;
        const std::vector states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};

        Gait gait;
        InverseKinematics ik_;
        ForwardKinematics fk_;

        std::map<std::string, std::vector<int>> jointAngles_ = {{"leg1", {0, 0, 0}},
                                                                {"leg2", {0, 0, 0}},
                                                                {"leg3", {0, 0, 0}},
                                                                {"leg4", {0, 0, 0}}};

        std::map<std::string, std::vector<int>> encoderValues_ = {{"leg1", {0, 0, 0}},
                                                                  {"leg2", {0, 0, 0}},
                                                                  {"leg3", {0, 0, 0}},
                                                                  {"leg4", {0, 0, 0}}};

        double batteryVoltage_ = 0;
};

#endif  //  INCLUDE_MANAGER_HPP_