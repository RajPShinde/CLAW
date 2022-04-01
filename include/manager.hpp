#ifndef INCLUDE_MANAGER_HPP_
#define INCLUDE_MANAGER_HPP_

#include <gait.hpp>

class Manager {
    public:
        Manager();

        ~Manager();

        void begin();

        void walk();

        void reset();

    private:
        std::string state_;
        std::string direction_;
        const std::vector states_ = {"IDLE", "SIT", "WALK", "MOVE_BASE", "UNKNOWN"};
        const std::vector directions_ = {"F", "B", "L", "R", "S"};
        std::vector legPhase_ = {1, 1, 1, 1};

        Gait gaitX_;
        Gait gaitY_;
        InverseKinematics ik_;
        ForwardKinematics fk_;
};

#endif  //  INCLUDE_MANAGER_HPP_