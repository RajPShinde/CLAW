#ifndef INCLUDE_VELOCITYKINEMATICS_HPP_
#define INCLUDE_VELOCITYKINEMATICS_HPP_

#include <math>
#include <claw.hpp>
#include <inverseKinematics.hpp>

class VelocityKinematics {
    public:
        VelocityKinematics();

        ~VelocityKinematics();

        std::vector<double> jacobian();

    private:
        Claw claw_;
};

#endif  //  INCLUDE_VELOCITYKINEMATICS_HPP_