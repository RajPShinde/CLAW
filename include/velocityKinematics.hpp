#ifndef INCLUDE_VELOCITYKINEMATICS_HPP_
#define INCLUDE_VELOCITYKINEMATICS_HPP_

#include <math>
#include <claw.hpp>
#include <forwardKinematics.hpp>

class VelocityKinematics {
    public:
        VelocityKinematics();

        ~VelocityKinematics();

        Eigen::MatrixXd jacobian(std::vector<double> JointAngles, int n);

        Eigen::Vector3d footVelocities(Eigen::Vector3d jointVelocities, std::vector<double> JointAngles, int n);

    private:
        Claw claw_;
};

#endif  //  INCLUDE_VELOCITYKINEMATICS_HPP_