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

        Eigen::VectorXd footVelocities(Eigen::Vector3d jointVelocities, std::vector<double> JointAngles, int n);

        Eigen::VectorXd reactionForces(Eigen::Vector3d jointTorques, std::vector<double> JointAngles, int n);
};

#endif  //  INCLUDE_VELOCITYKINEMATICS_HPP_