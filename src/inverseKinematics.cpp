#include <inverseKinematics.hpp>

InverseKinematics::InverseKinematics(){
}

InverseKinematics::~InverseKinematics(){
}

std::vector<double> InverseKinematics::computeJointAngles(double x, double y, double z, int n){
    
    double alpha, beta, gamma, r;
    r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

    if(n == (1 || 4))
        alpha = std::atan2(x, y) - std::atan2(std::sqrt(std::pow(r, 2) - std::pow(Claw::d, 2)), Claw::d);
    else
        alpha = std::atan2(x, y) + std::atan2(-1 * std::sqrt(std::pow(r, 2) - std::pow(Claw::d, 2)), -1 * Claw::d);

    double cosGamma = (std::pow(r - Claw::a, 2) + std::pow(z, 2) - std::pow(Claw::link1, 2) - std::pow(Claw::link2, 2))/(2 * Claw::link1 * Claw::link2);
    gamma = std::atan2(cosGamma, std::sqrt(1 - std::pow(cosGamma, 2)));

    beta = std::atan2(r - Claw::a, z) - std::atan2(Claw::link1 + Claw::link2 * std::cos(gamma), Claw::link2 * std::sin(gamma));

    return {alpha, beta, gamma};
}