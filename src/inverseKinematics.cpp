#include <inverseKinematics.hpp>

InverseKinematics::InverseKinematics(){
}

InverseKinematics::~InverseKinematics(){
}

std::vector<double> InverseKinematics::computeJointAngles(double x, double y, double z, int n){
    
    double alpha, beta, gamma, r;
    r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

    if(n == (1 || 4))
        alpha = std::atan2(y, x) - std::atan2(Claw::d, std::sqrt(std::pow(r, 2) - std::pow(Claw::d, 2)));
    else
        alpha = std::atan2(y, x) + std::atan2( -1 * Claw::d, std::sqrt(std::pow(r, 2) - std::pow(Claw::d, 2)));

    double cosGamma = (std::pow(r - Claw::a, 2) + std::pow(z, 2) - std::pow(Claw::link1, 2) - std::pow(Claw::link2, 2))/(2 * Claw::link1 * Claw::link2);
    gamma = std::atan2(-std::sqrt(1 - std::pow(cosGamma, 2)), cosGamma);

    beta = std::atan2(z, r - Claw::a) - std::atan2(Claw::link2 * std::sin(gamma), Claw::link1 + Claw::link2 * std::cos(gamma));

    return {alpha, beta, gamma};
}