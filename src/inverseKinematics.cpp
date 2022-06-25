#include <inverseKinematics.hpp>

InverseKinematics::InverseKinematics(){
}

InverseKinematics::~InverseKinematics(){
}

std::vector<double> InverseKinematics::computeJointAngles(double x, double y, double z, int n){
    double r = std::sqrt(std::pow(x, 2) + std::pow(y, 2) - std::pow(claw_.d, 2));
    
    double alpha = std::atan2(x, y) + std::atan2(-1 * std::sqrt(std::pow(r, 2) - std::pow(claw_.d, 2)), -claw_.d);

    double cosGamma = (std::pow(x, 2) + std::pow(y, 2) - std::pow(claw_.d, 2) + std::pow(z, 2) - std::pow(claw_.link1, 2) - std::pow(claw_.link2, 2))/(2 * claw_.link1 * claw_.link2);
    double gamma = std::atan2(cosGamma, std::sqrt(1 - std::pow(cosGamma, 2)));

    double beta = std::atan2(r, z) - std::atan2(claw_.link1 + claw_.link2 * std::cos(gamma), claw_.link2 * std::sin(gamma));

    return {alpha, beta, gamma};
}