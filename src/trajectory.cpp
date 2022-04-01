#include <trajectory.hpp>

Trajectory::Trajectory(){
}

Trajectory::~Trajectory(){
}

std::pair<double, double> Trajectory::baseTrajectory(double t){
    return std::make_pair<x, y>;
}

std::pair<double, double> Trajectory::legStanceTrajectory(double t) {
    return std::make_pair<x, y>;
}

std::pair<double, double> Trajectory::legSupportTrajectory(double t) {
    return std::make_pair<x, y>;
}