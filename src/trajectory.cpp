#include <trajectory.hpp>

Trajectory::Trajectory(Gait &obj): gait_(obj){
}

Trajectory::~Trajectory(){
}

Eigen::Vector3d Trajectory::jerkMinimizedTrajectory(double x0, double y0, double z0, double xT, double yT, double zT, double t){
    Polynomial pX(x0, 0, 0, xT, 0, 0, t);
    Polynomial pY(y0, 0, 0, yT, 0, 0, t);
    Polynomial pZ(z0, 0, 0, zT, 0, 0, t);

    Eigen::Vector3d position;
    position << pX.position(t), pY.position(t), pZ.position(t);
    return position;
}

Eigen::Vector3d Trajectory::stancePhaseTrajectory(double t) {
    double x, y, c2, c3;
    if(t>=0 && t<=gait_.te)
        x = ((-1*gait_.stride*gait_.te*std::sin(M_PI*t/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*t)/(2*gait_.tm)) + c2;
    else if(t>=gait_.te && t<=gait_.te + gait_.tn)
        x = gait_.stride * (((t-gait_.te)/gait_.tn) - (std::sin(2*M_PI*(t-gait_.te)/gait_.tn)/(2*M_PI))) - gait_.stride/2;
    else if(t>=gait_.te + gait_.tn && t<=gait_.tm)
        x = ((gait_.stride*gait_.te*std::sin(M_PI*(t-gait_.te-gait_.tn)/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*(t-gait_.te-gait_.tn))/(2*gait_.tm)) + c3;

    if(t>=0 && t<gait_.tm/2)
        y = 2*gait_.height * ((t/gait_.tm) - (std::sin(4*M_PI*t/gait_.tm)/(4*M_PI)));
    else if(t>=gait_.tm/2 && t<gait_.tm)
        y = 2*gait_.height * (1 - (t/gait_.tm) + (std::sin(4*M_PI*t/gait_.tm)/(4*M_PI)));

    Eigen::Vector3d position;
    position << x, y, 0;
    return position;
}

Eigen::Vector3d Trajectory::supportPhaseTrajectory(double t) {
    double xd = gait_.stride / gait_.tm;
    double x = xd*t - gait_.stride/2;
    double y = 0;

    Eigen::Vector3d position;
    position << x, y, 0;
    return position;
}