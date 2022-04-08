#include <trajectory.hpp>

Trajectory::Trajectory(Gait &obj): gait_(obj){
}

Trajectory::~Trajectory(){
}

std::pair<double, double> Trajectory::jerkMinimizedTrajectory(double x0, double y0, double xt, double yT, double t){
    Polynomial pX(x0, 0, 0, xt, 0, 0);
    Polynomial pY(y0, 0, 0, yt, 0, 0);

    double x = pX.position(t);
    double y = pY.position(t);

    return std::make_pair<x, y>;
}

std::pair<double, double> Trajectory::legStanceTrajectory(double t) {
    if(t>=0 && t<=gait_.te)
        double x = ((-1*gait_.stride*gait_.te*std::sin(PI_*t/gait_.te))/(2*gait_.tm*PI_)) - ((gait_.stride*t)/(2*gait_.tm)) + c2;
    else if(t>=gait_.te && t<=gait_.te + gait_.tn)
        double x = gait_.stride * (((t-gait_.te)/gait_.tn) - (std::sin(2*PI_*(t-gait_.te)/gait_.tn)/(2*PI_))) - gait_.stride/2;
    else if(t>=gait_.te + gait_.tn && t<=gait_.tm)
        double x = ((gait_.stride*gait_.te*std::sin(PI_*(t-gait_.te-gait_.tn)/gait_.te))/(2*gait_.tm*PI_)) - ((gait_.stride*(t-gait_.te-gait_.tn))/(2*gait_.tm)) + c3;

    if(t>=0 && t<gait_.tm/2)
        double y = 2*gait_.height * ((t/gait_.tm) - (std::sin(4*PI_*t/gait_.tm)/(4*PI_)));
    else if(t>=gait_.tm/2 && t<gait_.tm)
        double y = 2*gait_.height * (1 - (t/gait_.tm) + (std::sin(4*PI_*t/gait_.tm)/(4*PI_)));

    return std::make_pair<x, y>;
}

std::pair<double, double> Trajectory::legSupportTrajectory(double t) {
    double xd = gait_.stride / gait_.tm;
    double x = xd*t - gait_.stride/2;
    double y = 0;

    return std::make_pair<x, y>;
}