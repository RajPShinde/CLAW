#include <trajectory.hpp>

Trajectory::Trajectory(Gait &obj, std::string s): gait_(obj), legTrajectoryType_(s){
    controlPoints_ = {{-0.06, 0, 0},
                      {-0.08, 0, 0},
                      {-0.11, 0.05, 0},
                      {-0.11, 0.05, 0},
                      {-0.11, 0.05, 0},
                      {0, 0.05, 0},
                      {0, 0.05, 0},
                      {0, 0.08, 0},
                      {0.11, 0.08, 0},
                      {0.11, 0.08, 0},
                      {0.08, 0, 0},
                      {0.06, 0, 0}};
}

Trajectory::~Trajectory(){
}

Point Trajectory::jerkMinimizedTrajectory(double x0, double y0, double z0, double xT, double yT, double zT, double T, double t){
    Polynomial pX(x0, 0, 0, xT, 0, 0, T);
    Polynomial pY(y0, 0, 0, yT, 0, 0, T);
    Polynomial pZ(z0, 0, 0, zT, 0, 0, T);

    return {pX.position(t), pY.position(t), pZ.position(t)};
}

Eigen::Vector3d Trajectory::stancePhaseTrajectory(double t) {
    double x, y;
    if(legTrajectoryType_ == "compoundCycloid"){
        double c2, c3;
        c2 = -gait_.stride/2 - (((-1*gait_.stride*gait_.te*std::sin(M_PI*gait_.te/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*gait_.te)/(2*gait_.tm)));
        c3 = gait_.stride/2;
        if(t >= 0 && t <= gait_.te)
            x = ((-1*gait_.stride*gait_.te*std::sin(M_PI*t/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*t)/(2*gait_.tm)) +c2;
        else if(t >= gait_.te && t <= (gait_.te + gait_.tn))
            x = gait_.stride * (((t-gait_.te)/gait_.tn) - (std::sin(2*M_PI*(t-gait_.te)/gait_.tn)/(2*M_PI))) - gait_.stride/2;
        else if(t >= (gait_.te + gait_.tn) && t <= gait_.tm)
            x = ((gait_.stride*gait_.te*std::sin(M_PI*(t-gait_.te-gait_.tn)/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*(t-gait_.te-gait_.tn))/(2*gait_.tm)) + c3;

        if(t>=0 && t<gait_.tm/2)
            y = 2*gait_.height * ((t/gait_.tm) - (std::sin(4*M_PI*t/gait_.tm)/(4*M_PI)));
        else if(t>=gait_.tm/2 && t<gait_.tm)
            y = 2*gait_.height * (1 - (t/gait_.tm) + (std::sin(4*M_PI*t/gait_.tm)/(4*M_PI)));
    }

    else if(legTrajectoryType_ == "bezier"){
        t = (1/gait_.tm)*t;
        int n = controlPoints_.size() - 1;
        for(int i = 0; i < controlPoints_.size(); i++)
            {
                float coeff = factorial(n) / (factorial(i) * factorial(n - i));
                x += coeff * std::pow(t, i) * std::pow((1 - t), (n - i)) * controlPoints_[i].x;
                y += coeff * std::pow(t, i) * std::pow((1 - t), (n - i)) * controlPoints_[i].y;
            }
    }
    Eigen::Vector3d p;
    p << x, y, 0;
    return p;
}

Eigen::Vector3d Trajectory::supportPhaseTrajectory(double t) {
    double x, y;
    if(legTrajectoryType_ == "compoundCycloid"){
        t = gait_.tm- t;
        double c2 = ((-1*gait_.stride*gait_.te*std::sin(M_PI*gait_.te/gait_.te))/(2*gait_.tm*M_PI)) + ((gait_.stride*gait_.te)/(2*gait_.tm));
        double xd =(gait_.stride -  2*c2) / gait_.tm;
        x = xd*t - (gait_.stride-2*c2)/2;
        y = 0;
    }
    else if(legTrajectoryType_ == "bezier"){
        t = gait_.tm- t;
        double xd =(gait_.stride) / gait_.tm;
        x = xd*t - (gait_.stride)/2;
        y = 0;
    }
    Eigen::Vector3d p;
    p << x, y, 0;
    return p;
}

long int Trajectory::factorial(int n){
    return std::tgamma(n+1);
}

void Trajectory::setBezierControlPoints(std::vector<Point> points){
    
}
