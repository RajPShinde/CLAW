#ifndef INCLUDE_DATA_HPP_
#define INCLUDE_DATA_HPP_

struct Point{
    double x;
    double y;
    double z;

    Point(){};
    Point(double a, double b, double c): x(a), y(b), z(c) {};

    bool operator == (const Point &p) const {
        return (x == p.x && y == p.y && z == p.z);
    }
};

struct Pose{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct LegData{
    double n;
    double alpha;
    double beta;
    double gamma;
    Point p;
};

struct OdrivePosition{
    double alphaPosition;
    double betaPosition;
    double gammaPosition;
};

struct Gait{
    double stride;
    double height;

    double tSwing;
    double tStance;
    double tDelay;
    
    double te = 0.1;
    double tm = 0.15;
    double tn = 0.13;
    
    std::string type;

    Gait(){};
    Gait(double a, double b, double c, double d, double e, std::string f): stride(a), height(b), tSwing(c), tStance(d), tDelay(e), type(f) {};
};

#endif  //  INCLUDE_DATA_HPP_