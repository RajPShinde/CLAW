#ifndef INCLUDE_DATA_HPP_
#define INCLUDE_DATA_HPP_

struct Point{
    double x;
    double y;
    double z;
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

#endif  //  INCLUDE_DATA_HPP_