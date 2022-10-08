#ifndef INCLUDE_DATA_HPP_
#define INCLUDE_DATA_HPP_

struct Point{
    double x;
    double y;
    double z;

    Point(){};
    Point(double a, double b, double c): x(a), y(b), z(c) {};

    bool operator == (const Point &p) const {
        return (x == p.x && y == p.y);
    }

    std::vector<double> operator = (const Point &p) const {
        return {p.x, p.y, p.z};
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

#endif  //  INCLUDE_DATA_HPP_