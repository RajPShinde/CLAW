#ifndef INCLUDE_GAIT_HPP_
#define INCLUDE_GAIT_HPP_

#include <string>

class Gait {
    public:
        Gait(double s, double h, double e, double m, std::string type);

        ~Gait();

        const double stride;
        const double height;
        const double te;
        const double tm;
        const double tn;
        
        const std::string name;
};

#endif  //  INCLUDE_GAIT_HPP_