#ifndef INCLUDE_GAIT_HPP_
#define INCLUDE_GAIT_HPP_

class Gait {
    public:
        Gait(double stride, double height, double te, double tm, std::string type);

        ~Gait();

        const double stride;
        const double height;
        const double te;
        const double tm;
        const double tn;
        
        const std::string type;
};

#endif  //  INCLUDE_GAIT_HPP_