#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

#include <vector>
#include <map>
#include <math.h>

class Claw {
    public:
        Claw();

        ~Claw();

        const double femur = 0.20;
        const double tibia = 0.20;
        const double d = 0.01;
        const double a = 0.01;
        const double idleBaseHeight = 0.40;
        const double baseMaxHeight = 0.45;
        const double baseMinHeight = 0.20;
        const int noOfActuators = 12;
        const double reductionHA = 3;
        const double reductionHF = 10.8;
        const double reductionKF = 10.8;
        std::string currentLegConfiguration = ">>";
        std::vector<std::string> legConfigurations = {">>", "<<", "><"};
        std::vector<double> bodyTF = {{ x,  y, 0, 0, M_PI/2, 0},
                                      { x, -y, 0, 0, M_PI/2, 0},
                                      {-x, -y, 0, 0, M_PI/2, 0},
                                      {-x,  y, 0, 0, M_PI/2, 0}};
        
        const double kv = 90;
        const double kt = kv*2*3.142/60;
        const double maxCurrent = 30;
        const double minBatteryVoltage = 22.2;

        std::vector<int> encoderOffset = {{0, 0, 0},
                                          {0, 0, 0},
                                          {0, 0, 0},
                                          {0, 0, 0}};
};

#endif  //  INCLUDE_CLAW_HPP_