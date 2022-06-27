#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

#include <vector>
#include <map>
#include <math.h>

class Claw {
    public:
        Claw();

        ~Claw();

        // Mechanical Parameters
        const double femur = 0.20;
        const double tibia = 0.20;
        const double link1 = 0.20; // femur
        const double link2 = 0.20; // tibia
        const double d = 0.01;
        const double a = 0.01;
        const double idleBaseHeight = 0.40;
        const double baseMaxHeight = 0.45;
        const double baseMinHeight = 0.20;
        const int noOfActuators = 12;
        const double reductionHA = 3;
        const double reductionHF = 10.8;
        const double reductionKF = 10.8;
        std::vector<std::vector<double>> bodyTF = {{ 1,  1, 0, 0, M_PI/2, 0},   // To Front Left Shoulder
                                                   { 1, -1, 0, 0, M_PI/2, 0},   // To Front Right Shoulder
                                                   {-1, -1, 0, 0, M_PI/2, 0},   // To Rear Left Shoulder
                                                   {-1,  1, 0, 0, M_PI/2, 0}};  // To Rear Right Shoulder
                                            //  d  a  alpha
        std::vector<std::vector<double>> DH = {{0, 0, 0},  // HA->HF
                                               {0, 0, 0},  // HF->KF
                                               {0, 0, 0}}; // KF->Foot

        // Electrical Parameters
        const double kv = 90;
        const double kt = kv*2*3.142/60;
        const double maxCurrent = 30;
        const double minBatteryVoltage = 22.2;
        const double countsPerRevolution = 2048;
        const double abductionCPRAngleRelation = countsPerRevolution * reductionHA / 360;
        const double flexionCPRAngleRelation = countsPerRevolution * reductionHF / 360;
        std::vector<std::vector<int>> encoderOffset = {{0, 0, 0},
                                                       {0, 0, 0},
                                                       {0, 0, 0},
                                                       {0, 0, 0}};

        // General Parameters
        std::string currentLegConfiguration = ">>";
        std::vector<std::string> legConfigurations = {">>", "<<", "><"};
};

#endif  //  INCLUDE_CLAW_HPP_