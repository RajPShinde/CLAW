#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

#include <vector>
#include <map>
#include <math.h>

class Claw {
    public:
        // Mechanical Parameters
        static constexpr double femur = 0.2245;
        static constexpr double tibia = 0.238;
        static constexpr double link1 = 0.2245; // femur
        static constexpr double link2 = 0.238; // tibia
        static constexpr double a = 0.03617;
        static constexpr double d = 0.00;
        static constexpr double idleLegHeight = 0.35;
        static constexpr double legMaxHeight = 0.45;
        static constexpr double legMinHeight = 0.20;
        static constexpr int noOfActuators = 12;
        static constexpr double reductionHA = 3;
        static constexpr double reductionHF = 10.8;
        static constexpr double reductionKF = 10.8;
        static constexpr double bodyTF[4][6] = {{ 0.066 + 0.173,  0.141, -0.031 - 0.036, 0, M_PI/2, 0},   // To Front Left Shoulder
                                                { 0.066 + 0.173, -0.141, -0.031 - 0.036, 0, M_PI/2, 0},   // To Front Right Shoulder
                                                {-0.066 - 0.173, -0.141, -0.031 - 0.036, 0, M_PI/2, 0},   // To Rear Left Shoulder
                                                {-0.066 - 0.173,  0.141, -0.031 - 0.036, 0, M_PI/2, 0}};  // To Rear Right Shoulder

                                         //  d  a  alpha
        static constexpr double DH[3][3] = {{0, a,     -90},  // HA->HF
                                            {d, link1,   0},  // HF->KF
                                            {0, link2,   0}}; // KF->Foot

        // Electrical Parameters
        static constexpr double kv = 90;
        static constexpr double kt = kv*2*3.142/60;
        static constexpr double maxCurrent = 30;
        static constexpr double minBatteryVoltage = 22.2;
        static constexpr double countsPerRevolution = 2000;
        static constexpr double abductionCPRAngleRelation = countsPerRevolution * reductionHA / 360;
        static constexpr double flexionCPRAngleRelation = countsPerRevolution * reductionHF / 360;
        static constexpr int encoderOffset[4][3] = {{0, 0, 0},
                                                    {0, 0, 0},
                                                    {0, 0, 0},
                                                    {0, 0, 0}};


        // Network
        static constexpr char canDevice[] = "can0";

        // General Parameters
        std::string legConfiguration = ">>";
        std::vector<std::string> legConfigurations = {">>", "<<", "><"};
};

#endif  //  INCLUDE_CLAW_HPP_