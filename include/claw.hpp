#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

class Claw {
    public:
        Claw();

        ~Claw();

        const double femur = 0.20;
        const double tibia = 0.20;
        const double d = 0.01;
        const double idleBaseHeight = 0.40;
        const double baseMaxHeight = 0.45;
        const double baseMinHeight = 0.20;
        const int noOfActuators = 12;
        const double reductionHA = 3;
        const double reductionHF = 10.8;
        const double reductionKF = 10.8;
        const std::string legConfiguration = ">>";
        std::vector<double> bodyTF = {{ x,  y, 0, 0, PI/2, 0},
                                      { x, -y, 0, 0, PI/2, 0},
                                      {-x, -y, 0, 0, PI/2, 0},
                                      {-x,  y, 0, 0, PI/2, 0}};
        
        const double abductionKv = 90;
        const double abductionKt = 90*2*3.142/60;
        const double abductionMaxCurrent = 80;

        const double flexionKv = 270;
        const double flexionKt = 270*2*3.142/60;
        const double flexionMaxCurrent = 80;

        const double minBatteryVoltage = 22.2;
};

#endif  //  INCLUDE_CLAW_HPP_