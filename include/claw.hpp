#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

class Claw {
    public:
        Claw();

        ~Claw();

        const double femur = 20;
        const double tibia = 20;
        const double idleBaseHeight = 40;
        const double baseMaxHeight = 45;
        const double baseMinHeight = 20;
        const int noOfActuators = 12;
        const double reductionHA = 3;
        const double reductionHF = 10;
        const double reductionKF = 10;
        const std::string legConfiguration = ">>";

};

#endif  //  INCLUDE_CLAW_HPP_