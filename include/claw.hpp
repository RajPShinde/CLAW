#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

class Claw {
    public:
        Claw();

        ~Claw();

    private:
        const double femur_ = 20;
        const double tibia_ = 20;
        const double idleBaseHeight_ = 40;
        const double baseMaxHeight_ = 45;
        const double baseMinHeight_ = 20;
        const std::string legConfiguration_ = ">>";
        const int noOfActuators_ = 12;
};

#endif  //  INCLUDE_CLAW_HPP_