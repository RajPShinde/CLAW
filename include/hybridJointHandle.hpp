#ifndef INCLUDE_HYBRIDJOINTHANDLE_HPP_
#define INCLUDE_HYBRIDJOINTHANDLE_HPP_

#include <hardware_interface/joint_state_interface.h>

class HybridJointHandle : public hardware_interface::JointStateHandle {
    public:
        HybridJointHandle();

        HybridJointHandle(const JointStateHandle& js, double* positionDesired, double* velocityDesired, double* kp, double* kd, double* ff);

        ~HybridJointHandle();

        void setPositionDesired(double command);

        void setVelocityDesired(double command);

        void setKp(double command);

        void setKd(double command);

        void setFeedforward(double command);

        void setCommand(double positionDesired, double velocityDesired, double kp, double kd, double ff);

        double getPositionDesired();

        double getVelocityDesired();

        double getKp();

        double getKd();

        double getFeedforward();

    private:
        double* positionDesired_ = nullptr;
        double* velocityDesired_ = nullptr;
        double* kp_ = nullptr;
        double* kd_ = nullptr;
        double* ff_ = nullptr;
};

#endif  //  INCLUDE_HYBRIDJOINTHANDLE_HPP_