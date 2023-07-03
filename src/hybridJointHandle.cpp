#include <hybridJointHandle.hpp>

HybridJointHandle::HybridJointHandle(){

}

HybridJointHandle::HybridJointHandle(double* positionDesired, double* velocityDesired, double* kp, double* kd, double* ff)
                                    : positionDesired_(positionDesired), velocityDesired_(velocityDesired), kp_(kp), kd_(kd), ff_(ff) {
}

HybridJointHandle::~HybridJointHandle(){

}

void HybridJointHandle::setPositionDesired(double command) {
   assert(positionDesired_);
   *positionDesired_ = command;
}

void HybridJointHandle::setVelocityDesired(double command) {
   assert(velocityDesired_);
   *velocityDesired_ = command;
}

void HybridJointHandle::setKp(double command) {
   assert(kp_);
   *kp_ = command;
}

void HybridJointHandle::setKd(double command) {
   assert(kd_);
   *kd_ = command;
}

void HybridJointHandle::setFeedforward(double command) {
   assert(ff_);
   *ff_ = command;
}

void HybridJointHandle::setCommand(double positionDesired, double velocityDesired, double kp, double kd, double ff) {
   setPositionDesired(positionDesired);
   setVelocityDesired(velocityDesired);
   setKp(kp);
   setKd(kd);
   setFeedforward(ff);
}

double HybridJointHandle::getPositionDesired() {
   assert(positionDesired_);
   return *positionDesired_;
}

double HybridJointHandle::getVelocityDesired() {
   assert(velocityDesired_);
   return *velocityDesired_;
}

double HybridJointHandle::getKp() {
   assert(kp_);
   return *kp_;
}

double HybridJointHandle::getKd() {
   assert(kd_);
   return *kd_;
}

double HybridJointHandle::getFeedforward() {
   assert(ff_);
   return *ff_;
}