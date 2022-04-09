#ifndef INCLUDE_FORWARDKINEMATICS_HPP_
#define INCLUDE_FORWARDKINEMATICS_HPP_

/**
 * @file forwardKinematics.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <math>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <claw.hpp>

class ForwardKinematics {
    public:
        ForwardKinematics();
        
        ~ForwardKinematics();

        Eigen::Matrix4d translateH(double xTrans, double yTrans, double zTrans);

        Eigen::Matrix4d rotationH(double xRot, double yRot, double zRot);

        Eigen::Matrix4d inverseH(Eigen::Matrix4d H);

        Eigen::Matrix4d ai(double zRot, double d, double a, double xRot);

        Eigen::Matrix4d worldToBaseH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot);

        Eigen::Matrix4d baseToLegH(int n);

        Eigen::Matrix4d worldToLegH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, int n);

        Eigen::Matrix4d legToFootH(std::vector<double> JointAngles, int n);

        Eigen::Vector3d footInLegFrame(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, Eigen::Vector3d p, int n);

    private:
        Claw claw_;
};

#endif  //  INCLUDE_FORWARDKINEMATICS_HPP_