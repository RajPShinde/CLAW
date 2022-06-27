#include <forwardKinematics.hpp>

ForwardKinematics::ForwardKinematics(){
}

ForwardKinematics::~ForwardKinematics(){
}

Eigen::Matrix4d ForwardKinematics::translationH(double x, double y, double z){
    Eigen::Transform<double, 3, Eigen::Affine> transH;
    transH = Eigen::Translation<double, 3> (Eigen::Vector3d(x ,y, z));
    return transH.matrix();
}

Eigen::Matrix4d ForwardKinematics::rotationH(double roll, double pitch, double yaw){
    Eigen::Transform<double, 3, Eigen::Affine> rotH = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    rotH.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    rotH.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    rotH.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return rotH.matrix();
}

Eigen::Matrix4d ForwardKinematics::inverseH(Eigen::Matrix4d H){
    // invH = |invR -d|
    //        |  0   1|
    Eigen::Matrix3d inverseR = H.block<3,3>(0,0);
    inverseR.transposeInPlace();

    Eigen::Vector3d negatived = H.block<3,1>(0,3);
    negatived = negatived * -1;

    Eigen::Matrix4d a = Eigen::Matrix4d::Identity();
    a.block<3,3>(0,0) = inverseR;
    Eigen::Matrix4d b = Eigen::Matrix4d::Identity();
    b.block<3,1>(0,3) = negatived;
    return a * b;
}

Eigen::Matrix4d ForwardKinematics::ai(double theta, double d, double a, double alpha){
    Eigen::Matrix4d ai;
    ai << std::cos(theta), -1*std::sin(theta)*std::cos(alpha), std::sin(theta)*std::sin(alpha),    a*std::cos(theta),
          std::sin(theta), std::cos(theta)*std::cos(alpha),    -1*std::cos(theta)*std::sin(alpha), a*std::sin(theta),
          0              , std::sin(alpha)                ,    std::cos(alpha)                  ,  d,
          0              , 0                              ,    0                                ,  1;
    return ai;
}

Eigen::Matrix4d ForwardKinematics::worldToBaseH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot){
    return translationH(xTrans, yTrans, zTrans) * rotationH(xRot, yRot, zRot);
}

Eigen::Matrix4d ForwardKinematics::baseToLegH(int n){
    return translationH(claw_.bodyTF[n-1][0], claw_.bodyTF[n-1][1], claw_.bodyTF[n-1][2]) * rotationH(claw_.bodyTF[n-1][3], claw_.bodyTF[n-1][4], claw_.bodyTF[n-1][5]);
}

Eigen::Matrix4d ForwardKinematics::worldToLegH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, int n){
    return worldToBaseH(xTrans, yTrans, zTrans, xRot, yRot, zRot) * baseToLegH(n);
}

Eigen::Matrix4d ForwardKinematics::legToFootH(std::vector<double> jointAngles, int n){
    double dir = 1;
    if(n == (2 || 3))
        dir = -1;
    return ai(jointAngles[0], claw_.DH[0][1], claw_.DH[0][2], claw_.DH[0][3]) * ai(jointAngles[1], dir*claw_.DH[1][1], claw_.DH[1][2], claw_.DH[1][3]) * ai(jointAngles[2], claw_.DH[2][1], claw_.DH[2][2], claw_.DH[2][3]);
}

Eigen::Vector3d ForwardKinematics::footInLegFrame(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, Eigen::Vector3d p, int n){
    Eigen::Vector4d point(p(0), p(1), p(2), 1);
    return (inverseH(worldToLegH(xTrans, yTrans, zTrans, xRot, yRot, zRot, n)) * point).block<3,1>(0,0);
}