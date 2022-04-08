#include <forwardKinematics.hpp>

ForwardKinematics::ForwardKinematics(){
}

ForwardKinematics::~ForwardKinematics(){
}

Eigen::Matrix4d translateH(double x, double y, double z){
    Eigen::Transform<double, 3, Affine> transH;
    transH = Eigen::Translation<double, 3> (Eigen::Vector3d(x ,y, z));
    return transH;
}

Eigen::Matrix4d rotationH(double roll, double pitch, double yaw){
    Eigen::Transform<double, 3, Affine> rotH = Eigen::Transform<double, 3, Affine>::Identity();
    rotH.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnityX()));
    rotH.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnityY()));
    rotH.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnityZ()));
    return rotH;
}

Eigen::Matrix4d inverseH(Eigen::Matrix4d H){

}

Eigen::Matrix4d ai(double theta, double d, double a, double alpha){
    Eigen::Matrix4d ai << std::cos(theta), -1*std::sin(theta)*std::cos(alpha), std::sin(theta)*std::sin(alpha),    a*std::cos(theta),
                          std::sin(theta), std::cos(theta)*std::cos(alpha),    -1*std::cos(theta)*std::sin(alpha), a*std::sin(theta)),
                          0              , std::sin(alpha)                ,    std::cos(alpha)                  ,  d,
                          0              , 0                              ,    0                                ,  1;
    return ai;
}

Eigen::Matrix4d ForwardKinematics::worldToBaseH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot){
    return translateH(xTrans, yTrans, zTrans) * rotateH(xRot, yRot, zRot);
}

Eigen::Matrix4d ForwardKinematics::baseToLegH(int n){
    return translateH(claw_.bodyTF[n-1][0], claw_.bodyTF[n-1][1], claw_.bodyTF[n-1][2]) * rotationH(claw_.bodyTF[n-1][3], claw_.bodyTF[n-1][4], claw_.bodyTF[n-1][5]);
}

Eigen::Matrix4d ForwardKinematics::worldToLegH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, int n){
    return worldToBaseH(xTrans, yTrans, zTrans, xRot, yRot, zRot) * baseToLegH(int n);
}

Eigen::Matrix4d ForwardKinematics::legToFootH(std::vector<double> JointAngles, int n){
    double dir = 1;
    if(n == (2 || 3))
        dir = -1;
    return ai(jointAngles[0], DH[0][1], DH[0][2], DH[0][3]) * ai(jointAngles[1], dir*DH[1][1], DH[1][2], DH[1][3]) * ai(jointAngles[2], DH[2][1], DH[2][2], DH[2][3]);
}

Eigen::Vector3d footInLegFrame(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, Eigen::Vector3d p, int n){
    return inverseH(worldToLegH(xTrans, yTrans, zTrans, xRot, yRot, zRot, n))*p;
}
