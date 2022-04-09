#include <velocityKinematics.hpp>

VelocityKinematics::VelocityKinematics(){
}

VelocityKinematics::~VelocityKinematics(){
}

Eigen::MatrixXd VelocityKinematics::jacobian(std::vector<double> JointAngles, int n){
    double dir = 1;
    if(n == (2 || 3))
        dir = -1;

    Eigen::Matrix4d h12 = ai(jointAngles[0], DH[0][1], DH[0][2], DH[0][3]);
    Eigen::Matrix4d h23 = h12*ai(jointAngles[1], dir*DH[1][1], DH[1][2], DH[1][3]);
    Eigen::Matrix4d h34 = h23*ai(jointAngles[2], DH[2][1], DH[2][2], DH[2][3]);

    Eigen::Vector3d o2 = h12.block<3,1>(0,3);
    Eigen::Vector3d o3 = h23.block<3,1>(0,3);
    Eigen::Vector3d o4 = h34.block<3,1>(0,3);

    Eigen::Vector3d z1 = h12.block<3,1>(0,2);
    Eigen::Vector3d z2 = h23.block<3,1>(0,2);
    Eigen::Vector3d z3 = h34.block<3,1>(0,2);

    Eigen::MatrixXd jacobian(6,3);

    // Linear
    jacobian.block<3, 1>(0,0) = z1.cross(o4-o1);
    jacobian.block<3, 1>(0,1) = z2.cross(o4-o2);
    jacobian.block<3, 1>(0,2) = z3.cross(o4-o3);
    // Angular
    jacobian.block<3, 1>(3,0) = z1;
    jacobian.block<3, 1>(3,1) = z2;
    jacobian.block<3, 1>(3,2) = z3;

    return jacobian;
}

Eigen::Vector3d footVelocities(Eigen::Vector3d jointVelocities, std::vector<double> JointAngles, int n){
    return Jacobian(jointAngles, n) * jointVelocities;
}