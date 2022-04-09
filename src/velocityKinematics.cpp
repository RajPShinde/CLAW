#include <velocityKinematics.hpp>

VelocityKinematics::VelocityKinematics(){
}

VelocityKinematics::~VelocityKinematics(){
}

void VelocityKinematics::jacobian(std::vector<double> JointAngles, int n){
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

    Eigen::Vector3d jLinear1 = z1*(o4-o1);
    Eigen::Vector3d jLinear2 = z2*(o4-o2);
    Eigen::Vector3d jLinear3 = z3*(o4-o3);

    Eigen::Vector3d jAngular1 = z1;
    Eigen::Vector3d jAngular2 = z2;
    Eigen::Vector3d jAngular3 = z3;
}