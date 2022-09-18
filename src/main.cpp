#include <manager.hpp>
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "clawmanager");
    ros::NodeHandle nh;
    Manager claw(nh);
    return 0;
}