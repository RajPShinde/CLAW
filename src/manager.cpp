/*
BSD 3-Clause License

Copyright (c) 2022, Raj Shinde
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file manager.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#include <manager.hpp>

Manager::Manager(ros::NodeHandle &nh) {
    ROS_INFO_STREAM("Initializing Robot");
    jointStatePublisher_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 50, true);
    begin();
}

Manager::~Manager(){
}

int Manager::begin(){

    // Add all Odrive Axis's
    odrive_can_ros::CANSimple master;
    if ( !( master.add_axis(0, "KF1") && master.add_axis(1, "HF1") &&
            master.add_axis(2, "HA2") && master.add_axis(3, "HA1") &&
            master.add_axis(4, "KF2") && master.add_axis(5, "HF2") &&
            master.add_axis(6, "KF3") && master.add_axis(7, "HF3") &&
            master.add_axis(8, "HA4") && master.add_axis(9, "HA3") &&
            master.add_axis(10, "KF4") && master.add_axis(11, "HF4")))
    {
        ROS_ERROR_STREAM("Failed to create one or more axis. Aborting");
        return -1;
    }

    // // Create Interface to SocketCAN 
    // can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();
    // if (!driver->init(Claw::canDevice, 0, can::NoSettings::create()))
    // {
    //     ROS_ERROR_STREAM("Failed to initialize can device");
    //     return -1;
    // }
    // can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
    //     [&driver](const can::State& s) {
    //         std::string err;
    //         driver->translateError(s.internal_error, err);
    //         ROS_ERROR_STREAM("CAN Device error");
    //                     fprintf(stderr, "CAN Device error: %s, asio: %s.\n", 
    //             err.c_str(), s.error_code.message().c_str());
    //     }
    // );

    // // Pass the SocketCAN handle to master
    // master.init(driver);
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // // Background Thread
    // std::thread sensorData( [&master, this]() {
    //     while(master.is_ready() && managerStatus) {
    //         // Update Joint Encoder Data
    //         // for(int name = 0; name < axisNames.size(); name++){
    //         //     master.get_encoder_count(master.axis((axisNames[name]).c_str()));
    //         // }

    //         encoderShadowCount_ = {{master.axis("HA1").encoder_shadow_count, master.axis("HF1").encoder_shadow_count, master.axis("KF1").encoder_shadow_count},
    //                                {master.axis("HA2").encoder_shadow_count, master.axis("HF2").encoder_shadow_count, master.axis("KF2").encoder_shadow_count},
    //                                {master.axis("HA3").encoder_shadow_count, master.axis("HF3").encoder_shadow_count, master.axis("KF3").encoder_shadow_count},
    //                                {master.axis("HA4").encoder_shadow_count, master.axis("HF4").encoder_shadow_count, master.axis("KF4").encoder_shadow_count}};
    
    //         master.get_encoder_count(master.axis("HA3"));
    //         master.get_encoder_count(master.axis("HF3"));
    //         master.get_encoder_count(master.axis("KF3"));

    //         // ROS_ERROR_STREAM(encoderShadowCount_[2][1]);
            
    //         // Check and Update Battery Voltage
    //         master.get_vbus_voltage(master.axis("KF1"));
    //         batteryVoltage_ = master.axis("KF1").vbus_voltage;
    //         if(batteryVoltage_ < Claw::minBatteryVoltage){
    //             ROS_ERROR_STREAM("Battery Voltage Low: "<<batteryVoltage_);
    //             return -1;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     } 
    // } );

    // Initialize Legs

    // Gait gait_(0.15, 0.07, 0.2, 0.7, "trot");
    // Trajectory traj(gait_, "bezier");

    // Start Time
    auto start = std::chrono::high_resolution_clock::now();
    // Loop
    while(ros::ok()){

        move();
        poseManipulation();
    
        // Perform Inverse Kinematics & Obtain Joint Angles
        // auto leg1 = anglesToPosition(InverseKinematics::computeJointAngles(x1, y1, z1, 1) ,1);
        // auto leg2 = anglesToPosition(InverseKinematics::computeJointAngles(x2, y2, z2, 2), 2);
        // auto leg3 = anglesToPosition(InverseKinematics::computeJointAngles(x1, y1, z1, 3), 3);
        // auto leg4 = anglesToPosition(InverseKinematics::computeJointAngles(x2, y2, z2, 4), 4);

        // Send Joint Positions to respective odrive axis
        // for(int name = 0; name<name.size(); name++){
        //     for(int legNo = 1; legNo<=4; legNo++){
        //         master.set_input_pos(master.axis(name+std::to_string(legNo)), leg1[0]);
        //     }
        // }
        // Hip Abduction
        // master.set_input_pos(master.axis("HA1"), leg1[0]);
        // master.set_input_pos(master.axis("HA2"), leg2[0]);
        // master.set_input_pos(master.axis("HA3"), leg3[0]);
        // master.set_input_pos(master.axis("HA4"), leg4[0]);

        // // Hip Flexion
        // master.set_input_pos(master.axis("HF1"), leg1[1]);
        // master.set_input_pos(master.axis("HF2"), leg2[1]);
        // master.set_input_pos(master.axis("HF3"), leg3[1]);
        // master.set_input_pos(master.axis("HF4"), leg4[1]);

        // // Knee Flexion
        // master.set_input_pos(master.axis("KF1"), leg1[2]);
        // master.set_input_pos(master.axis("KF2"), leg2[2]);
        // master.set_input_pos(master.axis("KF3"), leg3[2]);
        // master.set_input_pos(master.axis("KF4"), leg4[2]);

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        ros::spinOnce();

    }
    managerStatus = false;
    // sensorData.join();
    // driver.reset();
}

std::vector<int> Manager::anglesToPosition(std::vector<double> angle, int n){
    return {Claw::encoderOffset[n-1][0] + angle[0] * Claw::abductionCPRAngleRelation, 
            Claw::encoderOffset[n-1][1] + angle[1] * Claw::flexionCPRAngleRelation, 
            Claw::encoderOffset[n-1][2] + angle[2] * Claw::flexionCPRAngleRelation};
}

std::vector<double> Manager::positionToAngle(std::vector<int> position, int n){
    return {(position[0] - Claw::encoderOffset[n-1][0]) / Claw::abductionCPRAngleRelation, 
            (position[1] - Claw::encoderOffset[n-1][1]) / Claw::flexionCPRAngleRelation, 
            (position[2] - Claw::encoderOffset[n-1][2]) / Claw::flexionCPRAngleRelation};
}

void Manager::move(){

    statePublisher(InverseKinematics::computeJointAngles(0.35, 0, 0, 1),
                InverseKinematics::computeJointAngles(0.35, 0, 0, 2),
                InverseKinematics::computeJointAngles(0.35, 0, 0, 3),
                InverseKinematics::computeJointAngles(0.35, 0, 0, 4));
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    int phaseSwitch = 1; 
    int prev = 1;
    int cycle = 1;
    bool inMotion = false;
    auto startWalk = std::chrono::high_resolution_clock::now();
    int f  =0;
    
    while(state_ == "WALK" || inMotion){ 
        inMotion = true;       

        Gait gait(0.12, 0.07, 1, 1, 0, "trot");
        Trajectory traj(gait, "bezier");

        auto elapsed = std::chrono::high_resolution_clock::now() - startWalk;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        double seconds = microseconds * 1e-6;

        // Reset time after half gait cycle is complete
        if(microseconds  * 1e-6 >= gait.tSwing){
            startWalk = std::chrono::high_resolution_clock::now();
            phaseSwitch = !phaseSwitch;
        }

        // Get Swing & Stance Trajectory Coordinates for time = seconds
        Eigen::Vector3d pSwing, pStance;
        pSwing = traj.swingPhaseTrajectory(seconds, cycle);
        pStance = traj.stancePhaseTrajectory(seconds, cycle);

        // Transform Trajectory Coordinates to Hip Frame and Rotate based on commandDirection
        pSwing = fk_.trajectoryToLegH(pSwing);
        pStance = fk_.trajectoryToLegH(pStance);

        Point p1, p2;
        if(phaseSwitch){
            p1 = {pSwing(0), pSwing(1), pSwing(2)};
            p2 = {pStance(0), pStance(1), pStance(2)};

        }
        else{
            p1 = {pStance(0), pStance(1), pStance(2)};
            p2 = {pSwing(0), pSwing(1), pSwing(2)};

        }

        statePublisher(InverseKinematics::computeJointAngles(p1.x, p2.y, p1.z, 1),
                       InverseKinematics::computeJointAngles(p2.x, p2.y, p2.z, 2),
                       InverseKinematics::computeJointAngles(p1.x, p1.y, p1.z, 3),
                       InverseKinematics::computeJointAngles(p2.x, p2.y, p2.z, 4));


        ros::spinOnce();

        if(phaseSwitch!=prev){
            f++;
            prev = phaseSwitch;
            if(cycle == -1)
                break;
            if(cycle == 1)
                cycle = 0;
            if(state_ != "WALK")
                cycle = -1;
        }
        if(f==1)
            state_ = "NONE";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    state_ = "WALK";
}

void Manager::poseManipulation(){
    while(state_ == "MOVE_BASE"){
        Pose worldPose;
        double reduceLegHeightBy = 0;
        Eigen::Vector3d foot1World = {Claw::bodyTF[0][0], Claw::bodyTF[0][1], Claw::bodyTF[0][2] - Claw::idleLegHeight - reduceLegHeightBy};
        Eigen::Vector3d foot2World = {Claw::bodyTF[1][0], Claw::bodyTF[1][1], Claw::bodyTF[1][2] - Claw::idleLegHeight - reduceLegHeightBy};
        Eigen::Vector3d foot3World = {Claw::bodyTF[2][0], Claw::bodyTF[2][1], Claw::bodyTF[2][2] - Claw::idleLegHeight - reduceLegHeightBy};
        Eigen::Vector3d foot4World = {Claw::bodyTF[3][0], Claw::bodyTF[3][1], Claw::bodyTF[3][2] - Claw::idleLegHeight - reduceLegHeightBy};

        Eigen::Vector3d foot1Leg = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot1World, 1);
        Eigen::Vector3d foot2Leg = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot2World, 2);
        Eigen::Vector3d foot3Leg = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot3World, 3);
        Eigen::Vector3d foot4Leg = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot4World, 4);

        statePublisher(InverseKinematics::computeJointAngles(foot1Leg(0), foot1Leg(1), foot1Leg(2), 1),
                       InverseKinematics::computeJointAngles(foot2Leg(0), foot2Leg(1), foot2Leg(2), 2),
                       InverseKinematics::computeJointAngles(foot3Leg(0), foot3Leg(1), foot3Leg(2), 3),
                       InverseKinematics::computeJointAngles(foot4Leg(0), foot4Leg(1), foot4Leg(2), 4));

        ros::spinOnce();
    }
}

void Manager::commandOdrives(){

}


void Manager::statePublisher(std::vector<double> l1, std::vector<double> l2, std::vector<double> l3, std::vector<double> l4){
    sensor_msgs::JointState state;
    state.header.frame_id = "";
    state.header.stamp = ros::Time::now();
    state.name  = {"HA1", "HF1", "KF1", "HA2", "HF2", "KF2", "HA3", "HF3", "KF3", "HA4", "HF4", "KF4"};
    state.position.resize(state.name.size());
    state.position[0] = l1[0];
    state.position[1] = l1[1];
    state.position[2] = l1[2];
    state.position[3] = l2[0];
    state.position[4] = l2[1];
    state.position[5] = l2[2];
    state.position[6] = l3[0];
    state.position[7] = l3[1];
    state.position[8] = l3[2];
    state.position[9] = l4[0];
    state.position[10] = l4[1];
    state.position[11] = l4[2];
    jointStatePublisher_.publish(state);
}