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

    // Create Interface to SocketCAN 
    can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();
    if (!driver->init(Claw::canDevice, 0, can::NoSettings::create()))
    {
        ROS_ERROR_STREAM("Failed to initialize can device");
        return -1;
    }
    can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
        [&driver](const can::State& s) {
            std::string err;
            driver->translateError(s.internal_error, err);
            ROS_ERROR_STREAM("CAN Device error");
                        fprintf(stderr, "CAN Device error: %s, asio: %s.\n", 
                err.c_str(), s.error_code.message().c_str());
        }
    );

    // Pass the SocketCAN handle to master
    master.init(driver);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Background Thread
    std::thread sensorData( [&master, this]() {
        while(master.is_ready() && managerStatus) {
            // Update Joint Encoder Data
            // for(int name = 0; name < axisNames.size(); name++){
            //     master.get_encoder_count(master.axis((axisNames[name]).c_str()));
            // }

            encoderShadowCount_ = {{master.axis("HA1").encoder_shadow_count, master.axis("HF1").encoder_shadow_count, master.axis("KF1").encoder_shadow_count},
                                   {master.axis("HA2").encoder_shadow_count, master.axis("HF2").encoder_shadow_count, master.axis("KF2").encoder_shadow_count},
                                   {master.axis("HA3").encoder_shadow_count, master.axis("HF3").encoder_shadow_count, master.axis("KF3").encoder_shadow_count},
                                   {master.axis("HA4").encoder_shadow_count, master.axis("HF4").encoder_shadow_count, master.axis("KF4").encoder_shadow_count}};
    
            master.get_encoder_count(master.axis("HA3"));
            master.get_encoder_count(master.axis("HF3"));
            master.get_encoder_count(master.axis("KF3"));

            // ROS_ERROR_STREAM(encoderShadowCount_[2][1]);
            
            // Check and Update Battery Voltage
            master.get_vbus_voltage(master.axis("KF1"));
            batteryVoltage_ = master.axis("KF1").vbus_voltage;
            if(batteryVoltage_ < Claw::minBatteryVoltage){
                ROS_ERROR_STREAM("Battery Voltage Low: "<<batteryVoltage_);
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } 
    } );

    // Initialize Legs

    Gait gait_(0.15, 0.07, 0.2, 0.7, "trot");
    Trajectory traj(gait_, "bezier");

    int phaseSwitch = 0; 

    // Start Time
    auto start = std::chrono::high_resolution_clock::now();
    // Loop
    while(ros::ok()){
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        double seconds = microseconds * 1e-6;


        // Reset time after half gait cycle is complete
        if(microseconds  * 1e-6 >= gait_.tm){
            start = std::chrono::high_resolution_clock::now();
            phaseSwitch = !phaseSwitch;
        }
        else{
        
        // Get Swing & Stance Trajectory Coordinates for time = seconds
        Eigen::Vector3d pStance, pSupport;
        pStance = traj.stancePhaseTrajectory(seconds);
        pSupport = traj.supportPhaseTrajectory(seconds);

        // Transform Trajectory Coordinates to Hip Frame and Rotate based on commandDirection
        pStance = fk_.trajectoryToLegH(pStance);
        pSupport = fk_.trajectoryToLegH(pSupport);

        Point p1, p2;
        if(phaseSwitch){
            p1 = {pStance(0), pStance(1), pStance(2)};
            p2 = {pSupport(0), pSupport(1), pSupport(2)};
        }
        else{
            p1 = {pSupport(0), pSupport(1), pSupport(2)};
            p2 = {pStance(0), pStance(1), pStance(2)};
        }

        // Perform Inverse Kinematics & Obtain Joint Angles
        // auto leg1 = anglesToPosition(InverseKinematics::computeJointAngles(x1, y1, z1, 1) ,1);
        // auto leg2 = anglesToPosition(InverseKinematics::computeJointAngles(x2, y2, z2, 2), 2);
        // auto leg3 = anglesToPosition(InverseKinematics::computeJointAngles(x1, y1, z1, 3), 3);
        // auto leg4 = anglesToPosition(InverseKinematics::computeJointAngles(x2, y2, z2, 4), 4);

        // statePublisher(InverseKinematics::computeJointAngles(j1(0), j1(1), j1(2), 1),
        //                InverseKinematics::computeJointAngles(j2(0), j2(1), j2(2), 2),
        //                InverseKinematics::computeJointAngles(j3(0), j3(1), j3(2), 3),
        //                InverseKinematics::computeJointAngles(j4(0), j4(1), j4(2), 4));

        statePublisher(InverseKinematics::computeJointAngles(p1.x, p2.y, p1.z, 1),
                       InverseKinematics::computeJointAngles(p2.x, p2.y, p2.z, 2),
                       InverseKinematics::computeJointAngles(p1.x, p1.y, p1.z, 3),
                       InverseKinematics::computeJointAngles(p2.x, p2.y, p2.z, 4));

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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        ros::spinOnce();
    }

        
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

}

void Manager::poseManipulation(Pose worldPose, double reduceLegHeightBy){
    Eigen::Vector3d foot1WorldPose = {Claw::bodyTF[0][0], Claw::bodyTF[0][1], Claw::bodyTF[0][2] - Claw::idleLegHeight - reduceLegHeightBy};
    Eigen::Vector3d foot2WorldPose = {Claw::bodyTF[1][0], Claw::bodyTF[1][1], Claw::bodyTF[1][2] - Claw::idleLegHeight - reduceLegHeightBy};
    Eigen::Vector3d foot3WorldPose = {Claw::bodyTF[2][0], Claw::bodyTF[2][1], Claw::bodyTF[2][2] - Claw::idleLegHeight - reduceLegHeightBy};
    Eigen::Vector3d foot4WorldPose = {Claw::bodyTF[3][0], Claw::bodyTF[3][1], Claw::bodyTF[3][2] - Claw::idleLegHeight - reduceLegHeightBy};
    Eigen::Vector3d j1 = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot1WorldPose, 1);
    Eigen::Vector3d j2 = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot2WorldPose, 2);
    Eigen::Vector3d j3 = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot3WorldPose, 3);
    Eigen::Vector3d j4 = fk_.footInLegFrame(worldPose.x, worldPose.y, worldPose.z, worldPose.roll, worldPose.pitch, worldPose.yaw, foot4WorldPose, 4);
}

// void Manager::commandOdrives(){

// }


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