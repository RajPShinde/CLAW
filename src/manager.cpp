#include <manager.hpp>

Manager::Manager(ros::NodeHandle &nh) {
    ROS_INFO_STREAM("Initializing Robot");
    begin();
}

Manager::~Manager(){
}

int Manager::begin(){
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
    const std::string canDevice_ = "can0";
    can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();
    if (!driver->init(canDevice_, 0, can::NoSettings::create()))
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
            ROS_ERROR_STREAM(master.axis("HA3").encoder_shadow_count);
            ROS_ERROR_STREAM(master.axis("HF3").encoder_shadow_count);
            ROS_ERROR_STREAM(master.axis("KF3").encoder_shadow_count);

            // ROS_ERROR_STREAM(encoderShadowCount_[2][1]);
            
            // Check and Update Battery Voltage
            master.get_vbus_voltage(master.axis("KF1"));
            batteryVoltage_ = master.axis("KF1").vbus_voltage;
            ROS_ERROR_STREAM("Battery Voltage Low: "<<batteryVoltage_);
            if(batteryVoltage_ < Claw::minBatteryVoltage){
                ROS_ERROR_STREAM("Battery Voltage Low: "<<batteryVoltage_);
                // return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } 
    } );

    // Initialize Legs
    Gait gait_(2, 2, 2, 2, "test");
    Trajectory traj(gait_);

    // commands
    commandValue_ = 1;
    commandDirection_ = 0;

    // Start Time
    auto start = std::chrono::high_resolution_clock::now();
    // Loop
    while(ros::ok()){
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        long long seconds = microseconds * 1e-6;
        
        // // Get Swing & Stance Trajectory Coordinates for time = seconds
        // auto stanceCoordinate = traj.stancePhaseTrajectory(seconds);
        // auto supportCoordinate = traj.supportPhaseTrajectory(seconds);

        // // Transform Trajectory Coordinates to Hip Frame and Rotate based on commandDirection
        // double x, y, z;

        // // Perform Inverse Kinematics & Obtain Joint Angles
        // auto leg1 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 1) ,1);
        // auto leg2 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 2), 2);
        // auto leg3 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 3), 3);
        // auto leg4 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 4), 4);

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

        ros::spinOnce();

        // Reset time after half gait cycle is complete
        if(microseconds >= gait_.tm * 1e+6)
            start = std::chrono::high_resolution_clock::now();
    }
    managerStatus = false;
    sensorData.join();
    driver.reset();
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
