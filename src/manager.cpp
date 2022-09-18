#include <manager.hpp>

Manager::Manager(ros::NodeHandle &nh) {
    ROS_INFO_STREAM("Initializing Robot");
    begin();
}

Manager::~Manager(){
}

int Manager::begin(){
    odrive_can_ros::CANSimple master;
    if ( !( master.add_axis(1, "HF1") && master.add_axis(2, "KF1") &&
            master.add_axis(3, "HA1") && master.add_axis(4, "HA2") &&
            master.add_axis(5, "HF2") && master.add_axis(6, "KF2") &&
            master.add_axis(7, "HF3") && master.add_axis(8, "KF3") &&
            master.add_axis(9, "HA3") && master.add_axis(10, "HA4") &&
            master.add_axis(11, "HF4") && master.add_axis(12, "KF4")))
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
        }
    );

    // Pass the SocketCAN handle to master
    master.init(driver);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Background Thread
    std::thread sensorData( [&master, this]() {
        while(master.is_ready() && managerStatus) {
            // Update Joint Angles
            jointAngles_ = {{master.axis("HA1").pos_enc_estimate, master.axis("HF1").pos_enc_estimate, master.axis("KF1").pos_enc_estimate},
                            {master.axis("HA2").pos_enc_estimate, master.axis("HF2").pos_enc_estimate, master.axis("KF2").pos_enc_estimate},
                            {master.axis("HA3").pos_enc_estimate, master.axis("HF3").pos_enc_estimate, master.axis("KF3").pos_enc_estimate},
                            {master.axis("HA4").pos_enc_estimate, master.axis("HF4").pos_enc_estimate, master.axis("KF4").pos_enc_estimate}};
            
            // Update and Check Battery Voltage
            batteryVoltage_ = master.axis("HA1").vbus_voltage;
            if(batteryVoltage_ < Claw::minBatteryVoltage){
                ROS_ERROR_STREAM("Battery Voltage Low");
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } 
    } );

    // Initialize Legs
    Gait gait_(2, 2, 2, 2, "test");
    Trajectory traj(gait_);

    // Temporary commands
    commandValue_ = 1;
    commandDirection_ = 0;

    // Start Time
    auto start = std::chrono::high_resolution_clock::now();
    // Loop
    while(true){
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        long long seconds = microseconds * 1e-6;
        
        // Get Swing & Stance Trajectory Coordinates for time = seconds
        auto stanceCoordinate = traj.stancePhaseTrajectory(seconds);
        auto supportCoordinate = traj.supportPhaseTrajectory(seconds);

        // Transform Trajectory Coordinates to Hip Frame and Rotate based on commandDirection
        double x, y, z;

        // Perform Inverse Kinematics & Obtain Joint Angles
        auto leg1 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 1) ,1);
        auto leg2 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 2), 2);
        auto leg3 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 3), 3);
        auto leg4 = anglesToPosition(InverseKinematics::computeJointAngles(x, y, z, 4), 4);

        // Send Joint Positions to respective odrive axis
        // Hip Abduction
        master.set_input_pos(master.axis("HA1"), leg1[0]);
        master.set_input_pos(master.axis("HA2"), leg2[0]);
        master.set_input_pos(master.axis("HA3"), leg3[0]);
        master.set_input_pos(master.axis("HA4"), leg4[0]);

        // Hip Flexion
        master.set_input_pos(master.axis("HF1"), leg1[1]);
        master.set_input_pos(master.axis("HF2"), leg2[1]);
        master.set_input_pos(master.axis("HF3"), leg3[1]);
        master.set_input_pos(master.axis("HF4"), leg4[1]);

        // Knee Flexion
        master.set_input_pos(master.axis("KF1"), leg1[2]);
        master.set_input_pos(master.axis("KF2"), leg2[2]);
        master.set_input_pos(master.axis("KF3"), leg3[2]);
        master.set_input_pos(master.axis("KF4"), leg4[2]);

        ros::spinOnce();

        // Reset time after half gait cycle is complete
        if(microseconds >= gait_.tm * 1e+6)
            start = std::chrono::high_resolution_clock::now();
    }
    managerStatus = false;
    sensorData.join();
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
