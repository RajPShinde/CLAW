#include <Manager.hpp>

Manager::Manager(){
    cprAngleRelation_ = claw_.countsPerRevolution * claw_.gearReduction / 360;
    begin();
}

Manager::~Manager(){
}

void Manager::begin(){
    CANSimple master;
    if ( !( master.add_axis(1, "odrive_axis_1") && master.add_axis(2, "odrive_axis_2") &&
            master.add_axis(3, "odrive_axis_3") && master.add_axis(4, "odrive_axis_4") &&
            master.add_axis(5, "odrive_axis_5") && master.add_axis(6, "odrive_axis_6") &&
            master.add_axis(7, "odrive_axis_7") && master.add_axis(8, "odrive_axis_8") &&
            master.add_axis(9, "odrive_axis_9") && master.add_axis(10, "odrive_axis_10") &&
            master.add_axis(11, "odrive_axis_11") && master.add_axis(12, "odrive_axis_12")))
    {
        fprintf(stderr, "Failed to create one or more axis. Aborting.\n");
        return -1;
    }

    // Create Interface to SocketCAN 
    const std::string canDevice_ = "can0";
    can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();
    if (!driver->init(canDevice_, 0, can::NoSettings::create()))
    {
        fprintf(stderr, "Failed to initialize can device at %s\n", canDevice_.c_str());
        return -1;
    }
    can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
        [&driver](const can::State& s) {
            std::string err;
            driver->translateError(s.internal_error, err);
            fprintf(stderr, "CAN Device error: %s, asio: %s.\n", 
                err.c_str(), s.error_code.message().c_str());
        }
    );

    // Pass the SocketCAN handle to master
    master.init(driver);
    std::this_thread::sleep_for(500ms);

    // Background Thread
    std::thread thr( [&master]() {
        while(master.is_ready()) {
            fprintf(stdout, "Axis 1: [pos] = [%4.3f]\n", master.axis("odrive_axis_1").pos_enc_estimate);
            
            // Check Battery Voltage
            if(batteryVoltage_ = master.axis("odrive_axis_1").vbus_voltage < claw.mimBatteryVoltage){
                fprintf(stderr, "Battery Voltage Low\n");
                return -1;
            }
            std::this_thread::sleep_for(500ms);
        } 
    } );

    // Initialize Legs


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
        auto stanceCoordinate = legStanceTrajectory(seconds);
        auto supportCoordinate = legSupportTrajectory(seconds);

        // Transform Trajectory Coordinates to Hip Frame and Rotate based on commandDirectiomn

        // Perform Inverse Kinematics & Obtain Joint Angles
        auto leg1 = anglesToPosition(ik_.computeJointAngles(, 1) ,1);
        auto leg2 = anglesToPosition(ik_.computeJointAngles(, 2), 2);
        auto leg3 = anglesToPosition(ik_.computeJointAngles(, 3), 3);
        auto leg4 = anglesToPosition(ik_.computeJointAngles(, 4), 4);

        // Send Joint Positions to respective odrive axis
        master.set_input_pos(master.axis("HA1"), leg1[0]);
        master.set_input_pos(master.axis("HA2"), leg2[0]);
        master.set_input_pos(master.axis("HA3"), leg3[0]);
        master.set_input_pos(master.axis("HA4"), leg4[0]);

        master.set_input_pos(master.axis("HF1"), leg1[1]);
        master.set_input_pos(master.axis("HF2"), leg2[1]);
        master.set_input_pos(master.axis("HF3"), leg3[1]);
        master.set_input_pos(master.axis("HF4"), leg4[1]);

        master.set_input_pos(master.axis("KF1"), leg1[2]);
        master.set_input_pos(master.axis("KF2"), leg2[2]);
        master.set_input_pos(master.axis("KF3"), leg3[2]);
        master.set_input_pos(master.axis("KF4"), leg4[2]);

    // Reset time after half gait cycle is complete
    if(microseconds >= gait_.tm * 1e+6)
        start = std::chrono::high_resolution_clock::now();
    }
}

std::vector<int> Manager::anglesToPosition(std::vector<double> angle, int n){
    return {claw_.encoderOffset[n-1][0] + angle[0] * cprAngleRelation_, claw_.encoderOffset[n-1][1] + angle[1] * cprAngleRelation_, claw_.encoderOffset[n-1][2] + angle[2] * cprAngleRelation_};
}

std::vector<double> Manager::positionToAngle(std::vector<int> position, int n){
    return {(position[0] - claw_.encoderOffset[n-1][0]) / cprAngleRelation_, (position[1] - claw_.encoderOffset[n-1][1]) / cprAngleRelation_, (position[2] - claw_.encoderOffset[n-1][2]) / cprAngleRelation_};
}
