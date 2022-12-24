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

    allAxis= {master.axis("HA1"), master.axis("HF1"), master.axis("KF1"),
              master.axis("HA2"), master.axis("HF2"), master.axis("KF2"),
              master.axis("HA3"), master.axis("HF3"), master.axis("KF3"),
              master.axis("HA4"), master.axis("HF4"), master.axis("KF4")};

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

    // Initialize Odrives
    initializeOdrives(master, "position");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO_STREAM("Initialized Odrives");
    

    // Background Thread
    std::thread sensorData( [&master, this]() {
        while(master.is_ready() && managerStatus) {
            // // Update Joint Encoder Data
            // for(auto &a:allAxis){
            //     master.get_encoder_count(a);
            //     master.get_encoder_estimates(a);
            // }

            // master.get_iq(master.axis("HA4"));
            // master.get_iq(master.axis("HF4"));
            // master.get_iq(master.axis("KF4"));

            // ROS_INFO_STREAM(master.axis("HA4").idq_second<<" "<<master.axis("HF4").idq_second<<" "<<master.axis("KF4").idq_second);

            // encoderShadowCount_ = {{master.axis("HA1").encoder_shadow_count, master.axis("HF1").encoder_shadow_count, master.axis("KF1").encoder_shadow_count},
            //                        {master.axis("HA2").encoder_shadow_count, master.axis("HF2").encoder_shadow_count, master.axis("KF2").encoder_shadow_count},
            //                        {master.axis("HA3").encoder_shadow_count, master.axis("HF3").encoder_shadow_count, master.axis("KF3").encoder_shadow_count},
            //                        {master.axis("HA4").encoder_shadow_count, master.axis("HF4").encoder_shadow_count, master.axis("KF4").encoder_shadow_count}};

            // jointVelocity_ = {master.axis("HA1").vel_enc_estimate*2*3.14, master.axis("HF1").vel_enc_estimate*2*3.14, master.axis("KF1").vel_enc_estimate*2*3.14};
            
            // Test
            // ROS_INFO_STREAM(encoderShadowCount_[0][0]<<" "<<encoderShadowCount_[0][1]<<" "<<encoderShadowCount_[0][2]);
            // ROS_INFO_STREAM(encoderShadowCount_[0][1]<<" "<<encoderShadowCount_[1][1]<<" "<<encoderShadowCount_[2][1]<<" "<<encoderShadowCount_[3][1]);

            // // Check and Update Battery Voltage
            // master.get_vbus_voltage(allAxis[0]);
            // batteryVoltage_ = allAxis[0].vbus_voltage;
            // if(batteryVoltage_ < Claw::minBatteryVoltage){
            //     ROS_ERROR_STREAM("Battery Voltage Low: "<<batteryVoltage_);
            // }
            // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } 
    } );

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Loop
    ROS_INFO_STREAM("Loop");
    while(ros::ok()){
        move(master);
        poseManipulation(master);
        ros::spinOnce();
    }

    managerStatus = false;
    
    sensorData.join();

    idleOdrives(master);
    driver.reset();
}

std::vector<double> Manager::anglesToPosition(std::vector<double> angle, int n){
    return {(Claw::encoderOffset[n-1][0]/Claw::countsPerRevolution) + (Claw::encoderDirection[n-1][0]*(angle[0]*Claw::reductionHA)/(2*M_PI)),
            (Claw::encoderOffset[n-1][1]/Claw::countsPerRevolution) + (Claw::encoderDirection[n-1][1]*(angle[1]*Claw::reductionHF)/(2*M_PI)), 
            (Claw::encoderOffset[n-1][2]/Claw::countsPerRevolution) + (Claw::encoderDirection[n-1][2]*(angle[2]*Claw::reductionHF)/(2*M_PI))};
}

std::vector<double> Manager::positionToAngle(int n){
    return {Claw::encoderDirection[n-1][0]*(encoderShadowCount_[n-1][0] - Claw::encoderOffset[n-1][0]) * Claw::abductionCPRAngleRelation, 
            Claw::encoderDirection[n-1][1]*(encoderShadowCount_[n-1][1] - Claw::encoderOffset[n-1][1]) * Claw::flexionCPRAngleRelation, 
            Claw::encoderDirection[n-1][2]*(encoderShadowCount_[n-1][2] - Claw::encoderOffset[n-1][2]) * Claw::flexionCPRAngleRelation};
}

void Manager::move(odrive_can_ros::CANSimple &master){
    // Pair1 (Leg 1 & 3), Pair2 (Leg 2 & 4)
    int phaseReference = 1;
    int phasePair1 = 1, prevPhasePair1 = 1; 
    int phasePair2 = 0, prevPhasePair2 = 0; 
    int cyclePair1 = 0, cyclePair2 = 0;

    bool inMotion = false;

    auto startWalk = std::chrono::high_resolution_clock::now();
    
    // TEST
    int f = 0;
    bool allow1 = true;
    bool allow2 = false;

    double x, y, z;
    double xd, yd, zd;
    double Kx = 100, Dx = 1;
    double Ky = 10, Dy = 1;
    double Kz = 100, Dz = 1;
    double ex, ey, ez, edx, edy, edz;
    double pvx = 0;
    double pvy = 0;
    double pvz = 0;

    VelocityKinematics v;
    
    Gait gait(0.12, 0.1, 0.4, 1, "trot");
    while(state_ == "WALK" || inMotion){ 
        inMotion = true;       

        // When trotting in place delay should be tSwing + (tStance + tSwing)/2
        Trajectory traj(gait, "bezier");

        auto elapsed = std::chrono::high_resolution_clock::now() - startWalk;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        double timeReference = microseconds * 1e-6;
        double time1 = offsetTime(timeReference, phaseReference, phasePair1, 0, gait);
        double time2 = offsetTime(timeReference, phaseReference, phasePair2, 0.7, gait);


        if(!allow2 && !phasePair2 && time2 >= 0){
            allow2 = true;
        }

        // Reset time after half gait cycle is complete
        if(timeReference >= (phaseReference ? gait.tSwing : gait.tStance)){
            startWalk = std::chrono::high_resolution_clock::now();
            phaseReference = !phaseReference;
        }
        else{

            if(phasePair1!=prevPhasePair1){
                prevPhasePair1 = phasePair1;
                f++;
                if(cyclePair1 == -1)
                    allow1 = false;
                if(cyclePair1 == 1)
                    cyclePair1 = 0;
                if(state_ != "WALK"){
                    cyclePair1 = -1;
                    gait.tSwing = 0.7;
                }
            }

            if(phasePair2!=prevPhasePair2){
                prevPhasePair2 = phasePair2;
                f++;
                if(cyclePair2 == -1)
                    allow2 = false;
                if(cyclePair2 == 1)
                    cyclePair2 = 0;
                if(state_ != "WALK")
                    cyclePair2 = -1;
            }

            Point pair1, pair2;
            
            if(allow1){
                if(phasePair1){
                    // Get Swing Trajectory Coordinates for time = seconds, and transform it to legt frame
                    Eigen::Vector3d pSwing;
                    // if(time1 <= 0.55/2)
                    //     pSwing = {0, traj.jerkMinimizedTrajectory(0, 0, 0, 0.07, 0, 0, 0.55/2, time1), 0};
                    // else
                    //     pSwing = {0, traj.jerkMinimizedTrajectory(0.07, 0, 0, 0, 0, 0, 0.55/2, time1 - (0.55/2)), 0};
                    pSwing = traj.swingPhaseTrajectory(time1, cyclePair1);
                    pSwing = fk_.trajectoryToLegH(pSwing);
                    pair1 = {pSwing(0), pSwing(1), pSwing(2)};
                }
                else{
                    // Get Stance Trajectory Coordinates for time = seconds, and transform it to legt frame
                    Eigen::Vector3d  pStance;
                    // pStance = {0,0,0};
                    pStance = traj.stancePhaseTrajectory(time1, cyclePair1);
                    pStance = fk_.trajectoryToLegH(pStance);
                    pair1 = {pStance(0), pStance(1), pStance(2)};
                }
            }

            if(allow2){
                if(phasePair2){
                    // Get Swing Trajectory Coordinates for time = seconds, and transform it to legt frame
                    Eigen::Vector3d pSwing;
                    // if(time2 <= 0.55/2)
                    //     pSwing = {0, traj.jerkMinimizedTrajectory(0, 0, 0, 0.07, 0, 0, 0.55/2, time2), 0};
                    // else
                    //     pSwing = {0, traj.jerkMinimizedTrajectory(0.07, 0, 0, 0, 0, 0, 0.55/2, time2 - (0.55/2)), 0};
                    pSwing = traj.swingPhaseTrajectory(time2, cyclePair2);
                    pSwing = fk_.trajectoryToLegH(pSwing);
                    pair2 = {pSwing(0), pSwing(1), pSwing(2)};
                }
                else{
                    // Get Stance Trajectory Coordinates for time = seconds, and transform it to legt frame
                    Eigen::Vector3d  pStance;
                    // pStance = {0,0,0};
                    pStance = traj.stancePhaseTrajectory(time2, cyclePair2);
                    pStance = fk_.trajectoryToLegH(pStance);
                    pair2 = {pStance(0), pStance(1), pStance(2)};
                }
            }
            else{
                pair2 = {Claw::idleLegHeight, 0, 0};
            }

            // Test
            // if(f==10)
            //     state_ = "NONE";

            legState1 = InverseKinematics::computeJointAngles(pair1.x, pair1.y, pair1.z, 1);
            legState2 = InverseKinematics::computeJointAngles(pair2.x, pair2.y, pair2.z, 2);
            legState3 = InverseKinematics::computeJointAngles(pair1.x, pair1.y, pair1.z, 3);
            legState4 = InverseKinematics::computeJointAngles(pair2.x, pair2.y, pair2.z, 4);
            // legState3[1] = -legState3[1];
            // legState3[2] = -legState3[2];
            // legState4[1] = -legState4[1];
            // legState4[2] = -legState4[2];

            // std::vector<double> angleleg1 = positionToAngle(1);
            // ROS_INFO_STREAM(angleleg1[0]<<" "<<angleleg1[1]<<" "<<angleleg1[2]);
            // Eigen::Vector3d actual = fk_.legToFootH(angleleg1, 1);
            // ROS_INFO_STREAM(actual[0]<<" "<<actual[1]<<" "<<actual[2]);

            commandOdrives(master);
            statePublisher();

            // Eigen::MatrixXd J = v.jacobian(angleleg1, 1);

            // Eigen::VectorXd thetad(3);
            // thetad(0) = -jointVelocity_[0]/Claw::reductionHA;
            // thetad(1) = jointVelocity_[1]/Claw::reductionHF;
            // thetad(2) = -jointVelocity_[2]/Claw::reductionKF;
            // // ROS_INFO_STREAM(thetad(0)<<" "<<thetad(1)<<" "<<thetad(2));

            // Eigen::VectorXd eV = J*thetad;

            // double vx = eV(0);
            // double vy = eV(1);
            // double vz = eV(2);
            // // ROS_INFO_STREAM(vx<<" "<<vy<<" "<<vz);


            // // Command Torque
            // xd = 0.35; //pair1.x;
            // yd = 0; //pair1.y;
            // zd = 0; //pair1.z;
            // x = actual(0);
            // y = actual(1);
            // z = actual(2);
            // ex = xd-x;
            // ey = yd-y;
            // ez = zd-z;
            // edx = 0 - vx;
            // edy = 0 - vy;
            // edz = 0 - vz;
            // pvx = vx;
            // pvy = vy;
            // pvz = vz;


            // Eigen::VectorXd f(6);
            // f(0)= Kx*ex + Dx*edx;
            // f(1)= Ky*ey + Dy*edy; 
            // f(2)= Kz*ez + Dz*edz; 
            // f(3)= 0; 
            // f(4)= 0; 
            // f(5)= 0;
            
            // Eigen::MatrixXd Jt = J.transpose();
            // // std::cout<<Jt<<"\n";
            
            // Eigen::VectorXd torque = Jt*f;
            // std::vector<double> tq;
            // tq.push_back(-torque(0)/Claw::reductionHA);
            // tq.push_back(torque(1)/Claw::reductionHF);
            // tq.push_back(-torque(2)/Claw::reductionHF);

            // ROS_INFO_STREAM(torque(0)<<" "<<torque(1)<<" "<<torque(2));
            // ROS_INFO_STREAM(tq[0]<<" "<<tq[1]<<" "<<tq[2]);

            // for(int i = 0; i<3; i++){
            //     master.set_input_torque(allAxis[i], tq[i]);
            // }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if(!allow1 && !allow2)
                break;
        }
        ros::spinOnce();
    }
}

void Manager::poseManipulation(odrive_can_ros::CANSimple &master){
    auto start = std::chrono::high_resolution_clock::now();
    Gait gait;
    Trajectory traj(gait, "test");
    double s = 0;
    double g = -0.23;

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    while(state_ == "MOVE_BASE"){
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        double timeReference = microseconds * 1e-6;

        transform.setOrigin( tf::Vector3(base.x, base.y, base.z) );
        tf::Quaternion q;
        q.setRPY(base.roll, base.pitch, base.yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_world", "base_link"));

        if(timeReference >= 5){
            start = std::chrono::high_resolution_clock::now();
            double temp = s;
            s = g;
            g = temp;
        }
        else{

            base.x = traj.jerkMinimizedTrajectory(0, 0, 0, 0, 0, 0, 5, timeReference);
            base.y = traj.jerkMinimizedTrajectory(0, 0, 0, 0, 0, 0, 5, timeReference);
            base.z = traj.jerkMinimizedTrajectory(0, 0, 0, 0, 0, 0, 5, timeReference);
            base.roll = traj.jerkMinimizedTrajectory(0, 0, 0, 0, 0, 0, 5, timeReference);
            base.pitch = traj.jerkMinimizedTrajectory(s, 0, 0, g, 0, 0, 5, timeReference);
            base.yaw = traj.jerkMinimizedTrajectory(0, 0, 0, 0, 0, 0, 5, timeReference);
            
            double reduceLegHeightBy = 0;
            Eigen::Vector3d foot1World = {Claw::bodyTF[0][0], Claw::bodyTF[0][1]+0.05, Claw::bodyTF[0][2] - Claw::idleLegHeight - reduceLegHeightBy};
            Eigen::Vector3d foot2World = {Claw::bodyTF[1][0], Claw::bodyTF[1][1]-0.05, Claw::bodyTF[1][2] - Claw::idleLegHeight - reduceLegHeightBy};
            Eigen::Vector3d foot3World = {Claw::bodyTF[2][0]-0.01, Claw::bodyTF[2][1]-0.05, Claw::bodyTF[2][2] - Claw::idleLegHeight - reduceLegHeightBy};
            Eigen::Vector3d foot4World = {Claw::bodyTF[3][0]-0.01, Claw::bodyTF[3][1]+0.05, Claw::bodyTF[3][2] - Claw::idleLegHeight - reduceLegHeightBy};

            Eigen::Vector3d foot1Leg = fk_.footInLegFrame(base.x, base.y, base.z, base.roll, base.pitch, base.yaw, foot1World, 1);
            Eigen::Vector3d foot2Leg = fk_.footInLegFrame(base.x, base.y, base.z, base.roll, base.pitch, base.yaw, foot2World, 2);
            Eigen::Vector3d foot3Leg = fk_.footInLegFrame(base.x, base.y, base.z, base.roll, base.pitch, base.yaw, foot3World, 3);
            Eigen::Vector3d foot4Leg = fk_.footInLegFrame(base.x, base.y, base.z, base.roll, base.pitch, base.yaw, foot4World, 4);

            legState1 = InverseKinematics::computeJointAngles(foot1Leg(0), foot1Leg(1), -foot1Leg(2), 1);
            legState2 = InverseKinematics::computeJointAngles(foot2Leg(0), foot2Leg(1), -foot2Leg(2), 2);
            legState3 = InverseKinematics::computeJointAngles(foot3Leg(0), foot3Leg(1), -foot3Leg(2), 3);
            legState4 = InverseKinematics::computeJointAngles(foot4Leg(0), foot4Leg(1), -foot4Leg(2), 4);

            commandOdrives(master);
            statePublisher();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ros::spinOnce();
    }
}

double Manager::offsetTime(double timeReference, int phaseReference, int &phasePair, double delay, Gait gait){
    double tnew = timeReference - delay;
    if(tnew>=0){
        phasePair = phaseReference;
        return tnew;
    }
    if(tnew < 0 && phaseReference){
        phasePair = 0;
        return gait.tStance + tnew;
    }
    if(tnew < 0 && !phaseReference){
        if(gait.tSwing + tnew >=0){
            phasePair = 1;
            return gait.tSwing + tnew;
        }
        phasePair = 0;
        return gait.tSwing + gait.tStance + tnew;
    }
    return tnew;
}

void Manager::initializeOdrives(odrive_can_ros::CANSimple &master, std::string controlMode){
    // Clear All Axis
    for(auto &a:allAxis){
        master.clear_errors(a);
    }

    odrive_can_ros::ControlMode mode;
    if(controlMode == "position")
        mode = odrive_can_ros::ControlMode::CONTROL_MODE_POSITION_CONTROL;
    else if(controlMode == "torque")
        mode = odrive_can_ros::ControlMode::CONTROL_MODE_TORQUE_CONTROL;

    // Start Position Control on All Axis
    for(int i = 0; i<allAxis.size(); i++){
        master.set_controller_modes(allAxis[i], mode, odrive_can_ros::InputMode::INPUT_MODE_PASSTHROUGH);
        master.set_axis_requested_state(allAxis[i], odrive_can_ros::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
}


void Manager::idleOdrives(odrive_can_ros::CANSimple &master){
    // idle All Axis
    for(auto &a:allAxis){
        master.set_axis_requested_state(a, odrive_can_ros::AxisState::AXIS_STATE_IDLE);
    } 
}

void Manager::commandOdrives(odrive_can_ros::CANSimple &master){

    // Joint Angles to Counts
    std::vector<std::vector<double>> counts;
    counts.push_back(anglesToPosition(legState1, 1));
    counts.push_back(anglesToPosition(legState2, 2));
    counts.push_back(anglesToPosition(legState3, 3));
    counts.push_back(anglesToPosition(legState4, 4));

    // Send Position Commands
    for(int i = 0; i<allAxis.size(); i++){
        master.set_input_pos(allAxis[i], counts[i/3][i%3]);
    }
}


void Manager::statePublisher(){
    sensor_msgs::JointState state;
    state.header.frame_id = "";
    state.header.stamp = ros::Time::now();
    state.name  = {"HA1", "HF1", "KF1", "HA2", "HF2", "KF2", "HA3", "HF3", "KF3", "HA4", "HF4", "KF4"};
    state.position.resize(state.name.size());
    state.position[0] = legState1[0];
    state.position[1] = legState1[1];
    state.position[2] = legState1[2];
    state.position[3] = legState2[0];
    state.position[4] = legState2[1];
    state.position[5] = legState2[2];
    state.position[6] = legState3[0];
    state.position[7] = legState3[1];
    state.position[8] = legState3[2];
    state.position[9] = legState4[0];
    state.position[10] = legState4[1];
    state.position[11] = legState4[2];
    jointStatePublisher_.publish(state);
}