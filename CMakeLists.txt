cmake_minimum_required(VERSION 3.10.0)
project (claw)

set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/version.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/include/version.hpp
)

string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(BoldRed     "${Esc}[1;31m")
set(BoldGreen   "${Esc}[1;32m")
set(BoldYellow  "${Esc}[1;33m")
set(BoldMagenta "${Esc}[1;35m")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "${BoldMagenta}Architecture: ${BoldYellow}${ARCHITECTURE}${ColourReset}" )

cmake_host_system_information(RESULT PROCESSORDESCRIPTION QUERY PROCESSOR_DESCRIPTION)
message(STATUS "${BoldMagenta}Processor: ${BoldYellow}${PROCESSORDESCRIPTION}${ColourReset}")

cmake_host_system_information(RESULT CORES QUERY NUMBER_OF_PHYSICAL_CORES)
cmake_host_system_information(RESULT THREADS QUERY NUMBER_OF_LOGICAL_CORES)
message(STATUS "${BoldMagenta}Cores: ${BoldYellow}${CORES} ${BoldMagenta}Threads: ${BoldYellow}${THREADS}${ColourReset}")

option(MARCH_NATIVE "${BoldYellow}Building with -march=native${ColourReset}" OFF)

set(CMAKE_CXX_STANDARD 17)

if(MARCH_NATIVE)
  add_compile_options(-march=native)
  message(STATUS "${BoldRed}Building with -march=native${ColourReset}")
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED
			 roscpp
			 rospy
			 roslaunch
			 std_msgs
			 geometry_msgs
			 tf
			 tf2
			 socketcan_interface
			 visualization_msgs
			 urdf
			)

catkin_package(INCLUDE_DIRS 
			   include
			  )

find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

include_directories(include 
                    lib/odrive_can/include/ 
					lib/ADS1115/ADS1115/include
					lib/ADS1115/impl/unix/include
					${catkin_INCLUDE_DIRS})

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

add_library(kinematics SHARED src/forwardKinematics.cpp src/inverseKinematics.cpp 
			src/velocityKinematics.cpp src/trajectory.cpp src/polynomial.cpp)


add_executable(clawHardwareInterface src/main.cpp src/manager.cpp src/hardwareInterface.cpp src/contactSensor.cpp src/contactSensorHandle.cpp src/hybridJointHandle.cpp src/status.cpp 
                                     lib/odrive_can/src/can_simple.cpp lib/ADS1115/impl/unix/src/unix.cpp)
target_link_libraries(clawHardwareInterface ${catkin_LIBRARIES})
add_dependencies(clawHardwareInterface ${catkin_EXPORTED_TARGETS})
