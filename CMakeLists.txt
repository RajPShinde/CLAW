cmake_minimum_required(VERSION 3.2.3)
project (claw)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED
			 roscpp
			 rospy
			 roslaunch
			 std_msgs
			 geometry_msgs
			 tf
			 tf2
			)

catkin_package(INCLUDE_DIRS include
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

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(claw src/main.cpp src/manager.cpp src/claw.cpp src/forwardKinematics.cpp src/inverseKinematics.cpp src/velocityKinematics.cpp src/trajectory.cpp src/gait.cpp src/polynomial.cpp)
target_link_libraries(claw ${catkin_LIBRARIES})
add_dependencies(claw ${catkin_EXPORTED_TARGETS})
