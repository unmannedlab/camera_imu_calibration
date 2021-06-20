cmake_minimum_required(VERSION 2.8.8)
project(camera_imu_calibration)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin QUIET COMPONENTS
        roscpp
        rosbag
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        tf
        imuPacket
)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs imuPacket
            INCLUDE_DIRS src
            LIBRARIES camimucalib_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )

##################################################
# Make the core library
##################################################
add_library(camimucalib_lib SHARED
            src/init/InertialInitializer.cpp
            src/state/State.cpp
            src/state/StateHelper.cpp
            src/state/Propagator.cpp
            src/track/cameraTracking.cpp)

target_link_libraries(camimucalib_lib ${thirdparty_libraries})
target_include_directories(camimucalib_lib PUBLIC src)

##################################################
# Adding different executables
##################################################
#add_executable(ros_test_node src/ros_linkalibr_test.cpp)
#target_link_libraries(ros_test_node linkalibr_lib ${catkin_LIBRARIES})








