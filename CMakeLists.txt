cmake_minimum_required(VERSION 3.0.2)
project(unordered_sorting)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
hirop_msgs
std_msgs
std_srvs
sensor_msgs
cv_bridge
image_transport
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unordered_sorting
#  CATKIN_DEPENDS roscpp rosmsg rospy
#  DEPENDS system_lib
)


include_directories(
 include
3rd/include/
3rd/include/proxy
  ${catkin_INCLUDE_DIRS}
)

add_executable(unordered_sorting
        src/main.cpp
        src/VisionClient.cpp
        src/UnOrderSortingServer.cpp
        src/RobSortingMotion.cpp
        src/Hsc3apiInstance.cpp
        src/SolomonCalibrate.cpp
        )

target_link_libraries(unordered_sorting
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
        ${OpenCV_LIBS}
        dl
        CommApi
        Hsc3Api
        LogApi
        #        boost_chrono
        #        boost_date_time
        #        boost_thread
        #        boost_system
        #        boost_filesystem
        )


add_executable(testp
         src/test.cpp
        )

target_link_libraries(testp
        ${catkin_LIBRARIES}
        )