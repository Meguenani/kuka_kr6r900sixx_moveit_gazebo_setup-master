cmake_minimum_required(VERSION 2.8.3)
project(moveit_cpp_for_kuka_kr6r900sixx)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
    tf
    rospy
    std_msgs
    sensor_msgs
    gazebo_msgs
    cv_bridge
    image_transport
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PkgConfig)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

if(EIGEN_FOUND)
  message(STATUS "Eigen found !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (include: ${EIGEN_INCLUDE_DIRS})")
endif(EIGEN_FOUND) 

if(EIGEN_NOT_FOUND)
  message(STATUS "Eigen not found !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
endif(EIGEN_NOT_FOUND) 



find_package(PCL 1.8 REQUIRED)

if(PCL_FOUND)
  message(STATUS "PCL found !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (include: ${PCL_INCLUDE_DIRS})")
endif(PCL_FOUND) 

if(PCL_NOT_FOUND)
  message(STATUS "PCL not found !!!")
endif(PCL_NOT_FOUND) 




find_package(VTK 8.2 REQUIRED)
if(VTK_FOUND)
  message(STATUS "VTK found 888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888 (include: ${VTK_INCLUDE_DIRS})")
endif(VTK_FOUND) 

if(VTK_NOT_FOUND)
  message(STATUS "VTK not found 888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888")
endif(VTK_NOT_FOUND)





catkin_package(
  LIBRARIES
    interactivity_utils


  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########
  message(STATUS "PROJECT_SOURCE_DIR 888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888 : ${PROJECT_SOURCE_DIR}")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/doc/move_group_interface_tutorial_kuka_kr6r900sixx/include)
link_directories(${catkin_LIBRARY_DIRS}
                 ${LD_LIBRARY_PATH}
                 ${EIGEN3_LIBRARY_DIRS}
                 ${HOME}/libs/opencv-2.4/build/lib
                 ${PCL_LIBRARY_DIRS}
                 ${PCL_INCLUDE_DIRS})


add_subdirectory(doc/move_group_interface_tutorial_kuka_kr6r900sixx)
