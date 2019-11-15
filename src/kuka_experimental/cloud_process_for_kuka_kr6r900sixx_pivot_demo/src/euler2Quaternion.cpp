#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <dirent.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry> 
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/LU> 
#include <Eigen/Core>
#include <atomic>
#include <chrono>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "/usr/local/include/pcl-1.8/pcl/io/io.h"
#include "/usr/local/include/pcl-1.8/pcl/io/pcd_io.h"
#include "/usr/local/include/pcl-1.8/pcl/point_cloud.h"
#include "/usr/local/include/pcl-1.8/pcl/console/parse.h"
#include "/usr/local/include/pcl-1.8/pcl/common/transforms.h"
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Point.h>
#include <cmath>   
#include "gazebo_msgs/ModelStates.h"








Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
        return q;
}
