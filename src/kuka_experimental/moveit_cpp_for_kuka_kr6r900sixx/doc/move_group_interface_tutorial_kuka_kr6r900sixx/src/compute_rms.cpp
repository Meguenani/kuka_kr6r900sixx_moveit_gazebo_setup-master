#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
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
#include "/usr/local/include/pcl-1.8/pcl/common/common.h"
#include "/usr/local/include/pcl-1.8/pcl/filters/crop_box.h"

#include "/usr/local/include/pcl-1.8/pcl/PCLPointCloud2.h"

#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Point.h>
#include <cmath>   
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <iterator>



using namespace std;


double compute_rms(std::vector<double> vect_a, std::vector<double> vect_b){
double rms = 0;

rms = sqrt(pow((vect_a[0]-vect_b[0]), 2)+pow((vect_a[1]-vect_b[1]), 2)+pow((vect_a[2]-vect_b[2]), 2)+pow((vect_a[3]-vect_b[3]), 2)+pow((vect_a[4]-vect_b[4]), 2)+pow((vect_a[5]-vect_b[5]), 2));

return rms;
}


