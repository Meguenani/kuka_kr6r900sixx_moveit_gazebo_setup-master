/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//
//#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <iostream>
//using namespace std;
//
//
//
//
//
//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "move_group_interface_tutorial_kuka_kr6r900sixx");
//  ros::NodeHandle node_handle;
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
//
//  // BEGIN_TUTORIAL
//  //
//  // Setup
//  // ^^^^^
//  //
//  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
//  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
//  // are used interchangably.
//  static const std::string PLANNING_GROUP = "manipulator";
//
//  // The :move_group_interface:`MoveGroup` class can be easily
//  // setup using just the name of the planning group you would like to control and plan for.
//  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//
//  // We will use the :planning_scene_interface:`PlanningSceneInterface`
//  // class to add and remove collision objects in our "virtual world" scene
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//
//  // Raw pointers are frequently used to refer to the planning group for improved performance.
//  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//
//
//
//
//
//
//
//
//  // Visualization
//  // ^^^^^^^^^^^^^
//  //
//  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
//  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
//  namespace rvt = rviz_visual_tools;
//  moveit_visual_tools::MoveItVisualTools visual_tools("tool0");
//  visual_tools.deleteAllMarkers();
//
//  // Remote control is an introspection tool that allows users to step through a high level script
//  // via buttons and keyboard shortcuts in RViz
//  visual_tools.loadRemoteControl();
//
//  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
//  text_pose.translation().z() = 1.75;
//  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
//
//
//
//  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//  visual_tools.trigger();
//
//  // Getting Basic Information
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // We can print the name of the reference frame for this robot.
//  ROS_INFO_NAMED("tutorial", "Reference frame !!!!!!!!: %s", move_group.getPlanningFrame().c_str());
//
//  // We can also print the name of the end-effector link for this group.
//  ROS_INFO_NAMED("tutorial", "End effector link !!!!!!: %s", move_group.getEndEffectorLink().c_str());
//
//
//
//
//
//
//  // Start the demo
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
//
//  int i;
//  cout << "Please enter 1 to start the demo ";
//  cin >> i;
//
//  // Planning to a Pose goal
//  // ^^^^^^^^^^^^^^^^^^^^^^^
//  // We can plan a motion for this group to a desired pose for the
//  // end-effector.
//  geometry_msgs::Pose target_pose1;
//
//
//  target_pose1.orientation.x =  0.0098242;
//  target_pose1.orientation.y =  0.99537;
//  target_pose1.orientation.z =  0.021263;
//  target_pose1.orientation.w = -0.093181;
//
//
//  target_pose1.position.x = 0.67866;
//  target_pose1.position.y = 0.0035329;
//  target_pose1.position.z = 0.6933;
//  move_group.setPoseTarget(target_pose1);
//
//
//  // Now, we call the planner to compute the plan and visualize it.
//  // Note that we are just planning, not asking move_group
//  // to actually move the robot.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//
//  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//
//
//
//  // Visualizing plans
//  // ^^^^^^^^^^^^^^^^^
//  // We can also visualize the plan as a line with markers in RViz.
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//  visual_tools.publishAxisLabeled(target_pose1, "pose1");
//  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//
//
//  //Move to the planned position
//  move_group.move();
//
//
//  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//
//
//  cout << "Please enter 1 to continue the demo ";
//  cin >> i;
//
//  // Moving to a pose goal
//  // ^^^^^^^^^^^^^^^^^^^^^
//  //
//  // Moving to a pose goal is similar to the step above
//  // except we now use the move() function. Note that
//  // the pose goal we had set earlier is still active
//  // and so the robot will try to move to that goal. We will
//  // not use that function in this tutorial since it is
//  // a blocking function and requires a controller to be active
//  // and report success on execution of a trajectory.
//
//  /* Uncomment below line when working with a real robot */
//  /* move_group.move(); */
//
//  // Planning to a joint-space goal
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // Let's set a joint space goal and move towards it.  This will replace the
//  // pose target we set above.
//  //
//  // To start, we'll create a pointer that references the current robot's state.
//  // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions;
//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//
//  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//  joint_group_positions[0] = -1.0;  // radians
//  move_group.setJointValueTarget(joint_group_positions);
//
//  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//
//  //Move to the planned position
//  move_group.move();
//
//  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//  cout << "Please enter 1 to continue the demo ";
//  cin >> i;
//
//
//
//
//
//
//  // Since we set the start state we have to clear it before planning other paths
//  move_group.setStartStateToCurrentState();
//
//  // Cartesian Paths
//  // ^^^^^^^^^^^^^^^
//  // You can plan a Cartesian path directly by specifying a list of waypoints
//  // for the end-effector to go through. Note that we are starting
//  // from the new start state above.  The initial pose (start state) does not
//  // need to be added to the waypoint list but adding it can help with visualizations
//  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
//
//  std::vector<geometry_msgs::Pose> waypoints;
//  waypoints.push_back(target_pose3);
//
//  target_pose3.position.z -= 0.2;
//  waypoints.push_back(target_pose3);  // down
//
//  target_pose3.position.y -= 0.2;
//  waypoints.push_back(target_pose3);  // right
//
//  target_pose3.position.z += 0.2;
//  target_pose3.position.y += 0.2;
//  target_pose3.position.x -= 0.2;
//  waypoints.push_back(target_pose3);  // up and left
//
//  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
//  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
//  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
//  move_group.setMaxVelocityScalingFactor(0.1);
//
//  // We want the Cartesian path to be interpolated at a resolution of 1 cm
//  // which is why we will specify 0.01 as the max step in Cartesian
//  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
//  // Warning - disabling the jump threshold while operating real hardware can cause
//  // large unpredictable motions of redundant joints and could be a safety issue
//  moveit_msgs::RobotTrajectory trajectory;
//  const double jump_threshold = 0.0;
//  const double eef_step = 0.01;
//  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//  for (std::size_t i = 0; i < waypoints.size(); ++i)
//    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//  visual_tools.trigger();
//
//
//  //Move to the planned position
//  move_group.move();
//
//  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//
//
//  cout << "Please enter 1 to END the demo ";
//  cin >> i;
//
//
//
//  // END_TUTORIAL
//
//  ros::shutdown();
//  return 0;
//}

































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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

//#include </usr/local/include/pcl-1.8/pcl/visualization/cloud_viewer.h>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_XYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_XYZRGB;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans_in_base_link (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans_in_pivot_link (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans_in_pivot_link_just_translated (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr recombined_pivot_cloud (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_recombined_pivot_cloud (new PointCloud_XYZRGB);


//To transfor the partial pivot pointcloud 
Eigen::Vector3f offset; 
Eigen::Quaternionf rotation;

Eigen::Vector3f offset_inverse; 

Eigen::Vector3f offset_null; 
Eigen::Quaternionf rotation_identity;

Eigen::Vector3f offset_pose; 
Eigen::Quaternionf rotation_pose;

Eigen::Vector3f offset_pose1; 
Eigen::Quaternionf rotation_pose1;

Eigen::Vector3f offset_prev; 
Eigen::Quaternionf rotation_prev;

Eigen::Vector3f offset_depth_sensor; 
Eigen::Quaternionf rotation_depth_sensor;

Eigen::Vector3f offset_btw_two_poses; 
Eigen::Quaternionf rotation_btw_two_poses;

int initialize = 1;

std::vector<double> joint_group_positions;

geometry_msgs::Point pp ;
geometry_msgs::Quaternion qq ;
geometry_msgs::Twist current_Twist ;


std::string modelName = (std::string)"cylinder";
gazebo_msgs::GetModelState getModelState;


std::ostringstream saved_pointcloud_name;

Eigen::VectorXd joint_positions(6); 



//Here we receive the partial pivot pointclouds
void pivot_cloud_Callback(const PointCloud_XYZRGB::ConstPtr& pivot_cloud_msg){

    pcl::copyPointCloud(*pivot_cloud_msg, *pivot_cloud);
     
    //remove NaN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pivot_cloud,*pivot_cloud_without_nans, indices);
}









void next_button_callback(const sensor_msgs::Joy::ConstPtr& joy){
  
  //cout buttons
  std::cout<<"joy buttons : "<< joy->buttons[3] <<std::endl;
  

  //Here we save the current robot state
  //save_current_robot_state(joint_group_positions);


  //Then we concatenate the partial pointclouds


/*
  // Now, let's modify  the joints, plan to the new joint space goal and move towards it
  joint_group_positions[0] = -0.014958316846041875;  // radians
  joint_group_positions[1] = -1.0537766915280988 ;
  joint_group_positions[2] =  1.439558418250959   ;
  joint_group_positions[3] =  0.046518732721668954;
  joint_group_positions[4] =  1.186862317693988   ;
  joint_group_positions[5] = -0.04605105974771462;
*/


if(joy->buttons[3] == 1){
recombined_pivot_cloud->points.clear();
pivot_cloud->points.clear();
pivot_cloud_without_nans->points.clear();
pivot_cloud_without_nans_in_pivot_link_just_translated->points.clear();
pivot_cloud_without_nans_in_pivot_link->points.clear();

pcl::PointXYZRGB Pt;
Pt.x = 0.0;
Pt.y = 0.0;
Pt.z = 0.0;
Pt.r = 0.0;
Pt.g = 0.0;
Pt.b = 0.0; 

recombined_pivot_cloud->push_back(Pt);

pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
}


if(joy->buttons[1] == 1){
    offset_pose[0]    = pp.x;
    offset_pose[1]    = pp.y;
    offset_pose[2]    = pp.z;
    
    rotation_pose.x() = qq.x;
    rotation_pose.y() = qq.y;
    rotation_pose.z() = qq.z;
    rotation_pose.w() = qq.w;

    
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset_pose, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation_pose.inverse());

    
    //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;

     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR CURRENT POSE  !!!!!!!!!! "<<std::endl;
     std::cout << " "<<std::endl;
     std::cout<<"<group_state name=\"cloud_acq_posei_face_j_2\" group=\"manipulator\">" <<std::endl;
     std::cout<<"<joint name=\"joint_a1\" value=\""<<joint_positions[0]<<"\" />"<<std::endl;
     std::cout<<"<joint name=\"joint_a2\" value=\""<<joint_positions[1]<<"\" />"<<std::endl;
     std::cout<<"<joint name=\"joint_a3\" value=\""<<joint_positions[2]<<"\" />"<<std::endl;
     std::cout<<"<joint name=\"joint_a4\" value=\""<<joint_positions[3]<<"\" />"<<std::endl;
     std::cout<<"<joint name=\"joint_a5\" value=\""<<joint_positions[4]<<"\" />"<<std::endl;
     std::cout<<"<joint name=\"joint_a6\" value=\""<<joint_positions[5]<<"\" />"<<std::endl;
     std::cout<<"</group_state>"<<std::endl;
     std::cout << " "<<std::endl;
     std::cout << " "<<std::endl;

}


}







void joints_state_callback(const sensor_msgs::JointState::ConstPtr& joint_states){

     joint_positions[0] = joint_states->position[3];
     joint_positions[1] = joint_states->position[4];
     joint_positions[2] = joint_states->position[5];
     joint_positions[3] = joint_states->position[6];
     joint_positions[4] = joint_states->position[7];
     joint_positions[5] = joint_states->position[8];

}






int main(int argc, char** argv){
  
  if(initialize == 1){
    offset_prev[0]    = 0;
    offset_prev[1]    = 0;
    offset_prev[2]    = 0;
    rotation_prev.x() = 0;
    rotation_prev.y() = 0;
    rotation_prev.z() = 0;
    rotation_prev.w() = 1;
    
    
    offset[0]    = 0;
    offset[1]    = 0;
    offset[2]    = 0;
    rotation.x() = 0;
    rotation.y() = 0;
    rotation.z() = 0;
    rotation.w() = 1;
    
    
    offset_depth_sensor[0]    = 0.65;
    offset_depth_sensor[1]    = 0.0;
    offset_depth_sensor[2]    = 0.4;
    rotation_depth_sensor.x() = 0.027;
    rotation_depth_sensor.y() = 0;
    rotation_depth_sensor.z() = 0;
    rotation_depth_sensor.w() = 1;
    
  
    
    offset_null[0]    = 0;
    offset_null[1]    = 0;
    offset_null[2]    = 0;
    rotation_identity.x() = 0;
    rotation_identity.y() = 0;
    rotation_identity.z() = 0;
    rotation_identity.w() = 1;
    
    
    pivot_cloud->header.frame_id                                            = "base_link";
    pivot_cloud_without_nans->header.frame_id                               = "base_link";
    pivot_cloud_without_nans_in_base_link->header.frame_id                  = "base_link";
    pivot_cloud_without_nans_in_pivot_link->header.frame_id                 = "base_link";
    pivot_cloud_without_nans_in_pivot_link_just_translated->header.frame_id = "base_link";
    recombined_pivot_cloud->header.frame_id                                 = "base_link";
    transformed_recombined_pivot_cloud->header.frame_id                     = "base_link";
    
    pivot_cloud->points.clear();
    pivot_cloud_without_nans->points.clear(); 
    pivot_cloud_without_nans_in_base_link->points.clear(); 
    pivot_cloud_without_nans_in_pivot_link->points.clear(); 
    pivot_cloud_without_nans_in_pivot_link_just_translated->points.clear(); 
    
    recombined_pivot_cloud->points.clear(); 
    transformed_recombined_pivot_cloud->points.clear(); 

    saved_pointcloud_name << std::string(std::getenv("HOME"))+"/catkin_ws/devel/lib/moveit_cpp_for_kuka_kr6r900sixx/recombined_pivot_cloud.pcd";

//Remove the last saved recombined cloud and replace it with a lighther version 
    pcl::PointXYZRGB Pt_;
    Pt_.x = 0.0;
    Pt_.y = 0.0;
    Pt_.z = 0.0;
    Pt_.r = 0.0;
    Pt_.g = 0.0;
    Pt_.b = 0.0; 
     
    recombined_pivot_cloud->push_back(Pt_);
    pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);

    initialize = 0;
  }



  
  ros::init(argc, argv, "move_group_interface_tutorial_kuka_kr6r900sixx");
  ros::NodeHandle nh;
  ros::ServiceClient client;
  ros::AsyncSpinner spinner(1);
  spinner.start();


    
  
  
  
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface` class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  
  
  
  
  
  //Publish the recombbined pointcloud of the Pivot part
  ros::Publisher recombined_pivot_cloud_pub = nh.advertise<PointCloud_XYZRGB> ("recombined_pivot_cloud", 1); //For the point cloud

  //Subscribe to the Pivot cloud
  ros::Subscriber sub_pivot_cloud= nh.subscribe("/partial_pivot_cloud", 50, pivot_cloud_Callback); 
 

  //Subscribe to next button
  ros::Subscriber sub_next_button= nh.subscribe("/rviz_visual_tools_gui", 50, next_button_callback); 


  //Subscribe to the joint states
  ros::Subscriber sub_joint_states = nh.subscribe("/joint_states", 50, joints_state_callback); 


/*


  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame !!!!!!!!: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link !!!!!!: %s", move_group.getEndEffectorLink().c_str());
*/




  
  
  

    

  
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  ros::Rate loop_rate(30);
  while (ros::ok()) {

  //Get the robot state
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    
    //Service to get cylinder model GetModelState
    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getModelState.request.model_name = modelName;
    client.call(getModelState);
    
    pp            = getModelState.response.pose.position;
    qq            = getModelState.response.pose.orientation;
    current_Twist = getModelState.response.twist;
    
    

    offset[0]    = pp.x;
    offset[1]    = pp.y;
    offset[2]    = pp.z;
    rotation.x() = qq.x;
    rotation.y() = qq.y;
    rotation.z() = qq.z;
    rotation.w() = qq.w;
    

    
    //Transform the recombined cloud so it follows the pivot part
    
    pcl::transformPointCloud (*recombined_pivot_cloud, *transformed_recombined_pivot_cloud, offset, rotation);
    //pcl::copyPointCloud(*recombined_pivot_cloud, *transformed_recombined_pivot_cloud);
    


    recombined_pivot_cloud->header.frame_id             = "base_link";
    transformed_recombined_pivot_cloud->header.frame_id = "base_link";
    
    pcl_conversions::toPCL(ros::Time::now(), recombined_pivot_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), transformed_recombined_pivot_cloud->header.stamp);
    
    //recombined_pivot_cloud_pub.publish(recombined_pivot_cloud);
    //recombined_pivot_cloud_pub.publish(transformed_recombined_pivot_cloud);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
  
  

  // END_MVT
  ros::shutdown();
  return 0;
}
