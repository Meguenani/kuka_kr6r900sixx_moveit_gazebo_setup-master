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























































































//Code with cout to continue 
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
//
//
//#include <iomanip>
//#include <stdio.h>
//#include <dirent.h>
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <Eigen/Geometry> 
//#include <Eigen/Dense>
//#include <Eigen/Eigen>
//#include <Eigen/LU> 
//#include <Eigen/Core>
//#include <atomic>
//#include <chrono>
//
//
//#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include "/usr/local/include/pcl-1.8/pcl/io/io.h"
//#include "/usr/local/include/pcl-1.8/pcl/io/pcd_io.h"
//#include "/usr/local/include/pcl-1.8/pcl/point_cloud.h"
//#include "/usr/local/include/pcl-1.8/pcl/console/parse.h"
//#include "/usr/local/include/pcl-1.8/pcl/common/transforms.h"
//#include "/usr/local/include/pcl-1.8/pcl/common/common.h"
//#include "/usr/local/include/pcl-1.8/pcl/filters/crop_box.h"
//
//#include "/usr/local/include/pcl-1.8/pcl/PCLPointCloud2.h"
//
//#include "std_msgs/Float32MultiArray.h"
//#include <geometry_msgs/Point.h>
//#include <cmath>   
//#include "gazebo_msgs/ModelStates.h"
//#include "gazebo_msgs/GetModelState.h"
//#include <tf/transform_broadcaster.h>
//#include <sensor_msgs/PointCloud2.h>
//
//
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/io.h>
//#include <pcl/filters/passthrough.h>
//#include <visualization_msgs/Marker.h>
//#include <tf/transform_listener.h>
//
////#include </usr/local/include/pcl-1.8/pcl/visualization/cloud_viewer.h>
//
//
//using namespace std;
//
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_XYZ;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_XYZRGB;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud (new PointCloud_XYZRGB);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans (new PointCloud_XYZRGB);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans_in_base_link (new PointCloud_XYZRGB);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans_in_pivot_link (new PointCloud_XYZRGB);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pivot_cloud_without_nans_in_pivot_link_just_translated (new PointCloud_XYZRGB);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr recombined_pivot_cloud (new PointCloud_XYZRGB);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_recombined_pivot_cloud (new PointCloud_XYZRGB);
//
//
////To transfor the partial pivot pointcloud 
//Eigen::Vector3f offset; 
//Eigen::Quaternionf rotation;
//
//Eigen::Vector3f offset_inverse; 
//
//Eigen::Vector3f offset_null; 
//Eigen::Quaternionf rotation_identity;
//
//Eigen::Vector3f offset_pose1; 
//Eigen::Quaternionf rotation_pose1;
//
//
//Eigen::Vector3f offset_prev; 
//Eigen::Quaternionf rotation_prev;
//
//Eigen::Vector3f offset_depth_sensor; 
//Eigen::Quaternionf rotation_depth_sensor;
//
//
//Eigen::Vector3f offset_btw_two_poses; 
//Eigen::Quaternionf rotation_btw_two_poses;
//
//
//int initialize = 1;
//
//
//void pivot_cloud_Callback(const PointCloud_XYZRGB::ConstPtr& pivot_cloud_msg){
//
//    pcl::copyPointCloud(*pivot_cloud_msg, *pivot_cloud);
//
//     
//    //remove NaN points from the cloud
//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*pivot_cloud,*pivot_cloud_without_nans, indices);
//
//    //std::cout << "pivot_cloud_without_nans.size : " << pivot_cloud_without_nans->size() <<std::endl;
//
//}
//
//
//
//
//
//
//int main(int argc, char** argv)
//{
//  
//  if(initialize == 1){
//    offset_prev[0]    = 0;
//    offset_prev[1]    = 0;
//    offset_prev[2]    = 0;
//    rotation_prev.x() = 0;
//    rotation_prev.y() = 0;
//    rotation_prev.z() = 0;
//    rotation_prev.w() = 1;
//    
//    
//    offset[0]    = 0;
//    offset[1]    = 0;
//    offset[2]    = 0;
//    rotation.x() = 0;
//    rotation.y() = 0;
//    rotation.z() = 0;
//    rotation.w() = 1;
//    
//    
//    offset_depth_sensor[0]    = 0.65;
//    offset_depth_sensor[1]    = 0.0;
//    offset_depth_sensor[2]    = 0.4;
//    rotation_depth_sensor.x() = 0.027;
//    rotation_depth_sensor.y() = 0;
//    rotation_depth_sensor.z() = 0;
//    rotation_depth_sensor.w() = 1;
//    
//  
//    
//    offset_null[0]    = 0;
//    offset_null[1]    = 0;
//    offset_null[2]    = 0;
//    rotation_identity.x() = 0;
//    rotation_identity.y() = 0;
//    rotation_identity.z() = 0;
//    rotation_identity.w() = 1;
//    
//    
//    pivot_cloud->header.frame_id                                            = "base_link";
//    pivot_cloud_without_nans->header.frame_id                               = "base_link";
//    pivot_cloud_without_nans_in_base_link->header.frame_id                  = "base_link";
//    pivot_cloud_without_nans_in_pivot_link->header.frame_id                 = "base_link";
//    pivot_cloud_without_nans_in_pivot_link_just_translated->header.frame_id = "base_link";
//    recombined_pivot_cloud->header.frame_id                                 = "base_link";
//    transformed_recombined_pivot_cloud->header.frame_id                     = "base_link";
//    
//    pivot_cloud->points.clear();
//    pivot_cloud_without_nans->points.clear(); 
//    pivot_cloud_without_nans_in_base_link->points.clear(); 
//    pivot_cloud_without_nans_in_pivot_link->points.clear(); 
//    pivot_cloud_without_nans_in_pivot_link_just_translated->points.clear(); 
//    
//    recombined_pivot_cloud->points.clear(); 
//    transformed_recombined_pivot_cloud->points.clear(); 
//
//
//
//    initialize = 0;
//  }
//  
//  ros::init(argc, argv, "move_group_interface_tutorial_kuka_kr6r900sixx");
//  ros::NodeHandle nh;
//  ros::ServiceClient client;
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
//
//  std::string modelName = (std::string)"cylinder";
//  gazebo_msgs::GetModelState getModelState;
//    
//  geometry_msgs::Point pp ;
//  geometry_msgs::Quaternion qq ;
//  geometry_msgs::Twist current_Twist ;
//  
//  
//  
//  
//  //Publish the recombbined pointcloud of the Pivot part
//  ros::Publisher recombined_pivot_cloud_pub = nh.advertise<PointCloud_XYZRGB> ("recombined_pivot_cloud", 1); //For the point cloud
//
//
//  //Subscribe to the Pivot cloud
//  ros::Subscriber sub_pivot_cloud= nh.subscribe("/partial_pivot_cloud", 50, pivot_cloud_Callback); //Waits for "GO!"
//
// 
//  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
//  static const std::string PLANNING_GROUP = "manipulator";
//
//  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
//  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//
//  // We will use the :planning_scene_interface:`PlanningSceneInterface` class to add and remove collision objects in our "virtual world" scene
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//
//  // Raw pointers are frequently used to refer to the planning group for improved performance.
//  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//
//
//
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
//  int i;
//  cout << "Please enter 1 to start the demo ";
//  cin >> i;
//  
//
//  
//  
///*
////Pre_pick_pose (By setting joint values directly)
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
//  // Pre_pick pose
//  joint_group_positions[0] = -0.014958316846041875;  // radians
//  joint_group_positions[1] = -1.0537766915280988 ;
//  joint_group_positions[2] =  1.439558418250959   ;
//  joint_group_positions[3] =  0.046518732721668954;
//  joint_group_positions[4] =  1.186862317693988   ;
//  joint_group_positions[5] = -0.04605105974771462;
//    
//  
//  move_group.setJointValueTarget(joint_group_positions);
//
//  // Now, we call the planner to compute the plan then move.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  
//  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 1 %s", success ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group.move();
//*/
//  
//
//
//
//
//
////Pre_pick_pose (With state name)
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
//  
//  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_pose");
//
//  move_group.setJointValueTarget(*current_state);
//
//  // Now, we call the planner to compute the plan then move.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  
//  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 1 %s", success ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group.move();
//
//
//  
//
//  
//  
//  
//  cout << "Please enter 1 to move to joint goal 2 ";
//  cin >> i;
//
//  
//
//  
//  
//
//
//
///*
////Pick_pose with joint values
//  // Since we set the start state we have to clear it before planning other paths
//  move_group.setStartStateToCurrentState();
//  
//  //current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "Pick_pose");
//  
//  // Pick_pose
//   joint_group_positions[0] = -0.010898417838824948; 
//   joint_group_positions[1] = -1.0041559788153602  ; 
//   joint_group_positions[2] =  1.5184823093622155   ; 
//   joint_group_positions[3] =  0.04941393852790732  ; 
//   joint_group_positions[4] =  1.058450119570816    ; 
//   joint_group_positions[5] = -0.04890616363120337 ;
//
//  move_group.setJointValueTarget(joint_group_positions);
//
//  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 2 %s", success ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group.move();
//*/
//
//
////Pick_pose  (With name) 
//  // Since we set the start state we have to clear it before planning other paths
//  move_group.setStartStateToCurrentState();
//
//  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pick_pose");
//
//  move_group.setJointValueTarget(*current_state);
//
//  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 2 %s", success ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group.move();
//  
//  
//  
//  
//  cout << "Please enter 1 operate the gripper ";
//  cin >> i;
//  
//  
//  
//  
//  
//  
//  
////Operate the gripper
//  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
//  static const std::string PLANNING_GROUP2 = "gripper";
//
//  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
//  moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
//  
//  // Raw pointers are frequently used to refer to the planning group for improved performance.
//  const robot_state::JointModelGroup* joint_model_group2 = move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
//  
//  // We can print the name of the reference frame for this robot.
//  ROS_INFO_NAMED("gripper mvt : ", "gripper frame !!!!!!!!: %s", move_group2.getPlanningFrame().c_str());
//
//  // We can also print the name of the end-effector link for this group.
//  ROS_INFO_NAMED("gripper mvt : ", "gripper link !!!!!!: %s", move_group2.getEndEffectorLink().c_str());
//  
//  
///*
////Open the gripper (By setting joint values directly)
// // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions2;
//  current_state->copyJointGroupPositions(joint_model_group2, joint_group_positions2);
//
//  // Now, let's modify  the joints, plan to the new joint space goal and move towards it
//  joint_group_positions2[0] = 1000;  // radians
//  joint_group_positions2[1] = 1000  ;
//  joint_group_positions2[2] = 1000  ;
//
//    
//  move_group2.setJointValueTarget(joint_group_positions2);
//
//  // Now, we call the planner to compute the plan then move.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
//  
//  bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("gripper mvt : ", "Open gripper %s", success ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group2.move();
//*/
//  
//  
//  
////Open the gripper (With state name)
// // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions2;
//
//  
//  current_state2->setToDefaultValues(current_state2->getJointModelGroup("gripper"), "open_gripper");
//
//  move_group2.setJointValueTarget(*current_state2);
//  
//
//  // Now, we call the planner to compute the plan then move.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
//  
//  bool success2 = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("gripper mvt : ", "Open gripper %s", success ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group2.move();
//  
//  
//  
//  
//    
//  
//  
//  
//  cout << "Please enter 1 to move to joint goal 3 ";
//  cin >> i;
//  
//  
//
//
//  
//  
//  //POSE FOR PARTIAL CLOUD ACQUISITION N°1
//  move_group.setStartStateToCurrentState();
//
//  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose1");
//
//  move_group.setJointValueTarget(*current_state);
//  
//  bool success_pose1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  
//  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 2 %s", success_pose1 ? "" : "FAILED");
//
//  //Move to the planned position
//  move_group.move();
//  
//  
//  
//  
//  cout << "Please enter 1 To finish (here we capture our first cloud) !!! ";
//  cin >> i;
//  
//  
//  
//  
//  
//  
//  
//  if(success_pose1 == true){
//    //Service to get cylinder model GetModelState
//    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
//    getModelState.request.model_name = modelName;
//    client.call(getModelState);
//    
//    pp            = getModelState.response.pose.position;
//    qq            = getModelState.response.pose.orientation;
//    current_Twist = getModelState.response.twist;
//
//
//    offset_pose1[0]    = pp.x;
//    offset_pose1[1]    = pp.y;
//    offset_pose1[2]    = pp.z;
//    rotation_pose1.x() = qq.x;
//    rotation_pose1.y() = qq.y;
//    rotation_pose1.z() = qq.z;
//    rotation_pose1.w() = qq.w;
//
//    
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset_pose1, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation_pose1.inverse());
//
//
//    
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 1  !!!!!!!!!! "<<std::endl;
//  }
//  
//  
//  
//
//  
//  
//  
//  
//  ros::Rate loop_rate(30);
//  while (ros::ok()) {
//    
//   
//
//    
//    //Service to get cylinder model GetModelState
//    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
//    getModelState.request.model_name = modelName;
//    client.call(getModelState);
//    
//    pp            = getModelState.response.pose.position;
//    qq            = getModelState.response.pose.orientation;
//    current_Twist = getModelState.response.twist;
//    
//    
//    offset_prev[0]    = offset[0];
//    offset_prev[1]    = offset[1];
//    offset_prev[2]    = offset[2];
//    rotation_prev.x() = rotation.x();
//    rotation_prev.y() = rotation.y();
//    rotation_prev.z() = rotation.z();
//    rotation_prev.w() = rotation.w();
//    
//
//    offset[0]    = pp.x;
//    offset[1]    = pp.y;
//    offset[2]    = pp.z;
//    rotation.x() = qq.x;
//    rotation.y() = qq.y;
//    rotation.z() = qq.z;
//    rotation.w() = qq.w;
//    
//    
//    offset_btw_two_poses   = offset - offset_prev;
//    rotation_btw_two_poses = rotation_prev.inverse()*rotation;
//    
//    
//    
//    //Transform the recombined cloud so it follows the pivot part
//    
//    pcl::transformPointCloud (*recombined_pivot_cloud, *transformed_recombined_pivot_cloud, offset, rotation);
//    //pcl::copyPointCloud(*recombined_pivot_cloud, *transformed_recombined_pivot_cloud);
//    
//    //pcl::transformPointCloud (*recombined_pivot_cloud, *transformed_recombined_pivot_cloud, offset_btw_two_poses, rotation_btw_two_poses);
//
//    
//
//    recombined_pivot_cloud->header.frame_id             = "base_link";
//    transformed_recombined_pivot_cloud->header.frame_id = "base_link";
//    
//    pcl_conversions::toPCL(ros::Time::now(), recombined_pivot_cloud->header.stamp);
//    pcl_conversions::toPCL(ros::Time::now(), transformed_recombined_pivot_cloud->header.stamp);
//    
//    //recombined_pivot_cloud_pub.publish(recombined_pivot_cloud);
//    recombined_pivot_cloud_pub.publish(transformed_recombined_pivot_cloud);
//
//    ros::spinOnce();
//    loop_rate.sleep();
//  }
//  
//  
//  
//  
//
//  // END_MVT
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
#include <iterator>
#include "compute_rms.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

//#include </usr/local/include/pcl-1.8/pcl/visualization/cloud_viewer.h>


using namespace std;
namespace rvt = rviz_visual_tools;

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

Eigen::Vector3f offset_pose1; 
Eigen::Quaternionf rotation_pose1;


Eigen::Vector3f offset_prev; 
Eigen::Quaternionf rotation_prev;

Eigen::Vector3f offset_depth_sensor; 
Eigen::Quaternionf rotation_depth_sensor;


Eigen::Vector3f offset_btw_two_poses; 
Eigen::Quaternionf rotation_btw_two_poses;


int initialize = 1;

bool success_pre_pick_pose           = false;
bool success_pre_pick_pose_bis       = false;
bool success_pick_pose               = false;
bool success_pick_pose_bis           = false;
bool success_gripper_open            = false;
bool success_gripper_open_bis        = false;
bool success_gripper_close           = false;
bool success_cloud_acq_pose1_face_1  = false;
bool success_cloud_acq_pose2_face_1  = false;
bool success_cloud_acq_pose3_face_1  = false;
bool success_cloud_acq_pose4_face_1  = false;
bool success_cloud_acq_pose5_face_1  = false;
bool success_cloud_acq_pose6_face_1  = false;
bool success_cloud_acq_pose7_face_1  = false;
bool success_cloud_acq_pose8_face_1  = false;
bool success_cloud_acq_pose9_face_1  = false;
bool success_cloud_acq_pose10_face_1 = false;

bool success_pre_pick_2_pose_0       = false;
bool success_pre_pick_2_pose_1       = false;
bool success_pre_pick_2_pose_2       = false;
bool success_pre_pick_2_pose_3       = false;
bool success_pick_2_pose             = false;



bool success_cloud_acq_pose1_face_2  = false;
bool success_cloud_acq_pose2_face_2  = false;
bool success_cloud_acq_pose3_face_2  = false;
bool success_cloud_acq_pose4_face_2  = false;
bool success_cloud_acq_pose5_face_2  = false;
bool success_cloud_acq_pose6_face_2  = false;
bool success_cloud_acq_pose7_face_2  = false;
bool success_cloud_acq_pose8_face_2  = false;
bool success_cloud_acq_pose9_face_2  = false;
bool success_cloud_acq_pose10_face_2 = false;



std::map<std::string, double> joint_group_positions_map;
std::map<std::string, double>::iterator iterator0;



std::vector<double> pick_pose_face_1(6);
std::vector<double> cloud_acq_pose1_face_1(6);
std::vector<double> cloud_acq_pose2_face_1(6);
std::vector<double> cloud_acq_pose3_face_1(6);
std::vector<double> cloud_acq_pose4_face_1(6);
std::vector<double> cloud_acq_pose5_face_1(6);
std::vector<double> cloud_acq_pose6_face_1(6);
std::vector<double> cloud_acq_pose7_face_1(6);
std::vector<double> cloud_acq_pose8_face_1(6);
std::vector<double> cloud_acq_pose9_face_1(6);
std::vector<double> cloud_acq_pose10_face_1(6);   


std::vector<double> pre_pick_2_pose_0(6);
std::vector<double> pre_pick_2_pose_1(6);
std::vector<double> pre_pick_2_pose_2(6);
std::vector<double> pre_pick_2_pose_3(6);
std::vector<double> pick_pose_face_2(6);
std::vector<double> cloud_acq_pose1_face_2(6);
std::vector<double> cloud_acq_pose2_face_2(6);
std::vector<double> cloud_acq_pose3_face_2(6);
std::vector<double> cloud_acq_pose4_face_2(6);
std::vector<double> cloud_acq_pose5_face_2(6);
std::vector<double> cloud_acq_pose6_face_2(6);
std::vector<double> cloud_acq_pose7_face_2(6);
std::vector<double> cloud_acq_pose8_face_2(6);
std::vector<double> cloud_acq_pose9_face_2(6);
std::vector<double> cloud_acq_pose10_face_2(6); 


std::vector<double> manipulator_joint_positions(6);

double joint_posi_rms_error    = 0.0005;

double joint_pick_pose_face_1_rms = 10.0;
double joint_posi_1_face_1_rms = 10.0;
double joint_posi_2_face_1_rms = 10.0;
double joint_posi_3_face_1_rms = 10.0;
double joint_posi_4_face_1_rms = 10.0;
double joint_posi_5_face_1_rms = 10.0;
double joint_posi_6_face_1_rms = 10.0;
double joint_posi_7_face_1_rms = 10.0;
double joint_posi_8_face_1_rms = 10.0;
double joint_posi_9_face_1_rms = 10.0;
double joint_posi_10_face_1_rms = 10.0;



double joint_pre_pick_2_pose_0_rms = 10.0;
double joint_pre_pick_2_pose_1_rms = 10.0;
double joint_pre_pick_2_pose_2_rms = 10.0;
double joint_pre_pick_2_pose_3_rms = 10.0;

double joint_pick_pose_face_2_rms = 10.0;
double joint_posi_1_face_2_rms = 10.0;
double joint_posi_2_face_2_rms = 10.0;
double joint_posi_3_face_2_rms = 10.0;
double joint_posi_4_face_2_rms = 10.0;
double joint_posi_5_face_2_rms = 10.0;
double joint_posi_6_face_2_rms = 10.0;
double joint_posi_7_face_2_rms = 10.0;
double joint_posi_8_face_2_rms = 10.0;
double joint_posi_9_face_2_rms = 10.0;
double joint_posi_10_face_2_rms = 10.0;



int go_to_pose_1_face_1  = 1;
int go_to_pose_2_face_1  = 0;
int go_to_pose_3_face_1  = 0;
int go_to_pose_4_face_1  = 0;
int go_to_pose_5_face_1  = 0;
int go_to_pose_6_face_1  = 0;
int go_to_pose_7_face_1  = 0;
int go_to_pose_8_face_1  = 0;
int go_to_pose_9_face_1  = 0;
int go_to_pose_10_face_1 = 0;
                           
int go_to_pose_1_face_2  = 0;
int go_to_pose_2_face_2  = 0;
int go_to_pose_3_face_2  = 0;
int go_to_pose_4_face_2  = 0;
int go_to_pose_5_face_2  = 0;
int go_to_pose_6_face_2  = 0;
int go_to_pose_7_face_2  = 0;
int go_to_pose_8_face_2  = 0;
int go_to_pose_9_face_2  = 0;
int go_to_pose_10_face_2 = 0;

int pick_1               = 1;
int place_1              = 0;
int pick_2               = 0;
int place_2              = 0;


int pose_1_face_1_reached  = 0;
int pose_2_face_1_reached  = 0;
int pose_3_face_1_reached  = 0;
int pose_4_face_1_reached  = 0;
int pose_5_face_1_reached  = 0;
int pose_6_face_1_reached  = 0;
int pose_7_face_1_reached  = 0;
int pose_8_face_1_reached  = 0;
int pose_9_face_1_reached  = 0;
int pose_10_face_1_reached = 0;
                           
int pose_1_face_2_reached  = 0;
int pose_2_face_2_reached  = 0;
int pose_3_face_2_reached  = 0;
int pose_4_face_2_reached  = 0;
int pose_5_face_2_reached  = 0;
int pose_6_face_2_reached  = 0;
int pose_7_face_2_reached  = 0;
int pose_8_face_2_reached  = 0;
int pose_9_face_2_reached  = 0;
int pose_10_face_2_reached = 0;


double ros_time_now = 0.0;

double time_open_gripper           = 0.0;
double time_close_gripper          = 0.0;

double time_open_gripper_bis       = 0.0;
double time_close_gripper_bis      = 0.0;
    
double time_pose_1_face_1_reached  = 0.0;
double time_pose_2_face_1_reached  = 0.0;
double time_pose_3_face_1_reached  = 0.0;
double time_pose_4_face_1_reached  = 0.0;
double time_pose_5_face_1_reached  = 0.0;
double time_pose_6_face_1_reached  = 0.0;
double time_pose_7_face_1_reached  = 0.0;
double time_pose_8_face_1_reached  = 0.0;
double time_pose_9_face_1_reached  = 0.0;
double time_pose_10_face_1_reached = 0.0;

double time_pose_1_face_2_reached  = 0.0;
double time_pose_2_face_2_reached  = 0.0;
double time_pose_3_face_2_reached  = 0.0;
double time_pose_4_face_2_reached  = 0.0;
double time_pose_5_face_2_reached  = 0.0;
double time_pose_6_face_2_reached  = 0.0;
double time_pose_7_face_2_reached  = 0.0;
double time_pose_8_face_2_reached  = 0.0;
double time_pose_9_face_2_reached  = 0.0;
double time_pose_10_face_2_reached = 0.0;

double time_pre_pick_pose_bis_reached = 0.0;

std::ostringstream saved_pointcloud_name;



moveit_msgs::MoveItErrorCodes success_pre_pick_2_pose_0_mv;
moveit_msgs::MoveItErrorCodes success_pre_pick_2_pose_1_mv;
moveit_msgs::MoveItErrorCodes success_pre_pick_2_pose_2_mv;
moveit_msgs::MoveItErrorCodes success_pre_pick_2_pose_3_mv;
moveit_msgs::MoveItErrorCodes success_pick_2_pose_mv      ;


  

void pivot_cloud_Callback(const PointCloud_XYZRGB::ConstPtr& pivot_cloud_msg){

    pcl::copyPointCloud(*pivot_cloud_msg, *pivot_cloud);

     
    //remove NaN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pivot_cloud,*pivot_cloud_without_nans, indices);

    //std::cout << "pivot_cloud_without_nans.size : " << pivot_cloud_without_nans->size() <<std::endl;

}






int main(int argc, char** argv)
{
  
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

  std::string modelName = (std::string)"cylinder";
  gazebo_msgs::GetModelState getModelState;
    
  geometry_msgs::Point pp ;
  geometry_msgs::Quaternion qq ;
  geometry_msgs::Twist current_Twist ;
  
  
  
  rvt::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rvt::RvizVisualTools("base_link", "/rviz_visual_tools"));
  
  // Create pose
  Eigen::Isometry3d arrow_pose;
  arrow_pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  arrow_pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

  /*
  Eigen::Isometry3d cube_pose;
  cube_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  cube_pose.translation() = Eigen::Vector3d(0.0, 0.5, 0.5 ); // translate x,y,z
  */


  
  
  
  //Publish the recombbined pointcloud of the Pivot part
  ros::Publisher recombined_pivot_cloud_pub = nh.advertise<PointCloud_XYZRGB> ("recombined_pivot_cloud", 1); //For the point cloud


  //Subscribe to the Pivot cloud
  ros::Subscriber sub_pivot_cloud= nh.subscribe("/partial_pivot_cloud", 50, pivot_cloud_Callback); //Waits for "GO!"

 
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface` class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame !!!!!!!!: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link !!!!!!: %s", move_group.getEndEffectorLink().c_str());


  //Declaring the gripper group 
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
  static const std::string PLANNING_GROUP2 = "gripper";

  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
   
  
  
  
  
  


  
  
  //Get the poses joint values
  //Face1  
  joint_group_positions_map = move_group.getNamedTargetValues("pick_pose");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      pick_pose_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose1_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose1_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  

  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose2_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose2_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose3_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose3_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose4_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose4_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose5_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose5_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose6_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose6_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose7_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose7_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose8_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose8_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }

  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose9_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose9_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
/*
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose10_face_1");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose10_face_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
*/
  
  





  //Face2
  

  
  
  joint_group_positions_map = move_group.getNamedTargetValues("pre_pick_2_pose_0");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      pre_pick_2_pose_0[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  joint_group_positions_map = move_group.getNamedTargetValues("pre_pick_2_pose_1");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      pre_pick_2_pose_1[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  joint_group_positions_map = move_group.getNamedTargetValues("pre_pick_2_pose_2");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      pre_pick_2_pose_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  joint_group_positions_map = move_group.getNamedTargetValues("pre_pick_2_pose_3");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      pre_pick_2_pose_3[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  joint_group_positions_map = move_group.getNamedTargetValues("pick_2_pose");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      pick_pose_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose1_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose1_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  

  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose2_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose2_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose3_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose3_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose4_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose4_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose5_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose5_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
/*
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose6_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose6_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose7_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose7_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose8_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose8_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }

  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose9_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose9_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
  
  
  joint_group_positions_map = move_group.getNamedTargetValues("cloud_acq_pose10_face_2");
  iterator0 = joint_group_positions_map.begin();
  
  int it = 0;
  while (iterator0 != joint_group_positions_map.end()){
      cloud_acq_pose10_face_2[it] = iterator0->second;
      iterator0++;
      it++;
  }
*/
  
  
  
  
  
  
  
  
  

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  int i;
  cout << "Please enter 1 to start the demo ";
  cin >> i;
  

  
  
/*
//Pre_pick_pose (By setting joint values directly)
  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create a pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
     
  // Pre_pick pose
  joint_group_positions[0] = -0.014958316846041875;  // radians
  joint_group_positions[1] = -1.0537766915280988 ;
  joint_group_positions[2] =  1.439558418250959   ;
  joint_group_positions[3] =  0.046518732721668954;
  joint_group_positions[4] =  1.186862317693988   ;
  joint_group_positions[5] = -0.04605105974771462;
    
  
  move_group.setJointValueTarget(joint_group_positions);

  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 1 %s", success ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
*/
  









/*
//Pick_pose with joint values
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();
  
  //current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "Pick_pose");
  
  // Pick_pose
   joint_group_positions[0] = -0.010898417838824948; 
   joint_group_positions[1] = -1.0041559788153602  ; 
   joint_group_positions[2] =  1.5184823093622155   ; 
   joint_group_positions[3] =  0.04941393852790732  ; 
   joint_group_positions[4] =  1.058450119570816    ; 
   joint_group_positions[5] = -0.04890616363120337 ;

  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pick and place : ", "Plan towards joint goal 2 %s", success ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
*/



/*
//Open the gripper (By setting joint values directly)
 // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions2;
  current_state->copyJointGroupPositions(joint_model_group2, joint_group_positions2);

  // Now, let's modify  the joints, plan to the new joint space goal and move towards it
  joint_group_positions2[0] = 1000;  // radians
  joint_group_positions2[1] = 1000  ;
  joint_group_positions2[2] = 1000  ;

    
  move_group2.setJointValueTarget(joint_group_positions2);

  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  
  bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("gripper mvt : ", "Open gripper %s", success ? "" : "FAILED");

  //Move to the planned position
  move_group2.move();
*/
  


  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    
    /*
    // Publish arrow vector of pose
    visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
    
    //visual_tools_->publishCuboid(cube_point1, cube_point2, rviz_visual_tools::BLUE);
    
    visual_tools_->publishWireframeCuboid(cube_pose, 0.6, 0.3, 0.5, rviz_visual_tools::BLUE, "Wireframe_Cuboid", 0);
    //double depth, double width, double height     
    
    // Don't forget to trigger the publisher!
    visual_tools_->trigger();
    */
    
    
    ros_time_now = ros::Time::now().toSec();

    //GET PIVOT POSE IN BASE_LINK FRAME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
    
    
    
    
    
    
    //Get the joint values
    std::vector<double> current_joint_group_positions;    
    current_joint_group_positions = move_group.getCurrentJointValues();
    
    
    

    
    
    joint_pick_pose_face_1_rms = compute_rms(pick_pose_face_1,       current_joint_group_positions);
    joint_posi_1_face_1_rms    = compute_rms(cloud_acq_pose1_face_1, current_joint_group_positions);

    joint_posi_2_face_1_rms = compute_rms(cloud_acq_pose2_face_1, current_joint_group_positions);
    joint_posi_3_face_1_rms = compute_rms(cloud_acq_pose3_face_1, current_joint_group_positions);
    joint_posi_4_face_1_rms = compute_rms(cloud_acq_pose4_face_1, current_joint_group_positions);
    joint_posi_5_face_1_rms = compute_rms(cloud_acq_pose5_face_1, current_joint_group_positions);
    joint_posi_6_face_1_rms = compute_rms(cloud_acq_pose6_face_1, current_joint_group_positions);
    joint_posi_7_face_1_rms = compute_rms(cloud_acq_pose7_face_1, current_joint_group_positions);
    joint_posi_8_face_1_rms = compute_rms(cloud_acq_pose8_face_1, current_joint_group_positions);
    joint_posi_9_face_1_rms = compute_rms(cloud_acq_pose9_face_1, current_joint_group_positions);
/*
    joint_posi_10_face_1_rms = compute_rms(cloud_acq_pose10_face_1, current_joint_group_positions);
*/
    
    joint_pre_pick_2_pose_0_rms= compute_rms(pre_pick_2_pose_0,      current_joint_group_positions);
    joint_pre_pick_2_pose_1_rms= compute_rms(pre_pick_2_pose_1,      current_joint_group_positions);
    joint_pre_pick_2_pose_2_rms= compute_rms(pre_pick_2_pose_2,      current_joint_group_positions);
    joint_pre_pick_2_pose_3_rms= compute_rms(pre_pick_2_pose_3,      current_joint_group_positions);
    joint_pick_pose_face_2_rms = compute_rms(pick_pose_face_2,       current_joint_group_positions);
    
    joint_posi_1_face_2_rms    = compute_rms(cloud_acq_pose1_face_2, current_joint_group_positions);
    joint_posi_2_face_2_rms    = compute_rms(cloud_acq_pose2_face_2, current_joint_group_positions);
    joint_posi_3_face_2_rms    = compute_rms(cloud_acq_pose3_face_2, current_joint_group_positions);
    joint_posi_4_face_2_rms    = compute_rms(cloud_acq_pose4_face_2, current_joint_group_positions);
    joint_posi_5_face_2_rms    = compute_rms(cloud_acq_pose5_face_2, current_joint_group_positions);
/*
    joint_posi_6_face_2_rms = compute_rms(cloud_acq_pose6_face_2, current_joint_group_positions);
    joint_posi_7_face_2_rms = compute_rms(cloud_acq_pose7_face_2, current_joint_group_positions);
    joint_posi_8_face_2_rms = compute_rms(cloud_acq_pose8_face_2, current_joint_group_positions);
    joint_posi_9_face_2_rms = compute_rms(cloud_acq_pose9_face_2, current_joint_group_positions);
    joint_posi_10_face_2_rms = compute_rms(cloud_acq_pose10_face_2, current_joint_group_positions);
*/

    



//**************************************************************************************************************************************//
    if(joint_posi_1_face_1_rms < joint_posi_rms_error && pose_1_face_1_reached == 0 && go_to_pose_1_face_1 == 1){
      pose_1_face_1_reached = 1;
      time_pose_1_face_1_reached = ros_time_now;
    }  
    
    
    if(pose_1_face_1_reached == 1 && (ros_time_now-time_pose_1_face_1_reached) > 0.5 && go_to_pose_2_face_1 == 0 && go_to_pose_1_face_1 == 1){
       pose_1_face_1_reached = 1;
       go_to_pose_2_face_1   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
    *recombined_pivot_cloud = *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 1 FACE 1 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_2_face_1_rms < joint_posi_rms_error && pose_2_face_1_reached == 0 && go_to_pose_1_face_1 == 1){
      pose_2_face_1_reached = 1;
      time_pose_2_face_1_reached = ros_time_now;
    }  
    
    
    if(pose_2_face_1_reached == 1 && (ros_time_now-time_pose_1_face_1_reached) > 0.5 && go_to_pose_3_face_1 == 0 && go_to_pose_1_face_1 == 1){
       pose_2_face_1_reached = 1;
       go_to_pose_3_face_1   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 2 FACE 1 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_3_face_1_rms < joint_posi_rms_error && pose_3_face_1_reached == 0 && go_to_pose_1_face_1 == 1){
      pose_3_face_1_reached = 1;
      time_pose_3_face_1_reached = ros_time_now;
    }  
    
    
    if(pose_3_face_1_reached == 1 && (ros_time_now-time_pose_3_face_1_reached) > 0.5 && go_to_pose_4_face_1 == 0 && go_to_pose_1_face_1 == 1){
       pose_3_face_1_reached = 1;
       go_to_pose_4_face_1   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 3 FACE 1 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_4_face_1_rms < joint_posi_rms_error && pose_4_face_1_reached == 0 && go_to_pose_1_face_1 == 1){
      pose_4_face_1_reached = 1;
      time_pose_4_face_1_reached = ros_time_now;
    }  
    
    
    if(pose_4_face_1_reached == 1 && (ros_time_now-time_pose_4_face_1_reached) > 0.5 && go_to_pose_5_face_1 == 0 && go_to_pose_1_face_1 == 1){
       pose_4_face_1_reached = 1;
       go_to_pose_5_face_1   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 4 FACE 1 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_5_face_1_rms < joint_posi_rms_error && pose_5_face_1_reached == 0 && go_to_pose_1_face_1 == 1){
      pose_5_face_1_reached = 1;
      time_pose_5_face_1_reached = ros_time_now;
    }  
    
    
    if(pose_5_face_1_reached == 1 && (ros_time_now-time_pose_5_face_1_reached) > 0.5 && go_to_pose_1_face_2 == 0 && go_to_pose_1_face_1 == 1){
       pose_5_face_1_reached = 1;
       go_to_pose_1_face_2   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 5 FACE 1 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


////**************************************************************************************************************************************//
//    if(joint_posi_6_face_1_rms < joint_posi_rms_error && pose_6_face_1_reached == 0){
//      pose_6_face_1_reached = 1;
//      time_pose_6_face_1_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_6_face_1_reached == 1 && (ros_time_now-time_pose_6_face_1_reached) > 0.5 && go_to_pose_7_face_1 == 0){
//       pose_6_face_1_reached = 1;
//       go_to_pose_7_face_1   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 6 FACE 1 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//
//
//
////**************************************************************************************************************************************//
//    if(joint_posi_7_face_1_rms < joint_posi_rms_error && pose_7_face_1_reached == 0){
//      pose_7_face_1_reached = 1;
//      time_pose_7_face_1_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_7_face_1_reached == 1 && (ros_time_now-time_pose_7_face_1_reached) > 0.5 && go_to_pose_8_face_1 == 0){
//       pose_7_face_1_reached = 1;
//       go_to_pose_8_face_1   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 7 FACE 1 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//
//
//
//
////**************************************************************************************************************************************//
//    if(joint_posi_8_face_1_rms < joint_posi_rms_error && pose_8_face_1_reached == 0){
//      pose_8_face_1_reached = 1;
//      time_pose_8_face_1_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_8_face_1_reached == 1 && (ros_time_now-time_pose_8_face_1_reached) > 0.5 && go_to_pose_9_face_1 == 0){
//       pose_8_face_1_reached = 1;
//       go_to_pose_9_face_1   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 8 FACE 1 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//
//
//
//
////**************************************************************************************************************************************//
//    if(joint_posi_9_face_1_rms < joint_posi_rms_error && pose_9_face_1_reached == 0){
//      pose_9_face_1_reached = 1;
//      time_pose_9_face_1_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_9_face_1_reached == 1 && (ros_time_now-time_pose_9_face_1_reached) > 0.5 && go_to_pose_10_face_1 == 0){
//       pose_9_face_1_reached  = 1;
//       go_to_pose_10_face_1   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 9 FACE 1 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//

    
    
    
    
    
    
    
    /*
    if(joint_posi_10_face_1_rms < joint_posi_rms_error){
      pose_10_face_1_reached = 1;
    }
    */
    
    
    
    
    

//**************************************************************************************************************************************//
    if(joint_posi_1_face_2_rms < joint_posi_rms_error && pose_1_face_2_reached == 0 && go_to_pose_2_face_1 == 1){
      pose_1_face_2_reached = 1;
      time_pose_1_face_2_reached = ros_time_now;
    }  
    
    
    if(pose_1_face_2_reached == 1 && (ros_time_now-time_pose_1_face_2_reached) > 0.5 && go_to_pose_2_face_2 == 0 && go_to_pose_2_face_1 == 1){
       pose_1_face_2_reached = 1;
       go_to_pose_2_face_2   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 1 FACE 2 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_2_face_2_rms < joint_posi_rms_error && pose_2_face_2_reached == 0 && go_to_pose_2_face_1 == 1){
      pose_2_face_2_reached = 1;
      time_pose_2_face_2_reached = ros_time_now;
    }  
    
    
    if(pose_2_face_2_reached == 1 && (ros_time_now-time_pose_1_face_2_reached) > 0.5 && go_to_pose_3_face_2 == 0 && go_to_pose_2_face_1 == 1){
       pose_2_face_2_reached = 1;
       go_to_pose_3_face_2   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 2 FACE 2 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_3_face_2_rms < joint_posi_rms_error && pose_3_face_2_reached == 0 && go_to_pose_2_face_1 == 1){
      pose_3_face_2_reached = 1;
      time_pose_3_face_2_reached = ros_time_now;
    }  
    
    
    if(pose_3_face_2_reached == 1 && (ros_time_now-time_pose_3_face_2_reached) > 0.5 && go_to_pose_4_face_2 == 0 && go_to_pose_2_face_1 == 1){
       pose_3_face_2_reached = 1;
       go_to_pose_4_face_2   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 3 FACE 2 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_4_face_2_rms < joint_posi_rms_error && pose_4_face_2_reached == 0 && go_to_pose_2_face_1 == 1){
      pose_4_face_2_reached = 1;
      time_pose_4_face_2_reached = ros_time_now;
    }  
    
    
    if(pose_4_face_2_reached == 1 && (ros_time_now-time_pose_4_face_2_reached) > 0.5 && go_to_pose_5_face_2 == 0 && go_to_pose_2_face_1 == 1){
       pose_4_face_2_reached = 1;
       go_to_pose_5_face_2   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 4 FACE 2 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


//**************************************************************************************************************************************//
    if(joint_posi_5_face_2_rms < joint_posi_rms_error && pose_5_face_2_reached == 0 && go_to_pose_2_face_1 == 1){
      pose_5_face_2_reached = 1;
      time_pose_5_face_2_reached = ros_time_now;
    }  
    
    
    if(pose_5_face_2_reached == 1 && (ros_time_now-time_pose_5_face_2_reached) > 0.5 && go_to_pose_6_face_2 == 0 && go_to_pose_2_face_1 == 1){
       pose_5_face_2_reached = 1;
       go_to_pose_6_face_2   = 1;
       
    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
    
    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
  
     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 5 FACE 2 !!!!!!!!!! "<<std::endl;
     
     pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);
    } 
//**************************************************************************************************************************************//


////**************************************************************************************************************************************//
//    if(joint_posi_6_face_2_rms < joint_posi_rms_error && pose_6_face_2_reached == 0){
//      pose_6_face_2_reached = 1;
//      time_pose_6_face_2_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_6_face_2_reached == 1 && (ros_time_now-time_pose_6_face_2_reached) > 0.5 && go_to_pose_7_face_2 == 0){
//       pose_6_face_2_reached = 1;
//       go_to_pose_7_face_2   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 6 FACE 2 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//
//
//
////**************************************************************************************************************************************//
//    if(joint_posi_7_face_2_rms < joint_posi_rms_error && pose_7_face_2_reached == 0){
//      pose_7_face_2_reached = 1;
//      time_pose_7_face_2_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_7_face_2_reached == 1 && (ros_time_now-time_pose_7_face_2_reached) > 0.5 && go_to_pose_8_face_2 == 0){
//       pose_7_face_2_reached = 1;
//       go_to_pose_8_face_2   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 7 FACE 2 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//
//
//
//
////**************************************************************************************************************************************//
//    if(joint_posi_8_face_2_rms < joint_posi_rms_error && pose_8_face_2_reached == 0){
//      pose_8_face_2_reached = 1;
//      time_pose_8_face_2_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_8_face_2_reached == 1 && (ros_time_now-time_pose_8_face_2_reached) > 0.5 && go_to_pose_9_face_2 == 0){
//       pose_8_face_2_reached = 1;
//       go_to_pose_9_face_2   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 8 FACE 2 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//
//
//
//
////**************************************************************************************************************************************//
//    if(joint_posi_9_face_2_rms < joint_posi_rms_error && pose_9_face_2_reached == 0){
//      pose_9_face_2_reached = 1;
//      time_pose_9_face_2_reached = ros_time_now;
//    }  
//    
//    
//    if(pose_9_face_2_reached == 1 && (ros_time_now-time_pose_9_face_2_reached) > 0.5 && go_to_pose_10_face_2 == 0){
//       pose_9_face_2_reached = 1;
//       go_to_pose_10_face_2   = 1;
//       
//    //Then we transform the partial pivot cloud to the ORIGINAL HOME POSE
//    
//    //translation ensuite rotation  (C'est la bonne ::::::::::::::::::::)
//    pcl::transformPointCloud (*pivot_cloud_without_nans, *pivot_cloud_without_nans_in_pivot_link_just_translated, -offset, rotation_identity.inverse());
//    pcl::transformPointCloud (*pivot_cloud_without_nans_in_pivot_link_just_translated, *pivot_cloud_without_nans_in_pivot_link, offset_null, rotation.inverse());
//  
//     //*recombined_pivot_cloud += *pivot_cloud_without_nans;
//    *recombined_pivot_cloud += *pivot_cloud_without_nans_in_pivot_link;
//     std::cout << "CLOUD ACQUIRED AND RECOMBINED FOR POSE 9 FACE 2 !!!!!!!!!! "<<std::endl;
//    } 
////**************************************************************************************************************************************//


    
    
    
    
    
    
    
    
    
    
    
    
/*
    //Transform the recombined cloud so it follows the pivot part
    pcl::transformPointCloud (*recombined_pivot_cloud, *transformed_recombined_pivot_cloud, offset, rotation);
    //pcl::copyPointCloud(*recombined_pivot_cloud, *transformed_recombined_pivot_cloud);
    
    

    recombined_pivot_cloud->header.frame_id             = "base_link";
    transformed_recombined_pivot_cloud->header.frame_id = "base_link";
    
    
    
    pcl_conversions::toPCL(ros::Time::now(), recombined_pivot_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), transformed_recombined_pivot_cloud->header.stamp);
    
    //recombined_pivot_cloud_pub.publish(recombined_pivot_cloud);
    recombined_pivot_cloud_pub.publish(transformed_recombined_pivot_cloud);
*/
    
    
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    
    
////////////////////////////////HERE WE START PLANNING !!!!!!////////////////////////////////    
if(go_to_pose_1_face_1 == 1){
  
  
if(success_pre_pick_pose == false){  
//Pre_pick_pose (With state name)
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  
  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_pose");

  move_group.setJointValueTarget(*current_state);

  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_pre_pick_pose = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pre_pick_pose : ", "Plan towards pre_pick_pose %s", success_pre_pick_pose ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}




if(success_pre_pick_pose == true && success_pick_pose == false){
//Pick_pose  (With name) 
  // Since we set the start state we have to clear it before planning other paths
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pick_pose");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  success_pick_pose = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pick_pose : ", "Plan towards pick_pose %s", success_pick_pose ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


  

     
//Operate the gripper
if(success_pick_pose ==true && joint_pick_pose_face_1_rms < joint_posi_rms_error && success_gripper_open == false){
  
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
  static const std::string PLANNING_GROUP2 = "gripper";

  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group2 = move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
  
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("gripper mvt : ", "gripper frame !!!!!!!!: %s", move_group2.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("gripper mvt : ", "gripper link !!!!!!: %s", move_group2.getEndEffectorLink().c_str());

  
//Open the gripper (With state name)
 // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions2;
  
  current_state2->setToDefaultValues(current_state2->getJointModelGroup("gripper"), "open_gripper");

  move_group2.setJointValueTarget(*current_state2);
  
  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  
  success_gripper_open = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("gripper_open : ", "Open gripper %s", success_gripper_open ? "" : "FAILED");

  //Move to the planned position
  move_group2.move();
  
  time_open_gripper = ros_time_now;
}
  
  


  //std::cout<<"(ros_time_now - time_open_gripper) : "<< (ros_time_now - time_open_gripper) <<std::endl;
  //std::cout<<" "<<std::endl;
  

  
  
//PLANNING POSE FOR PARTIAL CLOUD ACQUISITION N°1 FACE 1  ******************************
if(success_gripper_open == true && success_cloud_acq_pose1_face_1 == false && (ros_time_now - time_open_gripper) > 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose1_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose1_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose1_face_1 : ", "Plan towards cloud_acq_pose1_face_1 %s", success_cloud_acq_pose1_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}






  

//The other poses for face 1 **********************************************************

if(success_cloud_acq_pose1_face_1 == true && success_cloud_acq_pose2_face_1 == false && go_to_pose_2_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose2_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose2_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose2_face_1 : ", "Plan towards cloud_acq_pose2_face_1 %s", success_cloud_acq_pose2_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}

/*
std::cout<<"success_cloud_acq_pose2_face_1 : "<<success_cloud_acq_pose2_face_1<<std::endl;
std::cout<<"success_cloud_acq_pose3_face_1 : "<<success_cloud_acq_pose3_face_1<<std::endl;
std::cout<<"go_to_pose_1_face_1            : "<<go_to_pose_1_face_1<<std::endl;
*/


if(success_cloud_acq_pose2_face_1 == true && success_cloud_acq_pose3_face_1 == false && go_to_pose_3_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose3_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose3_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose3_face_1 : ", "Plan towards cloud_acq_pose3_face_1 %s", success_cloud_acq_pose3_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose3_face_1 == true && success_cloud_acq_pose4_face_1 == false && go_to_pose_4_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose4_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose4_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose4_face_1 : ", "Plan towards cloud_acq_pose4_face_1 %s", success_cloud_acq_pose4_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose4_face_1 == true && success_cloud_acq_pose5_face_1 == false && go_to_pose_5_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose5_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose5_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose5_face_1 : ", "Plan towards cloud_acq_pose5_face_1 %s", success_cloud_acq_pose5_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}

/*

if(success_cloud_acq_pose5_face_1 == true && success_cloud_acq_pose6_face_1 == false && go_to_pose_6_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose6_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose6_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose6_face_1 : ", "Plan towards cloud_acq_pose6_face_1 %s", success_cloud_acq_pose6_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose6_face_1 == true && success_cloud_acq_pose7_face_1 == false && go_to_pose_7_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose7_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose7_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose7_face_1 : ", "Plan towards cloud_acq_pose7_face_1 %s", success_cloud_acq_pose7_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose7_face_1 == true && success_cloud_acq_pose8_face_1 == false && go_to_pose_8_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose8_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose8_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose8_face_1 : ", "Plan towards cloud_acq_pose8_face_1 %s", success_cloud_acq_pose8_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose8_face_1 == true && success_cloud_acq_pose9_face_1 == false && go_to_pose_9_face_1 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose9_face_1");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose9_face_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose9_face_1 : ", "Plan towards cloud_acq_pose9_face_1 %s", success_cloud_acq_pose9_face_1 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}

*/


if(go_to_pose_1_face_2 == 1){
go_to_pose_1_face_1 = 0;
}
}    
    
    
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    
    
    
    
    
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$    
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FACE2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$    
//Here, go back to pick place, release the part and pick it the other side %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(go_to_pose_1_face_2 == 1){
  
  
if(success_cloud_acq_pose1_face_1 == true && success_pick_pose_bis == false){
//Pick_pose (With state name)
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pick_pose");

  move_group.setJointValueTarget(*current_state);

  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_pick_pose_bis = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pick_pose_bis : ", "Plan towards pick_pose_bis %s", success_pick_pose_bis ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
  
}




     
  
//Operate the gripper
if(success_pick_pose_bis == true && joint_pick_pose_face_1_rms < joint_posi_rms_error && success_gripper_close == false){
  
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
  static const std::string PLANNING_GROUP2 = "gripper";

  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group2 = move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
  
//Close the gripper (With state name)
 // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions2;
  
  current_state2->setToDefaultValues(current_state2->getJointModelGroup("gripper"), "close_gripper");

  move_group2.setJointValueTarget(*current_state2);
  
  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  
  success_gripper_close = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("gripper mvt : ", "Close gripper %s", success_gripper_close ? "" : "FAILED");

  //Move to the planned position
  move_group2.move();
  
  time_close_gripper = ros_time_now;
  
}
  
  
  
  
  
  

  
if(success_gripper_close == true && success_pre_pick_pose_bis == false && (ros_time_now - time_close_gripper) > 3.5){
//Pre_pick_pose  (With name) 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_pose");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  success_pre_pick_pose_bis = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pre_pick_pose_bis : ", "Plan towards success_pre_pick_pose_bis %s", success_pre_pick_pose_bis ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
  
  
  time_pre_pick_pose_bis_reached = ros_time_now;
}



  



//if(success_pre_pick_pose_bis == true && success_pre_pick_2_pose_0 == false && (ros_time_now - time_pre_pick_pose_bis_reached) > 1){
if(success_pre_pick_pose_bis == true && success_pre_pick_2_pose_0_mv.val != 1 && (ros_time_now - time_pre_pick_pose_bis_reached) > 1){
//Pre_pick_pose  (With name) 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_2_pose_0");

  move_group.setJointValueTarget(*current_state);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_pre_pick_2_pose_0 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pre_pick_2_pose_0 : ", "Plan towards pre_pick_2_pose_0 %s", success_pre_pick_2_pose_0 ? "" : "FAILED");

  //Move to the planned position
  success_pre_pick_2_pose_0_mv = move_group.move();
  std::cout<<"success_pre_pick_2_pose_0_mv : "<<success_pre_pick_2_pose_0_mv<<std::endl;
/*
  //while(success_pre_pick_2_pose_0_mv.val == moveit_msgs::MoveItErrorCodes::FAILURE){
  while(success_pre_pick_2_pose_0_mv.val != 1){
        success_pre_pick_2_pose_0_mv = move_group.move();
	std::cout<<"success_pre_pick_2_pose_0_mv : "<<success_pre_pick_2_pose_0_mv<<std::endl;
  }
*/
}
  
  
  


//if(success_pre_pick_2_pose_0 == true && success_pre_pick_2_pose_1 == false && joint_pre_pick_2_pose_0_rms < 1*joint_posi_rms_error){
if(success_pre_pick_2_pose_0 == true && success_pre_pick_2_pose_1_mv.val != 1 && joint_pre_pick_2_pose_0_rms < 1*joint_posi_rms_error){
  
//Pre_pick_pose  (With name) 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_2_pose_1");

  move_group.setJointValueTarget(*current_state);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_pre_pick_2_pose_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pre_pick_2_pose_1 : ", "Plan towards pre_pick_2_pose_1 %s", success_pre_pick_2_pose_1 ? "" : "FAILED");

  
  //Move to the planned position
  success_pre_pick_2_pose_1_mv = move_group.move();
  std::cout<<"success_pre_pick_2_pose_1_mv : "<<success_pre_pick_2_pose_1_mv<<std::endl;
/*
  //while(success_pre_pick_2_pose_1_mv.val == moveit_msgs::MoveItErrorCodes::FAILURE){
    while(success_pre_pick_2_pose_1_mv.val != 1){
        success_pre_pick_2_pose_1_mv = move_group.move();
	std::cout<<"success_pre_pick_2_pose_1_mv : "<<success_pre_pick_2_pose_1_mv<<std::endl;
  }
*/
}



//std::cout<<"joint_pre_pick_2_pose_1_rms : "<<joint_pre_pick_2_pose_1_rms<<std::endl;




//if(success_pre_pick_2_pose_1 == true && success_pre_pick_2_pose_2 == false && joint_pre_pick_2_pose_1_rms < 5*joint_posi_rms_error){
if(success_pre_pick_2_pose_1 == true && success_pre_pick_2_pose_2_mv.val != 1 && joint_pre_pick_2_pose_1_rms < 5*joint_posi_rms_error){

//Pre_pick_pose  (With name) 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_2_pose_2");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;  

  success_pre_pick_2_pose_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pre_pick_2_pose_2 : ", "Plan towards pre_pick_2_pose_2 %s", success_pre_pick_2_pose_2 ? "" : "FAILED");

  //Move to the planned position
  success_pre_pick_2_pose_2_mv = move_group.move();
  std::cout<<"success_pre_pick_2_pose_2_mv : "<<success_pre_pick_2_pose_2_mv<<std::endl;
/*
  //while(success_pre_pick_2_pose_2_mv.val == moveit_msgs::MoveItErrorCodes::FAILURE){
    while(success_pre_pick_2_pose_2_mv.val != 1){
        success_pre_pick_2_pose_2_mv = move_group.move();
	std::cout<<"success_pre_pick_2_pose_2_mv : "<<success_pre_pick_2_pose_2_mv<<std::endl;
  }
*/ 
}
  
  
  
//std::cout<<"joint_pre_pick_2_pose_2_rms : "<<joint_pre_pick_2_pose_2_rms<<std::endl;



  
//if(success_pre_pick_2_pose_2 == true && success_pre_pick_2_pose_3 == false && joint_pre_pick_2_pose_2_rms < 4*joint_posi_rms_error){
if(success_pre_pick_2_pose_2 == true && success_pre_pick_2_pose_3_mv.val != 1 && joint_pre_pick_2_pose_2_rms < 4*joint_posi_rms_error){

//Pre_pick_pose  (With name) 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pre_pick_2_pose_3");

  move_group.setJointValueTarget(*current_state);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_pre_pick_2_pose_3 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pre_pick_2_pose_3 : ", "Plan towards pre_pick_2_pose_3 %s", success_pre_pick_2_pose_3 ? "" : "FAILED");

  //Move to the planned position
  success_pre_pick_2_pose_3_mv = move_group.move();
  std::cout<<"success_pre_pick_2_pose_3_mv : "<<success_pre_pick_2_pose_3_mv<<std::endl;
/*
  //while(success_pre_pick_2_pose_3_mv.val == moveit_msgs::MoveItErrorCodes::FAILURE){
    while(success_pre_pick_2_pose_3_mv.val != 1){
        success_pre_pick_2_pose_3_mv = move_group.move();
	std::cout<<"success_pre_pick_2_pose_3_mv : "<<success_pre_pick_2_pose_3_mv<<std::endl;

  }
*/
  
}  

  

//std::cout<<"joint_pre_pick_2_pose_3_rms : "<<joint_pre_pick_2_pose_3_rms<<std::endl;

  

  
//if(success_pre_pick_2_pose_3 == true && success_pick_2_pose == false && joint_pre_pick_2_pose_3_rms < 4*joint_po
if(success_pre_pick_2_pose_3 == true && success_pick_2_pose_mv.val != 1 && joint_pre_pick_2_pose_3_rms < 4*joint_posi_rms_error){

//if(success_pre_pick_2_pose_3 == true && success_pick_2_pose == false){
//Pick_pose2  (With name) 
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  
  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "pick_2_pose");

  move_group.setJointValueTarget(*current_state);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_pick_2_pose = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pick and place : ", "Plan towards pick_2_pose %s", success_pick_2_pose ? "" : "FAILED");

  //Move to the planned position
  success_pick_2_pose_mv = move_group.move();
  std::cout<<"success_pick_2_pose_mv : "<<success_pick_2_pose_mv<<std::endl;
/*
  //while(success_pick_2_pose_mv.val == moveit_msgs::MoveItErrorCodes::FAILURE){
    while(success_pick_2_pose_mv.val != 1){
        success_pick_2_pose_mv = move_group.move();
	std::cout<<"success_pick_2_pose_mv : "<<success_pick_2_pose_mv<<std::endl;
  }
*/
}  



//std::cout<<"joint_pick_pose_face_2_rms : "<<joint_pick_pose_face_2_rms<<std::endl;



if(success_pick_2_pose == true && success_gripper_open_bis == false &&  joint_pick_pose_face_2_rms < joint_posi_rms_error){
//if(success_pick_2_pose == true && success_gripper_open_bis == false){
//Operate the gripper  
  
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group" are used interchangably.
  static const std::string PLANNING_GROUP2 = "gripper";

  // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group2 = move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);

  
//Open the gripper (With state name)
 // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions2;
  
  current_state2->setToDefaultValues(current_state2->getJointModelGroup("gripper"), "open_gripper");

  move_group2.setJointValueTarget(*current_state2);
  
  // Now, we call the planner to compute the plan then move.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  
  success_gripper_open_bis = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("gripper mvt : ", "Open gripper %s", success_gripper_open_bis ? "" : "FAILED");

  //Move to the planned position
  move_group2.move();
  
  time_open_gripper_bis = ros_time_now;
  
}
  
  
  
  
  


//PLANNING POSE FOR PARTIAL CLOUD ACQUISITION N°1 FACE 2  ******************************
if(success_gripper_open_bis == true && success_cloud_acq_pose1_face_2 == false && (ros_time_now-time_open_gripper_bis)>2){
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose1_face_2");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  success_cloud_acq_pose1_face_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("pick and place : ", "Plan towards cloud_acq_pose1_face_2 %s", success_cloud_acq_pose1_face_2 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
  
}








//The other poses for face 2 **********************************************************


if(success_cloud_acq_pose1_face_2 == true && success_cloud_acq_pose2_face_2 == false && go_to_pose_2_face_2 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose2_face_2");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose2_face_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose2_face_2 : ", "Plan towards cloud_acq_pose2_face_2 %s", success_cloud_acq_pose2_face_2 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}

/*
std::cout<<"success_cloud_acq_pose2_face_2 : "<<success_cloud_acq_pose2_face_2<<std::endl;
std::cout<<"success_cloud_acq_pose3_face_2 : "<<success_cloud_acq_pose3_face_2<<std::endl;
std::cout<<"go_to_pose_1_face_2            : "<<go_to_pose_1_face_2<<std::endl;
*/


if(success_cloud_acq_pose2_face_2 == true && success_cloud_acq_pose3_face_2 == false && go_to_pose_3_face_2 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose3_face_2");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose3_face_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose3_face_2 : ", "Plan towards cloud_acq_pose3_face_2 %s", success_cloud_acq_pose3_face_2 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose3_face_2 == true && success_cloud_acq_pose4_face_2 == false && go_to_pose_4_face_2 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose4_face_2");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose4_face_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose4_face_2 : ", "Plan towards cloud_acq_pose4_face_2 %s", success_cloud_acq_pose4_face_2 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}


if(success_cloud_acq_pose4_face_2 == true && success_cloud_acq_pose5_face_2 == false && go_to_pose_5_face_2 == 1){
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
  move_group.setStartStateToCurrentState();

  current_state->setToDefaultValues(current_state->getJointModelGroup("manipulator"), "cloud_acq_pose5_face_2");

  move_group.setJointValueTarget(*current_state);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  success_cloud_acq_pose5_face_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("cloud_acq_pose5_face_2 : ", "Plan towards cloud_acq_pose5_face_2 %s", success_cloud_acq_pose5_face_2 ? "" : "FAILED");

  //Move to the planned position
  move_group.move();
}







if(success_cloud_acq_pose5_face_2 == 1) {
go_to_pose_1_face_2 = 0;
}

}    
    
    
    
    
    
    

    
    

    
    
    

    
  
    
    

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
  
  

  // END_MVT
  ros::shutdown();
  return 0;
}
