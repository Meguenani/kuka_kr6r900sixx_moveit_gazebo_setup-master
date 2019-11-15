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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include "euler2Quaternion.h"







using namespace cv;
using namespace pcl;
using namespace std;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_XYZ;
pcl::PointCloud<pcl::PointXYZ>::Ptr pivot_cloud (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pivot_cloud_1 (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pivot_cloud (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr partial_pivot_cloud (new PointCloud_XYZ);

pcl::PCLPointCloud2* scene_cloud_pcl = new pcl::PCLPointCloud2;
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_pcl_XYZ (new PointCloud_XYZ);





//To transfor the raw pivot pointcloud 
Eigen::Vector3f offset; 
Eigen::Vector3f offset_1; 
Eigen::Quaternionf rotation;
Eigen::Quaternionf rotation_1;

int read_pivot_pointcloud = 1;



void gazebo_models_Callback(const gazebo_msgs::ModelStates ms){
  /*
  std::cout<<" ms.pose[1].position.x    : "<< ms.pose[1].position.x <<std::endl;
  std::cout<<" ms.pose[1].position.y    : "<< ms.pose[1].position.y <<std::endl;
  std::cout<<" ms.pose[1].position.z    : "<< ms.pose[1].position.z <<std::endl;
  std::cout<<" ms.pose[1].orientation.x : "<< ms.pose[1].orientation.x <<std::endl;
  std::cout<<" ms.pose[1].orientation.y : "<< ms.pose[1].orientation.y <<std::endl;
  std::cout<<" ms.pose[1].orientation.z : "<< ms.pose[1].orientation.z <<std::endl;  
  std::cout<<" ms.pose[1].orientation.w : "<< ms.pose[1].orientation.w <<std::endl;
  
  std::cout<<" " <<std::endl;
  std::cout<<" " <<std::endl;
  */
  offset[0]    = ms.pose[1].position.x;
  offset[1]    = ms.pose[1].position.y;
  offset[2]    = ms.pose[1].position.z;
  rotation.x() = ms.pose[1].orientation.x;
  rotation.y() = ms.pose[1].orientation.y;
  rotation.z() = ms.pose[1].orientation.z;
  rotation.w() = ms.pose[1].orientation.w;
  /*
  std::cout<<" offset[0]    : "<< offset[0] <<std::endl;
  std::cout<<" offset[1]    : "<< offset[1] <<std::endl;
  std::cout<<" offset[2]    : "<< offset[2] <<std::endl;
  std::cout<<" rotation.x() : "<< rotation.x() <<std::endl;
  std::cout<<" rotation.y() : "<< rotation.y() <<std::endl;
  std::cout<<" rotation.z() : "<< rotation.z() <<std::endl;
  std::cout<<" rotation.w() : "<< rotation.w() <<std::endl;

  std::cout<<" " <<std::endl;
  std::cout<<" " <<std::endl;
  */
  //Broadcast the tf between the child_link=cylinder_link and the parent link=base_link
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()), tf::Vector3(offset[0], offset[1], offset[2])), ros::Time::now(), "base_link", "cylinder_link"));

}





void gazebo_cloud_Callback(const sensor_msgs::PointCloud2ConstPtr& scene_cloud_msg){ 
 
  //Convert scene_cloud_msg to PCL point cloud

  pcl_conversions::toPCL(*scene_cloud_msg, *scene_cloud_pcl);  
  pcl::fromPCLPointCloud2(*(scene_cloud_pcl), *(scene_cloud_pcl_XYZ));
   
  std::cout<<"msg->width :" << scene_cloud_msg->width <<std::endl;
  //Convert ros gazebo pointcloud to PCL pointcloud


  //Read the pivot pointcloud in PCL FORMAT (original)  ONCE !
  if(read_pivot_pointcloud == 1){
    
     if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/anis/catkin_ws/src/cylinder/meshes/pivot_cloud_10000.pcd", *pivot_cloud) == -1) //* load the file
     {
       PCL_ERROR ("Couldn't read file pivot_cloud.pcd \n");
     }
    std::cout<<"Pivot cloud pcd loaded !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
    read_pivot_pointcloud = 0;
  }


  //Transform the originat pivot pointcloud
/*
  Eigen::Quaterniond q = euler2Quaternion(3.14159265359, 0.0, 3.14159265359);

  rotation_1.x() = q.x();
  rotation_1.y() = q.y();
  rotation_1.z() = q.z();
  rotation_1.w() = q.w();


  std::cout<<" q.x() : "<< q.x() <<std::endl;
  std::cout<<" q.y() : "<< q.y() <<std::endl;
  std::cout<<" q.z() : "<< q.z() <<std::endl;
  std::cout<<" q.w() : "<< q.w() <<std::endl;

  std::cout<<" " <<std::endl;
  std::cout<<" " <<std::endl;

  std::cout<<" rotation_1.x() : "<< rotation_1.x() <<std::endl;
  std::cout<<" rotation_1.y() : "<< rotation_1.y() <<std::endl;
  std::cout<<" rotation_1.z() : "<< rotation_1.z() <<std::endl;
  std::cout<<" rotation_1.w() : "<< rotation_1.w() <<std::endl;

  offset_1[0]    = 0.0;
  offset_1[1]    = 0.0;
  offset_1[2]    = 0.0;
*/

  //pcl::transformPointCloud (*pivot_cloud, *transformed_pivot_cloud_1, offset_1, rotation_1);

  //pcl::transformPointCloud (*transformed_pivot_cloud_1, *transformed_pivot_cloud, offset, rotation);
  pcl::transformPointCloud (*pivot_cloud,               *transformed_pivot_cloud, offset, rotation);
  

   //Process the msg cloud and the pivot_cloud to compute the partial_pivot_cloud
   //THe filtering box 
   pcl::PointXYZ minPt, maxPt;
   pcl::getMinMax3D (*transformed_pivot_cloud, minPt, maxPt);
   std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
   std::cout << "Max x: " << maxPt.x << std::endl;
   std::cout << "Max y: " << maxPt.y << std::endl;
   std::cout << "Max z: " << maxPt.z << std::endl;
   std::cout << "Min x: " << minPt.x << std::endl;
   std::cout << "Min y: " << minPt.y << std::endl;
   std::cout << "Min z: " << minPt.z << std::endl;
   std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;


   pcl::CropBox<pcl::PointXYZ> boxFilter;
   boxFilter.setMin(Eigen::Vector4f(minPt.x, minPt.y, minPt.z, 1.0));
   boxFilter.setMax(Eigen::Vector4f(maxPt.x, maxPt.y, maxPt.z, 1.0));
   boxFilter.setInputCloud(scene_cloud_pcl_XYZ);
   boxFilter.filter(*partial_pivot_cloud);


}












int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;


  //Sibscribe to gazebo models
  ros::Subscriber sub_gazebo_models = nh.subscribe("/gazebo/model_states", 1000, gazebo_models_Callback);

  //Subscribe to the Xtion pointcloud
  ros::Subscriber sub_Xtion_cloud= nh.subscribe("/rgbd_camera/depth/points", 50, gazebo_cloud_Callback); //Waits for "GO!"

  //Publish the whole pointcloud of the Pivot part
  ros::Publisher transformed_pivot_cloud_pub = nh.advertise<PointCloud_XYZ> ("transformed_pivot_cloud", 1); //For the point cloud


  //Publish the partial pointcloud of the Pivot part
  ros::Publisher partial_pivot_cloud_pub = nh.advertise<PointCloud_XYZ> ("partial_pivot_cloud", 1); //For the point cloud





  transformed_pivot_cloud->header.frame_id = "base_link";
  transformed_pivot_cloud->points.clear();

  //transformed_pivot_cloud_1->header.frame_id = "base_link";
  //transformed_pivot_cloud_1->points.clear();
  
  partial_pivot_cloud->header.frame_id = "base_link";
  transformed_pivot_cloud->points.clear();



  ros::Rate loop_rate(5);
  while (ros::ok()) {


    transformed_pivot_cloud->header.frame_id   = "base_link";
    //transformed_pivot_cloud_1->header.frame_id = "base_link";
    partial_pivot_cloud->header.frame_id       = "base_link";

    pcl_conversions::toPCL(ros::Time::now(), transformed_pivot_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), transformed_pivot_cloud_1->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), partial_pivot_cloud->header.stamp);

    transformed_pivot_cloud_pub.publish(transformed_pivot_cloud);
    partial_pivot_cloud_pub.publish(partial_pivot_cloud);


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
