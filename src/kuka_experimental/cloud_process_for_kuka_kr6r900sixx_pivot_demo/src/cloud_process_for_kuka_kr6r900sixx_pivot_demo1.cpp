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
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"

#include "/usr/local/include/pcl-1.8/pcl/point_cloud.h"
#include "/usr/local/include/pcl-1.8/pcl/point_types.h"
#include "/usr/local/include/pcl-1.8/pcl/common/io.h"
#include "/usr/local/include/pcl-1.8/pcl/filters/passthrough.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <rviz_visual_tools/rviz_visual_tools.h>



using namespace cv;
using namespace pcl;
using namespace std;
namespace rvt = rviz_visual_tools;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_XYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_XYZRGB;

pcl::PointCloud<pcl::PointXYZ>::Ptr pivot_cloud (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pivot_cloud_1 (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pivot_cloud (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr partial_pivot_cloud (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_pivot_cloud_RGB (new PointCloud_XYZRGB);

pcl::PCLPointCloud2* scene_cloud_pcl = new pcl::PCLPointCloud2;
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_pcl_XYZ (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZRGB (new PointCloud_XYZRGB);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZRGB_filtered (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZRGB_filtered2 (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZRGB_filtered3 (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZRGB_box_filtered (new PointCloud_XYZRGB);


pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_pcl_XYZ_filtered (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_pcl_XYZ_filtered2 (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_pcl_XYZ_filtered3 (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_pcl_XYZ_filtered4 (new PointCloud_XYZ);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZ_filtered_RGB  (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZ_filtered2_RGB (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZ_filtered3_RGB (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pcl_XYZ_filtered4_RGB (new PointCloud_XYZRGB);

pcl::PointCloud<pcl::PointXYZ>::Ptr clound_box_points (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr clound_box_points_transformed (new PointCloud_XYZ);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pivot_cloud_0 (new PointCloud_XYZ);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans_color_filtered  (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans_color_filtered2 (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans_color_filtered3 (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans_color_filtered4 (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans_color_filtered5 (new PointCloud_XYZRGB);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_msg_without_nans_color_filtered_and_box_filtered (new PointCloud_XYZRGB);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr recombined_pivot_cloud (new PointCloud_XYZRGB);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_recombined_pivot_cloud (new PointCloud_XYZRGB);

std::ostringstream saved_pointcloud_name;


//To transfor the raw pivot pointcloud 
Eigen::Vector3f offset; 
Eigen::Vector3f offset_1; 
Eigen::Quaternionf rotation;
Eigen::Quaternionf rotation_1;
Eigen::Quaternionf rotation_identity;
Eigen::Quaternion<double> eigein_quat_from_tool0_to_base_link(1, 0, 0, 0);
Eigen::Quaternion<double> rotation_to_tete_beche(1, 0, 0, 0);
Eigen::Quaternion<double> full_rotation_to_tete_beche(1, 0, 0, 0);


int read_pivot_pointcloud = 1;


double cube_pose_x = 0.225;   
double cube_pose_y = 0.7;  
double cube_pose_z = 0.5;

double width_cube  = 1.0; //ORIGINAL 0.45
double depth_cube  = 0.4;  //ORIGINAL 0.3
double height_cube = 0.8;  //ORIGINAL 0.3

double pass_z_min  = cube_pose_z - (height_cube/2);
double pass_z_max  = cube_pose_z + (height_cube/2);

double pass_y_min  = cube_pose_y - (depth_cube/2);
double pass_y_max  = cube_pose_y + (depth_cube/2);

double pass_x_min  = cube_pose_x - (width_cube/2);
double pass_x_max  = cube_pose_x + (width_cube/2);


visualization_msgs::Marker points, line_strip, line_list;
geometry_msgs::Point p;
int counter = 0;


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
  
  

  

  /*
  offset[0]    = ms.pose[2].position.x;
  offset[1]    = ms.pose[2].position.y;
  offset[2]    = ms.pose[2].position.z;
  rotation.x() = ms.pose[2].orientation.x;
  rotation.y() = ms.pose[2].orientation.y;
  rotation.z() = ms.pose[2].orientation.z;
  rotation.w() = ms.pose[2].orientation.w;
  */
  
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
  
 
}





//void gazebo_cloud_Callback(const sensor_msgs::PointCloud2ConstPtr& scene_cloud_msg){   //Subscription (1)
void gazebo_cloud_Callback(const PointCloud_XYZRGB::ConstPtr& scene_cloud_msg){             //Subscription (2)

  
  
    //std::cout << "scene_cloud_msg.size  !!!: " << scene_cloud_msg->size() <<std::endl;

      
      
      
    //remove NaN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*scene_cloud_msg,*scene_cloud_msg_without_nans, indices);
  

   //Color based filter   
   if(scene_cloud_msg_without_nans->points.size() > 0){ 	 
      for(int i=0; i<scene_cloud_msg_without_nans->points.size(); i++){
       if((int)scene_cloud_msg_without_nans->points[i].r < 5 && (int)scene_cloud_msg_without_nans->points[i].g < 5){
         scene_cloud_msg_without_nans_color_filtered->push_back(scene_cloud_msg_without_nans->points[i]);
         
       }  
      } 
   }
   
   
   
   if(scene_cloud_msg_without_nans->points.size() == 0){
      pcl::PointXYZRGB Pt;
      Pt.x = 0.0;
      Pt.y = 0.0;
      Pt.z = 0.0;
      Pt.r = 0.0;
      Pt.g = 0.0;
      Pt.b = 0.0; 
      
      scene_cloud_msg_without_nans_color_filtered->push_back(Pt);
   }
   
   

   
   
  //Convert scene_cloud_msg to PCL point cloud | Note : this conversion is used with the subscription (1) 
  //pcl::fromROSMsg(*scene_cloud_msg, *scene_cloud_pcl_XYZ);
   
  //We transform the entering pointcloud to publish it is the base_link "*scene_cloud_msg -> *scene_cloud_pcl_XYZ" (Here the cloud is transformed from the sensor frame to the robot base_link frame)
    pcl::transformPointCloud (*scene_cloud_msg_without_nans_color_filtered, *scene_cloud_pcl_XYZRGB, Eigen::Vector3f(0.2, 1.5, 0.5), Eigen::Quaternionf( 0.0, 0.0,  0.707,  -0.707));

    

    


  //Read the pivot pointcloud in PCL FORMAT (original)  ONCE !
  if(read_pivot_pointcloud == 1){
    
     if (pcl::io::loadPCDFile<pcl::PointXYZ> (std::string(std::getenv("HOME"))+"/catkin_ws/src/cylinder/meshes/pivot_cloud_10000.pcd", *pivot_cloud) == -1) //* load the file
     {
       PCL_ERROR ("Couldn't read file pivot_cloud.pcd \n");
     }
     else{
      std::cout<<"Pivot cloud pcd loaded !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
     }
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




  rotation_identity.x() = 0;
  rotation_identity.y() = 0;
  rotation_identity.z() = 0;
  rotation_identity.w() = 1;

  offset_1[0]    = 0.0;
  offset_1[1]    = 0.0;
  offset_1[2]    = 0.0;

    
    //Transform the loaded pointcloud into the base_link frame
    pcl::transformPointCloud (*pivot_cloud, *transformed_pivot_cloud, offset, rotation);
    
    //translation rotation en un seul coup (NE FONCTIONNE PAS!!!!!!)
    //pcl::transformPointCloud (*transformed_pivot_cloud_0, *transformed_pivot_cloud, -offset, rotation.inverse());
    
    /*
    //translation ensuite rotation  (C'est dans cet ordre que ça fonctionne !!!!!!!!!)
    pcl::transformPointCloud (*transformed_pivot_cloud_0, *transformed_pivot_cloud_1, -offset, rotation_identity.inverse());
    pcl::transformPointCloud (*transformed_pivot_cloud_1, *transformed_pivot_cloud, offset_1, rotation.inverse());
    */
    
    //pcl::copyPointCloud(*pivot_cloud, *transformed_pivot_cloud);


    
   //Process the msg cloud and the pivot_cloud to compute the partial_pivot_cloud
   //Tee filtering box 
   pcl::PointXYZ minPt, maxPt;
   pcl::getMinMax3D (*transformed_pivot_cloud, minPt, maxPt);


   clound_box_points->push_back(minPt);
   clound_box_points->push_back(maxPt);
   

   pcl::transformPointCloud (*clound_box_points, *clound_box_points_transformed, offset, rotation);
   //pcl::copyPointCloud(*clound_box_points, *clound_box_points_transformed);

   /*
   std::cout << "clound_box_points_transformed->points[0]: " << clound_box_points_transformed->points[0] << std::endl;
   std::cout << "clound_box_points_transformed->points[1]: " << clound_box_points_transformed->points[1] << std::endl;
   */
/*
   minPt.x = clound_box_points_transformed->points[0].x;
   minPt.y = clound_box_points_transformed->points[0].y;
   minPt.z = clound_box_points_transformed->points[0].z;
   maxPt.x = clound_box_points_transformed->points[1].x;
   maxPt.y = clound_box_points_transformed->points[1].y;
   maxPt.z = clound_box_points_transformed->points[1].z; 
*/
   
   /*
   std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
   std::cout << "Max x: " << maxPt.x << std::endl;
   std::cout << "Max y: " << maxPt.y << std::endl;
   std::cout << "Max z: " << maxPt.z << std::endl;
   std::cout << "Min x: " << minPt.x << std::endl;
   std::cout << "Min y: " << minPt.y << std::endl;
   std::cout << "Min z: " << minPt.z << std::endl;
   std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
   */
   
   
   

   
   

   
   //Box filter of "scene_cloud_msg_without_nans_color_filtered"
   cube_pose_x = 0.225;   
   cube_pose_y = 0.7;  
   cube_pose_z = 0.5;
   
   width_cube  = 0.45;
   depth_cube  = 0.3;
   height_cube = 0.3;
   
   pass_z_min  = cube_pose_z - (height_cube/2);
   pass_z_max  = cube_pose_z + (height_cube/2);
   
   pass_y_min  = cube_pose_y - (depth_cube/2);
   pass_y_max  = cube_pose_y + (depth_cube/2);
   
   pass_x_min  = cube_pose_x - (width_cube/2);
   pass_x_max  = cube_pose_x + (width_cube/2);
   
   
   
   //First filter the "scene_cloud_pcl_XYZ" point cloud 
   //Create the filtering object
   pcl::PassThrough<pcl::PointXYZRGB> pass_z;
   pass_z.setInputCloud (scene_cloud_pcl_XYZRGB);
   pass_z.setFilterFieldName ("z");
   pass_z.setFilterLimits (pass_z_min, pass_z_max);
   pass_z.filter (*scene_cloud_pcl_XYZRGB_filtered);
   
   //Create the filtering object y
   pcl::PassThrough<pcl::PointXYZRGB> pass_y;
   pass_y.setInputCloud (scene_cloud_pcl_XYZRGB_filtered);
   pass_y.setFilterFieldName ("y");
   pass_y.setFilterLimits (pass_y_min, pass_y_max);
   pass_y.filter (*scene_cloud_pcl_XYZRGB_filtered2);
   
   //Create the filtering object x
   pcl::PassThrough<pcl::PointXYZRGB> pass_x;
   pass_x.setInputCloud (scene_cloud_pcl_XYZRGB_filtered2);
   pass_x.setFilterFieldName ("x");
   pass_x.setFilterLimits (pass_x_min, pass_x_max);
   pass_x.filter (*scene_cloud_pcl_XYZRGB_filtered3);
   
   pcl::copyPointCloud(*scene_cloud_pcl_XYZRGB_filtered3, *scene_cloud_pcl_XYZRGB_box_filtered);

   
   
   if(scene_cloud_pcl_XYZRGB_box_filtered->points.size() == 0){
      pcl::PointXYZRGB Pt;
      Pt.x = 0.0;
      Pt.y = 0.0;
      Pt.z = 0.0;
      Pt.r = 0.0;
      Pt.g = 0.0;
      Pt.b = 0.0; 
      scene_cloud_pcl_XYZRGB_box_filtered->push_back(Pt);
   }
   

   pcl::copyPointCloud(*scene_cloud_pcl_XYZRGB_box_filtered, *partial_pivot_cloud_RGB);
   //pcl::copyPointCloud(*scene_cloud_pcl_XYZRGB, *partial_pivot_cloud_RGB);

   
   
   
   
   
   
   
   
   
   //Add p1
   p.x = minPt.x;
   p.y = minPt.y;
   p.z = minPt.z;         
   line_list.points.push_back(p);

   //Add p1
   p.x = minPt.x;
   p.y = maxPt.y;
   p.z = minPt.z;         
   line_list.points.push_back(p);
   

   //Add p1
   p.x = maxPt.x;
   p.y = minPt.y;
   p.z = minPt.z;     
   line_list.points.push_back(p);
   
	
   //Add p1
   p.x = maxPt.x;
   p.y = maxPt.y;
   p.z = minPt.z;         
   line_list.points.push_back(p);
	

   

   
   //Add p1
   p.x = minPt.x;
   p.y = minPt.y;
   p.z = maxPt.z;         
   line_list.points.push_back(p);
   
   //Add p1
   p.x = minPt.x;
   p.y = maxPt.y;
   p.z = maxPt.z;  
   line_list.points.push_back(p);
   
   //Add p1
   p.x = maxPt.x;
   p.y = maxPt.y;
   p.z = maxPt.z;     
   line_list.points.push_back(p);

	
   //Add p1
   p.x = maxPt.x;
   p.y = minPt.y;
   p.z = maxPt.z;    
   line_list.points.push_back(p);



}












int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::ServiceClient client;
  
  std::string modelName = (std::string)"cylinder";
  gazebo_msgs::GetModelState getModelState;
  


  
  
  
  rvt::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rvt::RvizVisualTools("base_link", "/rviz_visual_tools"));
  
  // Create pose
  Eigen::Isometry3d cube_pose;
  cube_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
  cube_pose.translation() = Eigen::Vector3d(0.225, 0.7, 0.5 ); // translate x,y,z

  
  
  geometry_msgs::Point pp ;
  geometry_msgs::Quaternion qq ;
  geometry_msgs::Quaternion qq_tete_beche ;
  geometry_msgs::Twist current_Twist_cylinder1 ;

  //Sibscribe to gazebo models
  ros::Subscriber sub_gazebo_models = nh.subscribe("/gazebo/model_states", 1000, gazebo_models_Callback);

  //Subscribe to the Xtion pointcloud
  ros::Subscriber sub_Xtion_cloud= nh.subscribe("/rgbd_camera/depth/points", 50, gazebo_cloud_Callback); //Waits for "GO!"

  //Publish the whole pointcloud of the Pivot part
  ros::Publisher transformed_pivot_cloud_pub = nh.advertise<PointCloud_XYZ> ("transformed_pivot_cloud", 1); //For the point cloud


  //Publish the partial pointcloud of the Pivot part
  ros::Publisher partial_pivot_cloud_pub = nh.advertise<PointCloud_XYZRGB> ("partial_pivot_cloud", 1); //For the point cloud

  
  //Publish the recombbined pointcloud of the Pivot part
  ros::Publisher recombined_pivot_cloud_pub = nh.advertise<PointCloud_XYZRGB> ("recombined_pivot_cloud", 1); //For the point cloud



  //Marker publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_p1", 10);

  
  
  
  
  

  transformed_pivot_cloud->header.frame_id = "base_link";
  transformed_pivot_cloud->points.clear();

  scene_cloud_pcl_XYZ->header.frame_id = "base_link";
  scene_cloud_pcl_XYZ->points.clear();
  
  partial_pivot_cloud->header.frame_id = "base_link";
  partial_pivot_cloud->points.clear();
  
  partial_pivot_cloud_RGB->header.frame_id = "base_link";
  partial_pivot_cloud_RGB->points.clear();

  scene_cloud_msg_without_nans_color_filtered->header.frame_id = "base_link";
  scene_cloud_msg_without_nans_color_filtered->points.clear();
  
  
  recombined_pivot_cloud->header.frame_id             = "base_link";
  recombined_pivot_cloud->points.clear(); 

  
  transformed_recombined_pivot_cloud->header.frame_id = "base_link";
  transformed_recombined_pivot_cloud->points.clear(); 
    
  saved_pointcloud_name << std::string(std::getenv("HOME"))+"/catkin_ws/devel/lib/moveit_cpp_for_kuka_kr6r900sixx/recombined_pivot_cloud.pcd";

  
  
  
  tf::TransformListener tf_listener;  


  
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.001;
  line_list.scale.x  = 0.001;
  
  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  
  
  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  
  //Publish the maerkers
  points.header.frame_id = line_strip.header.frame_id  = line_list.header.frame_id = "/base_link";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns  = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  
  


  //Ici on écrase le fichier recombined_pivot_cloud généré lors des précédentes simulations 
    if(recombined_pivot_cloud->points.size() == 0){
      pcl::PointXYZRGB Pt_;
      Pt_.x = 0.0;
      Pt_.y = 0.0;
      Pt_.z = 0.0;
      Pt_.r = 0.0;
      Pt_.g = 0.0;
      Pt_.b = 0.0; 
      
      recombined_pivot_cloud->push_back(Pt_);
   }

      pcl::io::savePCDFile(saved_pointcloud_name.str(), *recombined_pivot_cloud, true);





/*
 //If we want to publish wireframeCuboid only once (sometimes this does not work)
  //publishWireframeCuboid
  visual_tools_->publishWireframeCuboid(cube_pose, 0.45, 0.3, 0.3, rviz_visual_tools::BLUE, "Wireframe_Cuboid", 0);
  //double width, double depth, double height     
  
  // Don't forget to trigger the publisher!
  visual_tools_->trigger();
*/
  
  
  
  ros::Rate loop_rate(60);
  while (ros::ok()) {
    
    if(counter%120 == 0){
    //publishWireframeCuboid
    visual_tools_->publishWireframeCuboid(cube_pose, 0.45, 0.3, 0.3, rviz_visual_tools::BLUE, "Wireframe_Cuboid", 0);
    //double width, double depth, double height     
    
    // Don't forget to trigger the publisher!
    visual_tools_->trigger(); 
    }
    counter++;
    
    
    
    //Service to get cylinder model GetModelState
    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getModelState.request.model_name = modelName;
    client.call(getModelState);
    
    
    pp                      = getModelState.response.pose.position;
    qq                      = getModelState.response.pose.orientation;
    current_Twist_cylinder1 = getModelState.response.twist;

    offset[0]    = pp.x;
    offset[1]    = pp.y;
    offset[2]    = pp.z;
    rotation.x() = qq.x;
    rotation.y() = qq.y;
    rotation.z() = qq.z;
    rotation.w() = qq.w;
    
    
    
    
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

     

   	eigein_quat_from_tool0_to_base_link.x() = qq.x;
   	eigein_quat_from_tool0_to_base_link.y() = qq.y;
   	eigein_quat_from_tool0_to_base_link.z() = qq.z;
   	eigein_quat_from_tool0_to_base_link.w() = qq.w;
	
	
    





    rotation_to_tete_beche.x() = 0;
    rotation_to_tete_beche.y() = 0;
    rotation_to_tete_beche.z() = 1.0;
    rotation_to_tete_beche.w() = 0.0;

    full_rotation_to_tete_beche = eigein_quat_from_tool0_to_base_link*rotation_to_tete_beche;
    //full_rotation_to_tete_beche = eigein_quat_from_tool0_to_base_link;

    qq_tete_beche.x = full_rotation_to_tete_beche.x();
    qq_tete_beche.y = full_rotation_to_tete_beche.y();
    qq_tete_beche.z = full_rotation_to_tete_beche.z();
    qq_tete_beche.w = full_rotation_to_tete_beche.w();


    



	

    
    
    
    
    //Broadcast the tf between the child_link=cylinder_link and the parent link=base_link
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()), tf::Vector3(offset[0], offset[1], offset[2])), ros::Time::now(), "base_link", "cylinder_link"));

    
    

    transformed_pivot_cloud->header.frame_id                      = "base_link";
    scene_cloud_pcl_XYZ->header.frame_id                          = "base_link";
    partial_pivot_cloud->header.frame_id                          = "base_link";
    partial_pivot_cloud_RGB->header.frame_id                      = "base_link";
    scene_cloud_msg_without_nans_color_filtered->header.frame_id  = "base_link";
    
    pcl_conversions::toPCL(ros::Time::now(), transformed_pivot_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), scene_cloud_pcl_XYZ->header.stamp);
    //pcl_conversions::toPCL(ros::Time::now(), partial_pivot_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), partial_pivot_cloud_RGB->header.stamp);
    

    transformed_pivot_cloud_pub.publish(transformed_pivot_cloud);
    //partial_pivot_cloud_pub.publish(partial_pivot_cloud);
    
    //std::cout << "partial_pivot_cloud_RGB.size : " << partial_pivot_cloud_RGB->size() <<std::endl;

	 
    if(partial_pivot_cloud_RGB->points.size() == 0){
       pcl::PointXYZRGB Pt2;
       Pt2.x = 0.0;
       Pt2.y = 0.0;
       Pt2.z = 0.0;
       Pt2.r = 0.0;
       Pt2.g = 0.0;
       Pt2.b = 0.0; 
       partial_pivot_cloud_RGB->push_back(Pt2);
    }
    
    
    
    partial_pivot_cloud_pub.publish(partial_pivot_cloud_RGB);
    
    
    marker_pub.publish(line_list);
          
    clound_box_points->points.clear();     
    clound_box_points_transformed->points.clear();     
    scene_cloud_msg_without_nans_color_filtered->points.clear();
    
    /*
    try{
      tf_listener.lookupTransform("/base_link", "/tool0",  ros::Time(0), base_link_transform_tool0);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    */

    
    
//Read then publish the saved recombined pivot cloud 
    
     if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (saved_pointcloud_name.str(), *recombined_pivot_cloud) == -1) //* load the file
     {
       PCL_ERROR ("Couldn't read file pivot_cloud.pcd \n");
       
       
     }

    if(recombined_pivot_cloud->points.size() == 0){
      pcl::PointXYZRGB Pt_;
      Pt_.x = 0.0;
      Pt_.y = 0.0;
      Pt_.z = 0.0;
      Pt_.r = 0.0;
      Pt_.g = 0.0;
      Pt_.b = 0.0; 
      
      recombined_pivot_cloud->push_back(Pt_);
   }




    //Transform the recombined cloud so it follows the pivot part
    pcl::transformPointCloud (*recombined_pivot_cloud, *transformed_recombined_pivot_cloud, offset, rotation);

    recombined_pivot_cloud->header.frame_id             = "base_link";
    transformed_recombined_pivot_cloud->header.frame_id = "base_link";
    
    pcl_conversions::toPCL(ros::Time::now(), recombined_pivot_cloud->header.stamp);
    pcl_conversions::toPCL(ros::Time::now(), transformed_recombined_pivot_cloud->header.stamp);
    /*
    std::cout << "recombined_pivot_cloud.size              : " << recombined_pivot_cloud->size() <<std::endl;
    std::cout << "transformed_recombined_pivot_cloud.size  : " << transformed_recombined_pivot_cloud->size() <<std::endl;

    std::cout << " " <<std::endl;
    */
    recombined_pivot_cloud_pub.publish(transformed_recombined_pivot_cloud);
    
    recombined_pivot_cloud->points.clear(); 
    transformed_recombined_pivot_cloud->points.clear(); 
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
