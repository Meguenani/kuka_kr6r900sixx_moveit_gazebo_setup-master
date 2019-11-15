## Needed Installations (this code has been tested on Ubuntu 16.04) : 

#### ROS & GAZEBO INSTALLATION

- sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

- sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

- sudo apt-get update

- sudo apt-get install ros-kinetic-ros-base

- sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

- wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

- sudo apt-get update

- sudo apt-get install gazebo9

- sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control ros-kinetic-gazebo9* (for Gazebo 9)

- sudo apt-get install ros-kinetic-catkin

- sudo apt-get install rviz

- sudo apt-get install ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller ros-kinetic-rqt ros-kinetic-rqt-controller-manager ros-kinetic-rqt-joint-trajectory-controller ros-kinetic-ros-control ros-kinetic-rqt-gui

- sudo apt-get install ros-kinetic-rqt-plot ros-kinetic-rqt-graph ros-kinetic-rqt-rviz ros-kinetic-rqt-tf-tree

- sudo apt-get install ros-kinetic-kdl-conversions ros-kinetic-kdl-parser ros-kinetic-forward-command-controller ros-kinetic-tf-conversions ros-kinetic-xacro ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher

- sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

- Check (https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a) on how to install Gazebo 9 alongside ROS. If you install Gazebo 9 after installing ROS, ROS may be broken ! Gazebo 9 should be installed first !

- If you need to remove ROS so you can properly install Gazebo 9: https://www.quora.com/How-do-I-uninstall-ROS-Melodic-package-in-Ubuntu-18-04

- Install gazebo_ros_pkgs using (sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control) and check your installation by launching (rosrun gazebo_ros gazebo), for more details see (http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros). You can also directly install gazebo_ros_pkgs from sources (https://github.com/ros-simulation/gazebo_ros_pkgs).

- If pcl_ros is missing, sudo apt-get install ros-kinetic-pcl-ros

- Install Moveit! (sudo apt install ros-kinetic-moveit, see https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

- sudo apt-get install ros-kinetic-rviz-visual-tools

- sudo apt-get install ros-kinetic-moveit-visual-tools

- sudo apt-get install ros-hydro-robot-state-publisher

- sudo apt-get install ros-kinetic-tf2-geometry-msgs

- May also be needed : https://github.com/ros-perception/pcl_msgs, https://github.com/ros-perception/perception_pcl and https://github.com/ros/geometry2

- OpenCV 2.4 (https://opencv.org/releases/), 

- VTK 8.2 REQUIRED (https://vtk.org/download/)

- PCL 1.8 or later (if you encounter any vtk related problem when installing PCL, disable vtk -> OFF from ccmake ..), 

- Eigen3 REQUIRED, you can check your current eigen version with "pkg-config --modversion eigen3" (http://eigen.tuxfamily.org/index.php?title=Main_Page) 










## The following packages are also needed (to be found on my github, these packages are to be placed in your catkin_ws/src) : 

- mastering_ros_robot_description_pkg

- cylinder

- Gripper2

- gazebo-pkgs-master

- gazebo_ros_pkgs-kinetic-devel









## To run the code (each step in a terminal): 

### Simulation with 1 pivot part: 

1. roscore

2. cd ~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/src/kuka_experimental/kuka_kr6_gazebo/launch
roslaunch kr6r900sixx_gazebo_1_pivot.launch

3. cd ~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/src/kuka_experimental/kuka_kr6r900sixx_moveit_config/launch
roslaunch moveit_planning_execution_rsi.launch sim:=true 

4. rosrun rviz rviz -d ~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/Robotics_1_pivot_simu.rviz

5. cd ~/catkin_ws/devel/lib/cloud_process_for_kuka_kr6r900sixx_pivot_demo
./cloud_process_for_kuka_kr6r900sixx_1_pivot_demo

6. cd ~/catkin_ws/devel/lib/moveit_cpp_for_kuka_kr6r900sixx
./move_group_interface_tutorial_kuka_kr6r900sixx 

7. Optional (made for testing possible poses)
cd ~/catkin_ws/devel/lib/moveit_cpp_for_kuka_kr6r900sixx
./save_kuka_kr6r900sixx_poses 

After launching ./save_kuka_kr6r900sixx_poses, the "Robotics_1_pivot_simu.rviz" template can be used on rviz. 
The robot can be moved with the Moveit! interface then, using "RvizVisualToolsGui", the pointcloud of each pose can be saved (push button "Next"). Push the "Break" button to reset the acquisition. The robot joint poses are available on the "/joint_states" topic (rostopic echo /joint_states). 

The new poses can be saved directly in the SRDF file to be found in "~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/src/kuka_experimental/kuka_kr6r900sixx_moveit_config/config"



### OR run the simulation with 2 pivot parts: 

1. roscore

2. cd ~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/src/kuka_experimental/kuka_kr6_gazebo/launch
roslaunch kr6r900sixx_gazebo_2_pivot.launch

3. cd ~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/src/kuka_experimental/kuka_kr6r900sixx_moveit_config/launch
roslaunch moveit_planning_execution_rsi.launch sim:=true 

4. rosrun rviz rviz -d ~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/Robotics_2_pivots_simu.rviz

5. cd ~/catkin_ws/devel/lib/cloud_process_for_kuka_kr6r900sixx_pivot_demo
./cloud_process_for_kuka_kr6r900sixx_2_pivot_demo

6. cd ~/catkin_ws/devel/lib/moveit_cpp_for_kuka_kr6r900sixx
./move_group_interface_tutorial_kuka_kr6r900sixx2 

7. Optional (made for testing possible poses)
cd ~/catkin_ws/devel/lib/moveit_cpp_for_kuka_kr6r900sixx
./save_kuka_kr6r900sixx_poses 

After launching ./save_kuka_kr6r900sixx_poses, the "Robotics_2_pivot_simu.rviz" template can be used on rviz. 
The robot can be moved with the Moveit! interface then, using "RvizVisualToolsGui", the pointcloud of each pose can be saved (push button "Next"). Push the "Break" button to reset the acquisition. The robot joint poses are available on the "/joint_states" topic (rostopic echo /joint_states). 

The new poses can be saved directly in the SRDF file to be found in "~/catkin_ws/src/kuka_kr6r900sixx_moveit_gazebo_setup-master/src/kuka_experimental/kuka_kr6r900sixx_moveit_config/config"


## Useful Links : 

- Code based on : https://github.com/SebNag/kuka_kr6r900sixx_moveit_gazebo_setup

- To handle grasping in Gazebo : https://github.com/jenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin

- Rviz visual tools : https://github.com/PickNikRobotics/rviz_visual_tools
