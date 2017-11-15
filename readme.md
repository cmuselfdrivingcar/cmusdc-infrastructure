Ubuntu version: 14.04
ROS version: Indigo
CUDA version: 7.5

1. Velodyne:
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/velodyne.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

#disable wifi, connect ethernet 

#ip4 settings, change auto to manual: 
IP 192.168.1.25
netmask 255.255.255.0
gateway 0.0.0.0

#static ip for port
sudo ifconfig eth0 192.168.3.100

#route
sudo route add 192.168.1.201 eth0

rosrun velodyne_pointcloud gen_calibration.py PUCK_Calibration_File_2deg.xml
roslaunch velodyne_pointcloud VLP16_points.launch calibration:=/home/teame/catkin_ws/PUCK_Calibration_File_2deg.yaml
rosrun rviz rviz -f velodyne

2. PCL Installation:
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

#notes:
uncomment #include "octree2buf_base.h" in /usr/include/pcl-1.7/pcl/octree/octree_pointcloud.h



# Launch Octomap Steps
1. rosrun rviz rviz -f velodyne
2.1 roslaunch velodyne_pointcloud VLP16_points.launch calibration:=/home/teame16/catkin_ws/PUCK_Calibration_File_2deg.yaml
2.2 rosbag play -l *.bag 
3. rosrun background_subtraction downsampling 

3. roslaunch octomap_server octomap_mapping.launch 

# save background pcd
rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_points

# visualize background.pcd
rosrun pcl_ros pcd_to_pointcloud /home/teame16/CMUSelfDrivingCar/background.pcd 0.1 _frame_id:=/velodyne
