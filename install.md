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
roslaunch velodyne_pointcloud 32e_points.launch calibration:=/home/teame/catkin_ws/PUCK_Calibration_File_2deg.yaml
rosrun rviz rviz -f velodyne

2. PCL Installation:
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

