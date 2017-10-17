#include "ros/ros.h"
#include "std_msgs/String.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
// #include <pcl/octree/octree2buf_base.h>


#include <iostream>
#include <vector>
#include <ctime>


float resolution = 128.0f;

ros::Publisher pub;
ros::Subscriber sub;

// pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
bool isFirstTime = true;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  
  pub.publish(output);

  // Octree

  if (isFirstTime) {
    pcl::PointCloud<pcl::PointXYZ> mls_points;

    pcl::fromROSMsg (*cloud_msg, mls_points);
    cloudA->resize(mls_points.size());

    for (size_t i = 0; i < mls_points.points.size(); ++i) 
    { 
        cloudA->points[i].x=mls_points.points[i].x; //error 
        cloudA->points[i].y=mls_points.points[i].y; //error 
        cloudA->points[i].z=mls_points.points[i].z; //error 
    }
    isFirstTime = false;
    ROS_INFO("TEST");
  } else 
  {
    // cloud a
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);


    // cloud b
    pcl::PointCloud<pcl::PointXYZ> mls_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (*cloud_msg, mls_points);
    cloudB->resize(mls_points.size());

    for (size_t i = 0; i < mls_points.points.size(); ++i) 
    { 
        cloudB->points[i].x=mls_points.points[i].x; //error 
        cloudB->points[i].y=mls_points.points[i].y; //error 
        cloudB->points[i].z=mls_points.points[i].z; //error 

    }
    isFirstTime = false;
    ROS_INFO("----------START-----------");
    std::cout << cloudA->points.size ()<< std::endl;
    std::cout << cloudA->width<< std::endl;
    std::cout << cloudA->height<< std::endl;
    std::cout << cloudB->points.size ()<< std::endl;
    std::cout << cloudB->width<< std::endl;
    std::cout << cloudB->height<< std::endl;

    ROS_INFO("----------END-----------");

    octree.setInputCloud (cloudA);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers ();
    octree.setInputCloud (cloudB);
    octree.addPointsFromInputCloud ();
    
    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels (newPointIdxVector);
    std::cout << newPointIdxVector.size ()<< std::endl;

    // ROS_INFO("Output from getPointIndicesFromNewVoxels:");
    int j = 0;
    for (size_t i = 0; i < newPointIdxVector.size (); ++i) {
      ROS_INFO("%d",j);
      j++;
      // ROS_INFO(newPointIdxVector[i]);
      // ROS_INFO(cloudPtr->points[newPointIdxVector[i]].x );
      // ROS_INFO(cloudPtr->points[newPointIdxVector[i]].y );
      // ROS_INFO(cloudPtr->points[newPointIdxVector[i]].z );
      // ROS_INFO(i + "# Index: " + newPointIdxVector[i] + " Point:" 
      //           + cloudPtr->points[newPointIdxVector[i]].x + " "
      //           + cloudPtr->points[newPointIdxVector[i]].y + " "
      //           + cloudPtr->points[newPointIdxVector[i]].z);
      // ROS_INFO("TEST");
    }
    cloudA = cloudB;
  }
  

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (&PointCloudXYZ);
  // isFirstTime = false;
  // ROS_INFO("TEST");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::PointCloud2>("/filterd_points", 1);
  sub = n.subscribe("/velodyne_points", 1, cloud_callback);

  ros::spin();
  return 0;
}
