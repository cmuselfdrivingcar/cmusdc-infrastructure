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
#include <typeinfo>

float resolution = 0.1f;

ros::Publisher pub;
ros::Publisher pub_original;
ros::Subscriber sub;

// pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
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


  // Octree
  // pcl::PCLPointCloud2 filtered_cloud;
  if (isFirstTime) {
    // pcl::PointCloud<pcl::PointXYZRGBA> mls_points;

    pcl::fromROSMsg (*cloud_msg, *cloudA);
    // cloudA->resize(mls_points.size());

    // for (size_t i = 0; i < mls_points.points.size(); ++i) 
    // { 
    //     cloudA->points[i].x=mls_points.points[i].x; //error 
    //     cloudA->points[i].y=mls_points.points[i].y; //error 
    //     cloudA->points[i].z=mls_points.points[i].z; //error 
    // }
    isFirstTime = false;
  } else 
  {
    // cloud a
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> octree (resolution);


    // cloud b
    // pcl::PointCloud<pcl::PointXYZRGBA> mls_points;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromROSMsg (*cloud_msg, *cloudB);

    ROS_INFO("----------START-----------");
    // std::cout << cloudA->points.size ()<< std::endl;
    std::cout << cloudA->width<< std::endl;
    std::cout << cloudA->height<< std::endl;
    // std::cout << cloudB->points.size ()<< std::endl;
    std::cout << cloudB->width<< std::endl;
    std::cout << cloudB->height<< std::endl;


    octree.setInputCloud (cloudA);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers ();
    octree.setInputCloud (cloudB);
    octree.addPointsFromInputCloud ();
    
    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels (newPointIdxVector, 2);
    std::cout << newPointIdxVector.size ()<< std::endl;
    ROS_INFO("----------END-----------");


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud;
    
    filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (*cloudB));
    for (std::vector<int>::iterator it = newPointIdxVector.begin (); it != newPointIdxVector.end (); ++it) {
      // int32_t rgb = (255<<16) | (0 << 8) | 0;
      // filtered_cloud->points[*it].rgba = *(float *)(&rgb);
      cloud_diff->points.push_back(filtered_cloud->points[*it]);
      // std::cout << typeid(filtered_cloud->points[*it]).name()<< std::endl;
      // std::cout << filtered_cloud->points[*it].r<< std::endl;
    }
    // cloudA = cloudB;

    sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg (*filtered_cloud, output);
    pcl::toROSMsg (*cloud_diff, output);
    output.header.frame_id = "velodyne";
    pub.publish(output);

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg (*cloudB, output2);
    pub_original.publish(output2);
  }



  // pcl_conversions::fromPCL(cloud_filtered, output);

  
  // pub.publish(output);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (&PointCloudXYZ);
  // isFirstTime = false;
  // ROS_INFO("TEST");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::PointCloud2>("/filterd_points", 1);
  pub_original = n.advertise<sensor_msgs::PointCloud2>("/filterd_points_original", 1);
  sub = n.subscribe("/velodyne_points", 1, cloud_callback);

  ros::spin();
  return 0;
}
