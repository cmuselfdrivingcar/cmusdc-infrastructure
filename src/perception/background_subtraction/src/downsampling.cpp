#include "ros/ros.h"
#include "std_msgs/String.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/octree/octree2buf_base.h>


#include <iostream>
#include <vector>
#include <ctime>
#include <typeinfo>

float resolution = 0.1f;

ros::Publisher pub;
ros::Subscriber sub;

ros::Publisher pub_test;

// pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
bool isFirstTime = false;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  // Filtering Object
  // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;
  // pcl_conversions::toPCL(*cloud_msg, *cloud);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);  
  // pcl_conversions::fromPCL(cloud_filtered, output);
  // pub.publish(output);


  // Octree
  // cloud a
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  // cloud b
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*cloud_msg, *cloudB);

  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();
  octree.switchBuffers ();
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();
  
  std::vector<int> newPointIdxVector;

  octree.getPointIndicesFromNewVoxels (newPointIdxVector, 3);
  std::cout << newPointIdxVector.size ()<< std::endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  
  filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (*cloudB));
  for (std::vector<int>::iterator it = newPointIdxVector.begin (); it != newPointIdxVector.end (); ++it) {
    cloud_diff->points.push_back(filtered_cloud->points[*it]);
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud_diff, output);
  output.header.frame_id = "velodyne";
  pub.publish(output);


  // Creating the KdTree object for the search method of the extraction
  pcl::PCDWriter writer;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_diff);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_diff);
  ec.extract (cluster_indices);

  int j = 0;
  std::cout << cluster_indices.size ()<< std::endl;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_diff->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::PointCloud2>("/filterd_points", 1);
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/test_points", 1);
  sub = n.subscribe("/velodyne_points", 1, cloud_callback);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("background.pcd", *cloudA) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  ros::spin();
  return 0;
}
