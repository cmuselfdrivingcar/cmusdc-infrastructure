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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <typeinfo>

float resolution = 0.1f;

ros::Publisher pub_backgroundsub;
ros::Publisher pub_cluster;
ros::Publisher pub_boudingbox;
ros::Subscriber sub;

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

  // remove outliners
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg (*cloud_msg, *cloud);

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloud);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*cloud_filtered);

  // sensor_msgs::PointCloud2 output_outliners;
  // pcl::toROSMsg (*cloud_filtered, output_outliners);
  // output_outliners.header.frame_id = "velodyne";
  // pub_backgroundsub.publish(output_outliners);

  // Octree
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff_euclidean (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*cloud_msg, *cloudB);

  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();
  octree.switchBuffers ();
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();
  
  std::vector<int> newPointIdxVector;

  octree.getPointIndicesFromNewVoxels (newPointIdxVector, 3);
  // std::cout << newPointIdxVector.size ()<< std::endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  
  filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (*cloudB));
  for (std::vector<int>::iterator it = newPointIdxVector.begin (); it != newPointIdxVector.end (); ++it) {
    cloud_diff->points.push_back(filtered_cloud->points[*it]);

    pcl::PointXYZ cloud_xyz;
    cloud_xyz.x = filtered_cloud->points[*it].x;
    cloud_xyz.y = filtered_cloud->points[*it].y;
    cloud_xyz.z = 1;

    cloud_diff_euclidean->points.push_back(cloud_xyz);
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud_diff, output);
  output.header.frame_id = "velodyne";
  pub_backgroundsub.publish(output);


  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_diff_euclidean);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2);
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_diff_euclidean);
  ec.extract (cluster_indices);

  int j = 0;
  // std::cout << "cluster size: " << cluster_indices.size ()<< std::endl;
  std::cout << "\n"<< std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_pedestrian_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      pcl::PointXYZRGB cloud_xyzrgb;
      cloud_xyzrgb.x = cloud_diff_euclidean->points[*pit].x;
      cloud_xyzrgb.y = cloud_diff_euclidean->points[*pit].y;
      cloud_xyzrgb.z = cloud_diff->points[*pit].z;
      
      cloud_xyzrgb.r = 30 * j;
      cloud_cluster->points.push_back (cloud_xyzrgb);
      single_pedestrian_cluster->points.push_back (cloud_xyzrgb);
    }

    Eigen::Vector4f ci;
    pcl::compute3DCentroid (*single_pedestrian_cluster, ci);

    pcl::PointXYZRGB centroid;
    centroid.getVector4fMap() = ci;
    std::cout << "pedestrian #" << j << "\t x: " << ci[0] << " y:"<< ci[1]<< std::endl;
    centroid.r = 255;
    centroid.g = 0;
    centroid.b = 0;
    centroid_cluster->points.push_back(centroid);
    // cloud_cluster->width = cloud_cluster->points.size ();
    // cloud_cluster->height = 1;
    // cloud_cluster->is_dense = true;

    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    j++;
  }

  sensor_msgs::PointCloud2 cluster_output;
  pcl::toROSMsg (*cloud_cluster, cluster_output);
  cluster_output.header.frame_id = "velodyne";
  pub_cluster.publish(cluster_output);

  // Centroid
  sensor_msgs::PointCloud2 centroid_output;
  pcl::toROSMsg (*centroid_cluster, centroid_output);
  centroid_output.header.frame_id = "velodyne";
  pub_boudingbox.publish(centroid_output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  pub_backgroundsub = n.advertise<sensor_msgs::PointCloud2>("/filterd_points", 1);
  pub_cluster = n.advertise<sensor_msgs::PointCloud2>("/clustered_points", 1);
  pub_boudingbox = n.advertise<sensor_msgs::PointCloud2>("/bbox_points", 1);
  sub = n.subscribe("/velodyne_points", 1, cloud_callback);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("background.pcd", *cloudA) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  ros::spin();
  return 0;
}
