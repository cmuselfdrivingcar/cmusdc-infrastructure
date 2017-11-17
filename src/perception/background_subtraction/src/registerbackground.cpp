#include "ros/ros.h"
#include "std_msgs/String.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <typeinfo>

float resolution = 0.5f;

ros::Subscriber sub;

// pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
pcl::PointCloud<pcl::PointXYZ>::Ptr old_background (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene (new pcl::PointCloud<pcl::PointXYZ>);

int receivedTimes = 0;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
  public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.05);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    // showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  // p->removePointCloud ("source");
  // p->removePointCloud ("target");

  // PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  // PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  // p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  // p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

  // PCL_INFO ("Press q to continue the registration.\n");
  // p->spin ();

  // p->removePointCloud ("source"); 
  // p->removePointCloud ("target");

  //add the source to the transformed target
  // *output += *cloud_src;
  
  final_transform = targetToSource;
}

void start_reg() {
  

  PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
  source = new_scene;
  target = old_background;

  // Add visualization data
  // showCloudsLeft(source, target);

  PointCloud::Ptr temp (new PointCloud);
  pairAlign (source, target, temp, pairTransform, true);

  //transform current pair into the global transform
  pcl::transformPointCloud (*temp, *result, GlobalTransform);

  //update the global transform
  // GlobalTransform = GlobalTransform * pairTransform;

  //save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  ss << "registered" << ".pcd";
  pcl::io::savePCDFile (ss.str (), *result, true);
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *current_cloud);

  *new_scene += *current_cloud;

  receivedTimes ++;

  if (receivedTimes >= 10)
  {
    start_reg();
    sub.shutdown();
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "downsampling");
  ros::NodeHandle n;
  sub = n.subscribe("/velodyne_points", 1, cloud_callback);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("background.pcd", *old_background) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file background.pcd \n");
    return (-1);
  }

  ros::spin();
  return 0;
}
