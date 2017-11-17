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
#include <pcl/registration/ndt.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <typeinfo>

float resolution = 0.5f;

ros::Subscriber sub;

ros::Publisher pub_old_background;
ros::Publisher pub_new_background;
ros::Publisher pub_reference_frame;

// pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
pcl::PointCloud<pcl::PointXYZ>::Ptr old_background (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr new_background (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);


int receivedTimes = 0;
int MAXRECEIVETIMES = 1;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

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
  reg.setMaximumIterations (5);
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

  std::cout << targetToSource << std::endl;

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
  // source = new_scene;
  source = new_scene;
  target = old_background;

  // Add visualization data
  // showCloudsLeft(source, target);

  // PointCloud::Ptr temp (new PointCloud);
  pairAlign (source, target, result, pairTransform, false);

  //transform current pair into the global transform
  // pcl::transformPointCloud (*temp, *result, GlobalTransform);

  //update the global transform
  // GlobalTransform = GlobalTransform * pairTransform;

  //save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  ss << "registered" << ".pcd";
  pcl::io::savePCDFile (ss.str (), *result, true);

  *new_background = *result;
}

void start_reg2() {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(new_scene);
  icp.setInputTarget(old_background);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::transformPointCloud (*old_background, *new_background, icp.getFinalTransformation());
}

void start_reg3() {
  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (old_background);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (new_scene);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (0, 0, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  ndt.align (*new_background, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*old_background, *new_background, ndt.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("registered.pcd", *new_background);

}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  if (receivedTimes < MAXRECEIVETIMES)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *current_cloud);

    *new_scene += *current_cloud;
  } else if (receivedTimes == MAXRECEIVETIMES)
  {
    start_reg3();
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*old_background, output);
  output.header.frame_id = "velodyne";
  pub_old_background.publish(output);
  

  sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg (*new_background, output2);
  output2.header.frame_id = "velodyne";
  pub_new_background.publish(output2);

  sensor_msgs::PointCloud2 output3;
  pcl::toROSMsg (*filtered_cloud, output3);
  output3.header.frame_id = "velodyne";
  pub_reference_frame.publish(output3);

  receivedTimes ++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "downsampling");
  ros::NodeHandle n;
  sub = n.subscribe("/velodyne_points", 1, cloud_callback);
  pub_old_background = n.advertise<sensor_msgs::PointCloud2>("/old_background", 1);
  pub_new_background = n.advertise<sensor_msgs::PointCloud2>("/new_background", 1);
  pub_reference_frame = n.advertise<sensor_msgs::PointCloud2>("/reference_frame", 1);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("background.pcd", *old_background) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file background.pcd \n");
    return (-1);
  }

  ros::spin();
  return 0;
}
