#include "ros/ros.h"
#include "std_msgs/String.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <limits>

#include <pedestrian_tracking/PedestrianPose.h>
#include <pedestrian_tracking/PedestrianPoseList.h>

ros::Subscriber centroid_sub;

ros::Publisher pub_centroid_points;
ros::Publisher pub_centroid_msg;
pcl::PointXYZ previous_centroid(0.0f,0.0f,0.0f);

bool isFirstTime = true;
int count;

void centroid_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	pcl::PointCloud<pcl::PointXYZ>::Ptr centroids (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *centroids);
  float minDistance = FLT_MAX;

  int index=-1, i=0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = centroids->begin(); it!= centroids->end(); it++){
    // std::cout << it->x << ", " << it->y << ", " << it->z << std::endl;
    float distance = (it->x-previous_centroid.x)*(it->x-previous_centroid.x) +
    	(it->y-previous_centroid.y)*(it->y-previous_centroid.y);
    if (distance < minDistance) {
    	minDistance = distance;
    	index = i;
    }
    i++;
  }

  // std::cout << minDistance << std::endl;
  // std::cout << i << std::endl;
  // std::cout <<  std::endl;

  if (index >= 0) {
  	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  	centroid_cluster->points.push_back(centroids->points[index]);
  	// if (isFirstTime) {
   //    previous_centroid = centroids->points[index];
   //    isFirstTime = false;
   //  }
    previous_centroid = centroids->points[index];
    // else if (minDistance < 0.5) {
    //   previous_centroid = centroids->points[index];
    // }

  	sensor_msgs::PointCloud2 track_output;
	  pcl::toROSMsg (*centroid_cluster, track_output);
	  track_output.header.frame_id = "velodyne";
	  pub_centroid_points.publish(track_output);
	  
	  // TODO: change to real tracking
	  index = 1;
    pedestrian_tracking::PedestrianPoseList poselist;
		for(pcl::PointCloud<pcl::PointXYZ>::iterator it = centroids->begin(); it!= centroids->end(); it++){
	  	pedestrian_tracking::PedestrianPose pose;
			pose.pedID = index;
	  	pose.frameID = count;
			pose.x = it->x;
			pose.y = it->y;
			poselist.poses.push_back(pose);
			index++;
		}

    poselist.header.stamp = ros::Time::now();
    poselist.header.frame_id = "velodyne";
		poselist.frameID = count;

	  pub_centroid_msg.publish(poselist);
	  count++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_pedestrian");
  ros::NodeHandle n;
  centroid_sub = n.subscribe("/centroid_points", 10, centroid_callback);
  pub_centroid_points = n.advertise<sensor_msgs::PointCloud2>("/track_points", 10);
  pub_centroid_msg = n.advertise<pedestrian_tracking::PedestrianPoseList>("/track_points_msg", 10);

  count = 1;

  ros::spin();
  return 0;
}
