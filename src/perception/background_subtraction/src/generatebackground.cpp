#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>       // std::string
#include <sstream> 
#include "ros/ros.h"

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr background (new pcl::PointCloud<pcl::PointXYZ>);

  for (int i=1; i<=25; i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::stringstream ss;
    ss << i;
    std::string name = ss.str();
    std::string fullName = "/home/teame/Desktop/background_raw/"+name+".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fullName, *cloud) == -1) //* load the file
    {
      // PCL_ERROR ("Couldn't read file " + name + ".pcd \n");
      return (-1);
    }
    // std::cout << "Loaded "
    //           << cloud->width * cloud->height
    //           << " data points from <<"
    //           << name
    //           << ".pcd with the following fields: "
    //           << std::endl;
    // for (size_t i = 0; i < cloud->points.size (); ++i)
    //   std::cout << "    " << cloud->points[i].x
    //             << " "    << cloud->points[i].y
    //             << " "    << cloud->points[i].z << std::endl;

    // pcl::concatenateFields (*cloud, *background, *background);
    *background+=*cloud;
    std::cout << name
              << ".pcd Finished"
              << std::endl;
  }
  
  pcl::io::savePCDFileASCII ("background_newewew.pcd", *background);
  return (0);
}