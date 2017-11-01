#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>       // std::string
#include <sstream> 
#include "ros/ros.h"

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr background (new pcl::PointCloud<pcl::PointXYZ>);
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir ("/home/teame16/Desktop/background_raw/Version3/")) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      if (strcmp(ent->d_name,".") == 0 || strcmp(ent->d_name,"..") == 0 )
        continue;
      // printf ("%s\n", ent->d_name);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      // std::stringstream ss;
      // ss << i;
      char fullName[1000];
      // std::string name = ent->d_name;
      // std::string fullName = "/home/teame16/Desktop/background_raw/Version3/"+ent->d_name;
      strcpy (fullName,"/home/teame16/Desktop/background_raw/Version3/");
      strcat (fullName,ent->d_name);
      printf ("%s\n", fullName);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (fullName, *cloud) == -1) //* load the file
      {
        return (-1);
      }
      *background+=*cloud;
      std::cout << ent->d_name
                << " Finished"
                << std::endl;
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    return EXIT_FAILURE;
  }
  
  pcl::io::savePCDFileASCII ("background_neweweNewew.pcd", *background);
  return (0);
}