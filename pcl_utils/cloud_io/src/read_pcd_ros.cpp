#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");

  std::string pcd_file;
  if(argc < 2) {
    printf("You must provide at least one argument\n");
    exit(0);
  }

  pcd_file = argv[1];

  ros::NodeHandle nh;

  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_pcd", 10);
 
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
 
  if (pcl::io::loadPCDFile (pcd_file, output) == -1) { //* load the file  
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  output.header.frame_id = "Base_Camera";  // for Stefan-s
  while (ros::ok()){
    pub.publish (output);
    ros::Duration(0.4).sleep(); // sleep for 0.4 seconds
  }

}




