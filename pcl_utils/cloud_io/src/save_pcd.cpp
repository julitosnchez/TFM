#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

unsigned int num = 0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  char pcd_file[15]; 

  pcl::PointCloud<pcl::PointXYZRGB> cloud; 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud(&cloud); 
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
  //pcl::PCLPointCloud2 cloud_filtered;

  std::cout << "TO PCL" << std::endl;
  // Convert to PCL data type
  pcl::fromROSMsg(*input, cloud);

  std::cout << "FILTER" << std::endl;
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (pcloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.8);
  pass.filter(cloud_filtered);

	sprintf(pcd_file, "test_pcd%d.pcd", num);
  std::cout << "SAVE" << std::endl;
  pcl::io::savePCDFileASCII(pcd_file , cloud_filtered); // ASCII format
  num++;
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "save_pcd");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 10, cloud_cb);
  
  // Spin
  ros::spin ();
}





