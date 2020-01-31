#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_cloud_matlab");
  ros::NodeHandle nh;

// The laser_scan_assembler accumulates laser scans by listening to the appropriate topic 
//   and accumulating messages in a ring buffer of a specific size. 
// When the assemble_scans service is called,
//   the contents of the current buffer that fall between two times are converted into a single cloud and returned.

  ros::service::waitForService("assemble_scans");
  ros::ServiceClient assemble_client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/assembled_cloud", 10);
  ros::Publisher pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2", 10);
 



  laser_assembler::AssembleScans srv;  
  // assemble from "NOW"
  srv.request.begin = ros::Time::now();
  
  ros::Duration(5).sleep();

  // assemble untill "NOW"
  srv.request.end = ros::Time::now();


  if (assemble_client.call(srv)) {
    std::cout << "Got cloud with " <<  srv.response.cloud.points.size() << " points\n";
    pub_cloud.publish(srv.response.cloud);
    // if the node is going to finish, the publisher needs some time to send the data
    ros::Duration(0.5).sleep();
   
    ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points.", (int)srv.response.cloud.points.size() );

    // conversion to PointCloud2
    sensor_msgs::PointCloud2 pcloud2;
    if (!sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, pcloud2)) {
      ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
      return -1;
    } 
    pub_cloud2.publish (pcloud2);
    ros::Duration(0.5).sleep();
 
  } 
  else {
    printf("Assembling service call failed\n");
  }
  return 0;
}

