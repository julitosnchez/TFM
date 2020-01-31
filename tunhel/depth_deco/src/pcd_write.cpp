#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
int count=0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 

  count = count + 1;
	if (count==1){
  	  pcl::PointCloud<pcl::PointXYZ> cloud;

  	  pcl::fromROSMsg (*cloud_msg, cloud);

  	  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  	  std::cerr << "Saved " << cloud.points.size () << " data  points to test_pcd.pcd." << std::endl;

  	//for (size_t i = 0; i < cloud.points.size (); ++i)
    	//std::cerr << "    " << cloud.points[i].x << " " << 		cloud.points[i].y << " " << cloud.points[i].z << std::endl;
 	}
 
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
  

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
