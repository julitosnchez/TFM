#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <limits>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <depth_deco/ROI_identifier.h>
#include "DEOpt/DEOpt.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>

using namespace cv;
using namespace std;

std::vector<sensor_msgs::PointCloud2> cloudlist;



int main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "Hole_publication_node");
        ros::NodeHandle nh;

	// Create a ROS publisher for the ROI cloud
         ros::Publisher pub_ROIcloud2 = nh.advertise<sensor_msgs::PointCloud2> ("ROICloud2", 1);
        // Create a ROS publisher for the ROI cloud
         ros::Publisher pub_ROIcloud1 = nh.advertise<sensor_msgs::PointCloud2> ("ROICloud1", 1);
        
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 cld1_msg;
	sensor_msgs::PointCloud2 cld2_msg;

  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("roi1.pcd", *cld1) == -1) //* load the file
  	{
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return (-1);
  	}
  	std::cout << "Loaded Cloud 1:"
            << cld1->width * cld1->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  	/*for (size_t i = 0; i < cld1->points.size (); ++i)
    	std::cout << "    " << cld1->points[i].x
              << " "    << cld1->points[i].y
              << " "    << cld1->points[i].z << std::endl;*/

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("roi2.pcd", *cld2) == -1) //* load the file
  	{
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return (-1);
  	}
  	/*std::cout << "Loaded Cloud 2:"
            << cld2->width * cld2->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  	for (size_t i = 0; i < cld2->points.size (); ++i)
    	std::cout << "    " << cld2->points[i].x
              << " "    << cld2->points[i].y
              << " "    << cld2->points[i].z << std::endl;*/


	pcl::toROSMsg (*cld1, cld1_msg);
    	pcl::toROSMsg (*cld2, cld2_msg);
    	cld1_msg.header.frame_id = "world";
	cld2_msg.header.frame_id = "world";

         while(ros::ok())  {

		
		pub_ROIcloud1.publish(cld1_msg);		
		pub_ROIcloud2.publish(cld2_msg);
		ros::Duration(0.5).sleep();
		
	}
}

