#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*
Esto funciona pero la lectura de las PCDs es bastante lenta. 
Si queremos tenes información en un tiempo más realista, 10/20Hz, 
hay que hacer la lectura primero (off-line), y luego la publicación.

Cambiar MAX_NUM_PCDS segun los datos a utilizar.

Quiza la ultima PCD no esté bien.
*/


#define MAX_NUM_PCDS 33

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "multi_pcd_reader");
  ros::NodeHandle nh;

	char name[15];	  

  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_pcd", 10);
  //m_NodeHandle.subscribe< PointCloud<Point_t> >(topic_cloud_pcd, 1, &ParametricShapeFittingNode::receiveCloud, this);
  //pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("cloud_pcd", 10);

  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);

  unsigned int cloud_nr = 0;
  ros::Rate r(10); // 10 hz
  while (ros::ok()){
		sprintf(name, "test_pcd%d.pcd", cloud_nr);

    //if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *output) == -1) { //* load the file
    if (pcl::io::loadPCDFile (name, output) == -1) { //* load the file  
      PCL_ERROR ("Couldn't read file\n");
      return (-1);
    }
    output.header.frame_id = "camera_frame";
    pub.publish (output);
   
    if (cloud_nr < MAX_NUM_PCDS)
      cloud_nr = cloud_nr + 1;
    else
      cloud_nr = 0;

    //ros::Duration(0.2).sleep(); // sleep for 0.2 seconds
    r.sleep();
  }
}

