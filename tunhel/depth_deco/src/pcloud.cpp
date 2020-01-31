#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <pcl/filters/extract_indices.h>
#include <depth_deco/ROI_identifier.h>


ros::Publisher pub;

void depth_cb(const depth_deco::ROI_identifier hole_ROI)
{
   int ROI_x=hole_ROI.x_offset;
   int ROI_y=hole_ROI.y_offset;
   int height=hole_ROI.height;
   int width=hole_ROI.width;
   int seq_img=hole_ROI.seq;

   /*std::cout << "ROI point: (" <<hole_ROI.x_offset << "," << hole_ROI.y_offset<< ")"<< std::endl<< std::endl;*/

     sensor_msgs::PointCloud2ConstPtr cloud_msg;
  
  do{
    cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect2/hd/points", ros::Duration(0.2));
  }while(cloud_msg == NULL);

  std_msgs::Header h = cloud_msg->header;

  std::cout << "Cloud received sequence:" << h.seq<< std::endl<< std::endl;
  std::cout << "Depth sequence:" << seq_img<< std::endl<< std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ROI (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*cloud_msg, *cloud);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  for (int i=0; i< width; i++){
      for (int j=0; j<height; j++){

          inliers->indices.push_back( (j+ROI_y)*512 + (i+ROI_x) );
      }
  }

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_ROI);


  pcl::io::savePCDFileASCII ("ROI_pcd.pcd", *cloud_ROI);
  std::cerr << "Saved " << cloud_ROI->points.size () << " data  points to ROI_pcd.pcd." << std::endl;


  sensor_msgs::PointCloud2 cloud_ROI_msg;
  pcl::toROSMsg (*cloud_ROI, cloud_ROI_msg);
  cloud_ROI_msg.header.frame_id = "world";



  // Publish the data
  pub.publish (cloud_ROI_msg);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Cloud processing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/ROI_location", 1, depth_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("HoleCloud", 1);

  // Spin
  ros::spin ();
}
