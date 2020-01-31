// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// C++
#include <iostream>
using namespace std;

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
  //General
  //Row -> 4 colors: Grey, Red, Green, Blue / Column -> R G B
  int color[4][3] = { {155,155,155}, {255,0,0}, {0,255,0}, {0,0,255} }; 
  int object_id = 0;

  // Container for original cloud
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  ///////////////////////////////////
  
  //Some objects PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud,*temp_cloud);

  // Read in the cloud data
  std::cout << "PointCloud before filtering has: " << temp_cloud->points.size () << " data points." << std::endl; 

  // Create the filtering object: downsample the dataset using a leaf size of 5cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (temp_cloud);
  vg.setLeafSize (0.01, 0.01, 0.01); 
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
	
    //Colorize planar point clouds
    for (size_t i_point = 0; i_point < cloud_plane->points.size (); i_point++)
    {
       pcl::PointXYZRGB point;
       point.x = *(cloud_plane->points[i_point].data);
       point.y = *(cloud_plane->points[i_point].data + 1);
       point.z = *(cloud_plane->points[i_point].data + 2);
       
       //First "object": planar component --> object_id = 0, first row in color array
       point.r = color[object_id][0]; 
       point.g = color[object_id][1];
       point.b = color[object_id][2];

       colored_cloud->points.push_back (point);
     }
  }
  std::cout << "Colored_cloud done." << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    { 
	cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
    } 

    //Colorize each cluster
    for (size_t i_point = 0; i_point < cloud_cluster->points.size (); i_point++)
    {
         pcl::PointXYZRGB point;
         point.x = *(cloud_cluster->points[i_point].data);
         point.y = *(cloud_cluster->points[i_point].data + 1);
         point.z = *(cloud_cluster->points[i_point].data + 2);

	 object_id = j%3+1; //Every 3 objects, repeat the color (row 2, 3, 4 of color array)
	 point.r = color[object_id][0];
	 point.g = color[object_id][1];
	 point.b = color[object_id][2];

         colored_cloud->points.push_back (point);
    }
   
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "object_" << j;
    std::cout << "Clustering object: " << ss.str () << std::endl;
    j++;
  }
  ///////////////////////////////////*/

  // Convert to ROS data type
  pcl::PCLPointCloud2* aux_filtered = new pcl::PCLPointCloud2;

  //pcl::toPCLPointCloud2(*cloud_filtered, *aux_filtered); //Para probar a mostrar sólo la nube filtrada
  pcl::toPCLPointCloud2(*colored_cloud, *aux_filtered); 
  
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(*aux_filtered, output);

  std::cout << "Publicando salida............. " <<  std::endl;

  // Publish the data
  output.header.frame_id = "/camera_link";
  output.header.stamp = ros::Time::now();

  pub.publish (output);
  std::cout << "--- Publicada salida ---" <<  std::endl;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial3");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_pcd", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
