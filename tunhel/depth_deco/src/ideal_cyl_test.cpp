#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

typedef pcl::PointXYZ PointT;
ros::Publisher pub_cloud;
ros::Publisher pub_CylPoints;
ros::Publisher pub_CylModel;

pcl::PointCloud<pcl::Normal>::Ptr calc_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //point cloud normals calculation
{
    //Objects
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    //Datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (5);
    ne.compute (*cloud_normals);

return (cloud_normals);
}


void cylinderModel (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //Cylinder model obtention and publishing from a ransac segmented point cloud
{
    //Objets
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    //Datasets
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
    sensor_msgs::PointCloud2 msg_cylinder;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.005, 1);
    seg.setInputCloud (cloud);



    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cld_cylinder);
    if (cld_cylinder->points.empty ())
        std::cout << "Can't find the cylindrical component." << std::endl;
      else
      {
              std::cout << "PointCloud representing the cylindrical component: " << cld_cylinder->points.size () << " data points." << std::endl;
              pcl::toROSMsg (*cld_cylinder, msg_cylinder);
              pub_CylPoints.publish (msg_cylinder);

              visualization_msgs::Marker cyl_marker;
              cyl_marker.header.frame_id = "/world";
              cyl_marker.header.stamp = ros::Time::now();
              //cyl_marker.ns = "basic_shapes" + boost::lexical_cast<std::string>(it);
              cyl_marker.id = 0;
              cyl_marker.type = visualization_msgs::Marker::CYLINDER;
              //cyl_marker.action = visualization_msgs::Marker::ADD;
              cyl_marker.pose.position.x = coefficients_cylinder->values[0];
              cyl_marker.pose.position.y = coefficients_cylinder->values[1];
              cyl_marker.pose.position.z = coefficients_cylinder->values[2];
              cyl_marker.pose.orientation.x = coefficients_cylinder->values[3];
              cyl_marker.pose.orientation.y = coefficients_cylinder->values[4];
              cyl_marker.pose.orientation.z = coefficients_cylinder->values[5];
              cyl_marker.pose.orientation.w = 1.0;
              cyl_marker.scale.x = coefficients_cylinder->values[6];
              cyl_marker.scale.y = coefficients_cylinder->values[6];
              cyl_marker.scale.z = coefficients_cylinder->values[6];
              cyl_marker.color.r = 0.0f;
              cyl_marker.color.g = 1.0f;
              cyl_marker.color.b = 0.0f;
              cyl_marker.color.a = 1.0;
              cyl_marker.lifetime = ros::Duration();
              pub_CylModel.publish(cyl_marker);
      }

}


void cylinderModel_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //Cylinder model obtention and publishing from a ransac segmented point cloud using normals
{
    //Objets
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PassThrough<PointT> pass;

    //Datasets
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
    sensor_msgs::PointCloud2 msg_cylinder;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);


    //Normal calculation
    cloud_normals=calc_normals(cloud);


    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (1000000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);


    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cld_cylinder);
    if (cld_cylinder->points.empty ())
        std::cout << "Can't find the cylindrical component." << std::endl;
      else
      {
              std::cout << "PointCloud representing the cylindrical component: " << cld_cylinder->points.size () << " data points." << std::endl;
              pcl::toROSMsg (*cld_cylinder, msg_cylinder);
              //pub_CylPoints.publish (msg_cylinder);

              visualization_msgs::Marker cyl_marker;
              cyl_marker.header.frame_id = "/world";
              cyl_marker.header.stamp = ros::Time::now();
              //cyl_marker.ns = "basic_shapes" + boost::lexical_cast<std::string>(it);
              cyl_marker.id = 0;
              cyl_marker.type = visualization_msgs::Marker::CYLINDER;
              //cyl_marker.action = visualization_msgs::Marker::ADD;
              cyl_marker.pose.position.x = coefficients_cylinder->values[0];
              cyl_marker.pose.position.y = coefficients_cylinder->values[1];
              cyl_marker.pose.position.z = coefficients_cylinder->values[2];
              cyl_marker.pose.orientation.x = coefficients_cylinder->values[3];
              cyl_marker.pose.orientation.y = coefficients_cylinder->values[4];
              cyl_marker.pose.orientation.z = coefficients_cylinder->values[5];
              cyl_marker.pose.orientation.w = 1.0;
              cyl_marker.scale.x = coefficients_cylinder->values[6];
              cyl_marker.scale.y = coefficients_cylinder->values[6];
              cyl_marker.scale.z = coefficients_cylinder->values[6];
              cyl_marker.color.r = 0.0f;
              cyl_marker.color.g = 1.0f;
              cyl_marker.color.b = 0.0f;
              cyl_marker.color.a = 1.0;
              cyl_marker.lifetime = ros::Duration();
              //pub_CylModel.publish(cyl_marker);
      }

}



int main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "ideal_cyl_test");
        ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
  	if (pcl::io::loadPLYFile<pcl::PointXYZ> ("cilinder_resampled.ply", *cloud) == -1) //* load the file
  	{
    		PCL_ERROR ("Couldn't read file \n");
    		
  	}
  	else {
            std::cout << "Loaded " << cloud->width * cloud->height << " data points from cilinder_resampled.ply with the following fields: " << std::endl;
 	    //for (size_t i = 0; i < cloud->points.size (); ++i)
    	    //std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " "    << cloud->points[i].z << std::endl;
        }

	sensor_msgs::PointCloud2 cloud_msg;
    	pcl::toROSMsg (*cloud, cloud_msg);
    	cloud_msg.header.frame_id = "world";
    	//pub_cloud.publish (cloud_msg);
	
	cylinderModel_normals(cloud);

        // Create a ROS publisher for the ROI cloud
        pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("Cloud", 1);

       
        // Create a ROS publisher for the Cylinder obtained through RANSAC
        pub_CylPoints = nh.advertise<sensor_msgs::PointCloud2> ("CylinderPoints", 1);

        // Create a ROS publisher for DE-Optmimized Cylinder Model
        pub_CylModel = nh.advertise<visualization_msgs::Marker> ("CylinderModel", 1);

        // Spin
        ros::spin ();
}
