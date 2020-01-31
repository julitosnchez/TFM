/*
 * manual_calibration.cpp
 *
 *  Created on: 27.2.2014
 *      Author: ivelas
 */

#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

#include <but_calibration_camera_velodyne/aux_class.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace message_filters;
using namespace but_calibration_camera_velodyne;

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;
string VELODYNE_COLOR_TOPIC;

ros::Publisher pub;

cv::Mat projection_matrix;

cv::Mat frame_rgb;
cv::Mat mirror;
vector<float> DoF;

PointCloud<PointXYZRGB> lev_dex(PointCloud<PointXYZRGB> &color_cloud)
{
  PointCloud<PointXYZRGB> new_cloud;
  PointXYZRGB pt_new;
  for (PointCloud<PointXYZRGB>::iterator pt = color_cloud.points.begin(); pt < color_cloud.points.end(); pt++)
  {  
      pt_new.rgb =pt->rgb; 
      pt_new.x=pt->y;
      pt_new.y=pt->x;
      pt_new.z=pt->z;
            
      new_cloud.push_back(pt_new);
    
  }
  return new_cloud;
}

//void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
//{
//  float p[12];
//  float *pp = p;
//  for (boost::array<double, 12ul>::const_iterator i = msg->P.begin(); i != msg->P.end(); i++)
//  {
//    *pp = (float)(*i);
//    pp++;

//  }

//  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
//}

//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{



//  /*mirror = cv_ptr->image;
//  flip(mirror, frame_rgb,1);*/
//}

void callback(const sensor_msgs::ImageConstPtr& msg_image,const sensor_msgs::CameraInfoConstPtr& msg_info,const sensor_msgs::PointCloud2ConstPtr& msg)
{

//	cout << "He entrado " << endl;

  // ---------------------------------------------------------------- IMAGE CALLBACK ----------------------------------------------------------------------------------------------------
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;

	// --------------------------------------------------------------- CAMERA INFO CALLBACK -----------------------------------------------------------------------------------------------
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;

  }

//  namedWindow( "display", WINDOW_AUTOSIZE );// Create a window for display.
//	imshow("display",frame_rgb);
	
//	int num_point = 0;
//	for(int i=0;i<frame_rgb.rows;i++)
//		for(int j=0;j<frame_rgb.cols;j++)
//			if(frame_rgb.at<Vec3b>(i,j).val[0] == 151 and frame_rgb.at<Vec3b>(i,j).val[62] and frame_rgb.at<Vec3b>(i,j).val[68])
//				num_point++;
////			cout << frame_rgb.at<Vec3b>(i,j) << endl;

//	cout << "NUM POINT: " << num_point << endl;



  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);

  // -------------------------------------------------------------- POINTCLOUD CALLBACK -------------------------------------------------------------------------------------------------
  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg, pc);

  // x := x, y := -z, z := y,
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);
  pointcloud = pointcloud.lev_dex();
  

  Image::Image img(frame_rgb);
  Velodyne::Velodyne transformed = pointcloud.transform(DoF);
  PointCloud<Velodyne::Point> visible_points;
  transformed.project(projection_matrix, Rect(0, 0, frame_rgb.cols, frame_rgb.rows), &visible_points);

  Velodyne::Velodyne visible_scan(visible_points);

  PointCloud<PointXYZRGB> color_cloud = visible_scan.colour(frame_rgb, projection_matrix);

  //Writing velodyne .pcd 	ESTO SE HA COMENTADO
//        std::ostringstream ost;
//	
//        ost << "color.pcd";
//        string vat=ost.str();
//        const char *filenamt=vat.c_str();
//        pcl::io::savePCDFileASCII (filenamt, color_cloud);
  
  

  color_cloud=lev_dex(color_cloud);
  
  // reverse axis switching:
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  transformPointCloud(color_cloud, color_cloud, transf);

  sensor_msgs::PointCloud2 color_cloud2;
  toROSMsg(color_cloud, color_cloud2);
  color_cloud2.header = msg->header;
   sensor_msgs::PointCloud2 pcloud2;

  pub.publish(color_cloud2);
  
  /*aux_class listener;
  pcl_ros::transformPointCloud	(std::string("kinect2_rgb_optical_frame"), color_cloud2, pcloud2, listener.frame_listener);*/

  /*tf::Quaternion q(0.0, 0.0, 0.0, 1);
  tf::Vector3 t(0.141994, -0.0488426, 0.0160775); 
  tf::Transform transform(q,t);
  pcl_ros::transformPointCloud	(color_cloud2, pcloud2, transform);
  pub.publish(pcloud2);*/

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coloring_node");
  
  ros::NodeHandle n;
  n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_color_topic", VELODYNE_COLOR_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/6DoF", DoF);
 

  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, CAMERA_FRAME_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
  pub = n.advertise<sensor_msgs::PointCloud2>(VELODYNE_COLOR_TOPIC, 1);


  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

//  // Subscribe input camera image
//  image_transport::ImageTransport it(n);
//  image_transport::Subscriber sub = it.subscribe(CAMERA_FRAME_TOPIC, 10, imageCallback);

//  ros::Subscriber info_sub = n.subscribe(CAMERA_INFO_TOPIC, 10, cameraInfoCallback);

//  ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>(VELODYNE_TOPIC, 1, pointCloudCallback);

  ros::spin();

  return EXIT_SUCCESS;
}
