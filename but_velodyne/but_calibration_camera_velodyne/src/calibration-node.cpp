#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Calibration.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include <but_calibration_camera_velodyne/Image.h>

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;



// marker properties:
double STRAIGHT_DISTANCE; // 23cm
double RADIUS; // 8.25cm

ros::Publisher pub;
ros::Publisher vis_pub;
ros::Publisher pub_pc;

Mat projection_matrix;
Mat frame_rgb;
Mat mirror;
Velodyne::Velodyne pointcloud;
Velodyne::Velodyne aux;
bool doRefinement = false;


bool writeAllInputs()
{
  bool result = true;

  pointcloud.save("velodyne_pc.pcd");
  cv::imwrite("frame_rgb.png", frame_rgb);
  cv::FileStorage fs_P("projection.yml", cv::FileStorage::WRITE);
  fs_P << "P" << projection_matrix;
  fs_P.release();

  return result;
}

Calibration6DoF calibration(bool doRefinement = false)
{
  
  Mat frame_gray;

  cvtColor(frame_rgb, frame_gray, CV_BGR2GRAY);
  
  // Marker detection:
  Calibration3DMarker marker(frame_gray, projection_matrix, pointcloud.getPointCloud(), STRAIGHT_DISTANCE, RADIUS);
  vector<float> radii2D;
  vector<Point2f> centers2D;
  
  if (!marker.detectCirclesInImage(centers2D, radii2D))
  {
    return Calibration6DoF::wrong();
  }
  float radius2D = accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
  
  vector<float> radii3D;
  vector<Point3f> centers3D;
	PointCloud<PointXYZ> pc;


  if (!marker.detectCirclesInPointCloud(centers3D, radii3D,pc))
  {
    return Calibration6DoF::wrong();
  }
  float radius3D = accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();

	cout << "ESTOY DENTRO " << endl;



  pcl::PointCloud<pcl::PointXYZ> msg (pc);
  msg.header.frame_id = "base_link2";
  pcl_conversions::toPCL(ros::Time::now(), msg.header.stamp);

	pub_pc.publish(msg);
  ros::Duration(2).sleep();

	cout << "Publishing pointcloud con " << msg.size() << endl;

  visualization_msgs::Marker markerSphere;
  markerSphere.header.frame_id = "base_link2";
  markerSphere.header.stamp = ros::Time::now();
//  markerSphere.ns = "my_namespace";
  for(int i=0;i<4;i++)
	{

    markerSphere.id = i;
    markerSphere.type = visualization_msgs::Marker::SPHERE;
    markerSphere.action = visualization_msgs::Marker::ADD;
    markerSphere.pose.position.x = centers3D[i].x;
    markerSphere.pose.position.y = centers3D[i].y;
//		centers3D[i].z += 0.02;
    markerSphere.pose.position.z = centers3D[i].z;

    markerSphere.pose.orientation.x = 0.0;
    markerSphere.pose.orientation.y = 0.0;
    markerSphere.pose.orientation.z = 0.0;
    markerSphere.pose.orientation.w = 1.0;

    markerSphere.scale.x = radii3D[i];
    markerSphere.scale.y = radii3D[i];
    markerSphere.scale.z = radii3D[i];

    markerSphere.color.a = 1.0; // Don't forget to set the alpha!
    markerSphere.color.r = 0.0;
    markerSphere.color.g = 1.0;
    markerSphere.color.b = 0.0;

    cout << "Publishing --> (" << centers3D[i].x << "," << centers3D[i].y << "," << centers3D[i].z << ")" << endl;
    vis_pub.publish( markerSphere );
    ros::Duration(1).sleep();
    
    
  }


//  sensor_msgs::PointCloud2 cloud_plane;
//  pcl::toROSMsg (pointcloud.open("vis_scan.pcd"), cloud_plane);
//	cloud_plane.header.frame_id = "velodyne";
//  pub.publish(cloud_plane);
//  ros::Duration(1).sleep();
  
  // rough calibration
  Calibration6DoF translation = Calibration::findTranslation(centers2D, centers3D, projection_matrix, radius2D, radius3D);
  
  if (doRefinement)
  {
    ROS_INFO("Coarse calibration:");
    translation.print();
    ROS_INFO("Refinement process started - this may take a minute.");
    size_t divisions = 5;
    float distance_transl = 0.02;
    float distance_rot = 0.01;
    Calibration6DoF best_calibration, avg_calibration;
    Calibration::calibrationRefinement(Image::Image(frame_gray), pointcloud, projection_matrix, translation.DoF[0],
                                       translation.DoF[1], translation.DoF[2], distance_transl, distance_rot, divisions,
                                       best_calibration, avg_calibration);
    return avg_calibration;
  }
  else
  {
    return translation;
  }
}

void callback(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& msg_info,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
  ROS_INFO_STREAM("Image received at " << msg_img->header.stamp.toSec());
  ROS_INFO_STREAM( "Camera info received at " << msg_info->header.stamp.toSec());
  ROS_INFO_STREAM( "Velodyne scan received at " << msg_pc->header.stamp.toSec());

  // Loading camera image:
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;
  /*mirror = cv_ptr->image;
  flip(mirror, frame_rgb,1);*/

  /*namedWindow("Original", CV_WINDOW_AUTOSIZE);//ROI_scaled= scaled selected roi
  imshow("Original", mirror);
  waitKey(0);

  namedWindow("Espejo", CV_WINDOW_AUTOSIZE);//ROI_scaled= scaled selected roi
  imshow("Espejo", frame_rgb);
  waitKey(0);*/ 

  // Loading projection matrix from param P of topic camera_info:
  float p[12];
  float *pp = p;

  for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;
  }
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);

  // Loading Velodyne point cloud
  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg_pc, pc);

	
//	pc.save("velodyne_original.pcd");
  // x := x, y := -z, z := y,
  aux = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);
//aux = Velodyne::Velodyne(pc).transform(0, 0, 0,0, 0, 0);
//  aux.save("nube.pcd");
  pointcloud = Velodyne::Velodyne(aux.getPointCloud()).lev_dex(); // ¿CREA COPIA O QUE  HACE?
  


  sensor_msgs::PointCloud2 cloud_velodyne;
  pcl::toROSMsg (pointcloud.getPointCloud(), cloud_velodyne);
	cloud_velodyne.header.frame_id = "base_link";
  pub.publish(cloud_velodyne);
  ros::Duration(1).sleep();

  //pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

  // calibration:
  writeAllInputs();

  Calibration6DoF calibrationParams = calibration(doRefinement);

  if (calibrationParams.isGood())
  {
    ROS_INFO_STREAM("Calibration succeeded, found parameters:");
    calibrationParams.print();
    shutdown();
  }
  else
  {
    ROS_WARN("Calibration failed - trying again after 5s ...");
    ros::Duration(5).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");

  int c;
  while ((c = getopt(argc, argv, "r")) != -1)
  {
    switch (c)
    {
      case 'r':
        doRefinement = true;
        break;
      default:
        return EXIT_FAILURE;
    }
  }

  ros::NodeHandle n;
  n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/marker/circles_distance", STRAIGHT_DISTANCE);
  n.getParam("/but_calibration_camera_velodyne/marker/circles_radius", RADIUS);

  std::cout << CAMERA_FRAME_TOPIC << std::endl;
  std::cout << VELODYNE_TOPIC << std::endl;
  std::cout << CAMERA_INFO_TOPIC << std::endl;

  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, CAMERA_FRAME_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
  pub = n.advertise<sensor_msgs::PointCloud2>("Velodyne_cloud_girada", 1);

	//Synchronizing the messages received in the same CALLBACK
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


  pub_pc = n.advertise<PointCloud<PointXYZ> >("pc_marker", 0);

  ros::spin();

  return EXIT_SUCCESS;
}
