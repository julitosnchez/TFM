#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "camera_info_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::CameraInfo> ("/camera/rgb/camera_info", 10);
  
  // Create a container for the data.
  sensor_msgs::CameraInfo info;
  unsigned int seq = 0;

  info.header.seq = seq;
  info.header.frame_id = "camera_rgb_optical_frame";
  info.height = 480;
  info.width = 640;
  info.distortion_model = "plumb_bob";
  info.D.resize(5);
  for (unsigned int t = 0; t < 5; t++) {
    info.D[t] = 0.0;
  }
  info.K[0] = 525.0;
  info.K[1] = 0.0;
  info.K[2] = 319.5;
  info.K[3] = 0.0;
  info.K[4] = 525.0;
  info.K[5] = 239.5;
  info.K[6] = 0.0;
  info.K[7] = 0.0;
  info.K[8] = 1.0;

  info.R[0] = 1.0;
  info.R[1] = 0.0;
  info.R[2] = 0.0;
  info.R[3] = 0.0;
  info.R[4] = 1.0;
  info.R[5] = 0.0;
  info.R[6] = 0.0;
  info.R[7] = 0.0;
  info.R[8] = 1.0;

  info.P[0] = 525.0;
  info.P[1] = 0.0;
  info.P[2] = 319.5;
  info.P[3] = 0.0;
  info.P[4] = 0.0;
  info.P[5] = 525.0;
  info.P[6] = 239.5;
  info.P[7] = 0.0;
  info.P[8] = 0.0;
  info.P[9] = 0.0;
  info.P[10] = 1.0;
  info.P[11] = 0.0;

  info.binning_x = 0;
  info.binning_y = 0;

  info.roi.x_offset = 0;
  info.roi.y_offset = 0;
  info.roi.height = 0;
  info.roi.width = 0;
  info.roi.do_rectify = false;

  tf::TransformBroadcaster br;

  tf::Transform transform_1;
  transform_1.setOrigin(tf::Vector3(0.0, -0.045, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform_1.setRotation(q);

  tf::Transform transform_2;
  transform_2.setOrigin(tf::Vector3(0.0, -0.02, 0.0) );
  q.setRPY(0, 0, 0);
  transform_2.setRotation(q);

  tf::Transform transform_3;
  transform_3.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
  q.setRPY(-1.571, -0.0, -1.571);
  transform_3.setRotation(q);

  tf::Transform transform_4;
  transform_4.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
  q.setRPY(-1.571, -0.0, -1.571);
  transform_4.setRotation(q);


  ros::Rate r(10); // 10 hz
  while (ros::ok()){
	  info.header.seq = seq;	
    info.header.stamp = ros::Time::now();
	  
    pub.publish(info);	 
    br.sendTransform(tf::StampedTransform(transform_1, ros::Time::now(), "/camera_link", "/camera_rgb_frame"));
    br.sendTransform(tf::StampedTransform(transform_2, ros::Time::now(), "/camera_link", "/camera_depth_frame"));
    br.sendTransform(tf::StampedTransform(transform_3, ros::Time::now(), "/camera_rgb_frame", "/camera_rgb_optical_frame"));
    br.sendTransform(tf::StampedTransform(transform_4, ros::Time::now(), "/camera_depth_frame", "/camera_depth_optical_frame"));

    seq = seq + 1;
    r.sleep();
  }
}
