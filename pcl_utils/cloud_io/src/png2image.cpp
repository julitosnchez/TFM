#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main( int argc, char ** argv )
{
  ros::init(argc, argv, "png2Image");
  ros::NodeHandle nh;

  ros::Publisher pub2 = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10);
  ros::Publisher pub = nh.advertise<sensor_msgs::CameraInfo> ("/camera/rgb/camera_info", 10);

  cv::Mat image = cv::imread( "test1.png", CV_LOAD_IMAGE_COLOR );

  sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  unsigned int seq = 0;
 
  // Create a container for the data.
  sensor_msgs::CameraInfo info;
  
  info.header.frame_id = "camera_rgb_optical_frame";
  info.height = 480;
  info.width = 640;
  info.distortion_model = "plumb_bob";
  info.D.resize(5);
  info.D[0] = 0.0;
  info.D[1] = 0.0;
  info.D[2] = 0.0;
  info.D[3] = 0.0;
  info.D[4] = 0.0;

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

  im_msg->header.frame_id = "camera_rgb_optical_frame";

  ros::Rate rate( 10 );
  while( ros::ok() )  {
    info.header.seq = seq;
    info.header.stamp = ros::Time::now();
    im_msg->header.seq = seq;
    im_msg->header.stamp = info.header.stamp;

    pub.publish(info);
    pub2.publish(*im_msg);

    seq = seq + 1;
    rate.sleep();
  }
}




