#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

class CameraRGBSynch 
{
  private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_image_;
    ros::Publisher pub_info_;
    ros::Publisher pub_rgb_;
    sensor_msgs::CameraInfo info;

    unsigned int seq;

  public:
/*--------------------------------------------------------------------
 * NodeExample()
 * Constructor.
 *------------------------------------------------------------------*/
CameraRGBSynch () 
{
ROS_INFO("Synchro1");
  // Subscribe to the cloud topic using both the old message format and the new
  sub_image_ = nh_.subscribe ("/kinect2/hd/image_color", 10, &CameraRGBSynch::imageCb, this);
  ROS_INFO("Synchro2");
  pub_info_ = nh_.advertise<sensor_msgs::CameraInfo> ("/my_info", 10);
  pub_rgb_ = nh_.advertise<sensor_msgs::Image> ("/my_color", 10);

  seq = 0;

//////////////////////////////////////
  info.header.seq = 0;
  info.header.frame_id = "/camera_rgb_frame";
  info.height = 1080;
  info.width = 1920;
  info.distortion_model = "plumb_bob";
  info.D.resize(5);
  for (unsigned int t = 0; t < 5; t++) {
    info.D[t] = 0.0;
  }
  info.K[0] = 1081.3720703125;
  info.K[1] = 0.0;
  info.K[2] = 959.5;
  info.K[3] = 0.0;
  info.K[4] = 1081.3720703125;
  info.K[5] = 539.5;
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

  info.P[0] = 1081.3720703125;
  info.P[1] = 0.0;
  info.P[2] = 959.5;
  info.P[3] = 0.0;
  info.P[4] = 0.0;
  info.P[5] = 1081.3720703125;
  info.P[6] = 539.5;
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
}

/*--------------------------------------------------------------------
 * ~CameraRGBSynch()
 * Destructor
 *------------------------------------------------------------------*/
~CameraRGBSynch()
{ } 

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
ROS_INFO("Synchro");

  sensor_msgs::Image output_msg;
  output_msg = *msg;

  output_msg.header.seq = seq;	
  output_msg.header.stamp = ros::Time::now();

  info.header.seq = seq;	
  info.header.stamp = output_msg.header.stamp;
    
  pub_info_.publish(info);
  pub_rgb_.publish(output_msg);

  seq = seq + 1;
}

};

int main (int argc, char** argv) {

   // Initialize ROS
  ros::init (argc, argv, "camera_info_sync");

  CameraRGBSynch synchron;

  ros::spin();

  return (0);
}


