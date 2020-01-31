
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class MyClass
{
public:
  ros::NodeHandle nh;

  ros::Publisher pub;

  MyClass( ) 
  {
    pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_rect_new", 100);
  }

  virtual ~MyClass()
  {

  }

  void depth_cb (const sensor_msgs::Image msg) { 
     sensor_msgs::Image new_msg;
     new_msg = msg;
     new_msg.header.frame_id = "camera_rgb_optical_frame";
     pub.publish(new_msg);
  }
};



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "header_change");
  MyClass pub_class;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = pub_class.nh.subscribe ("/camera/depth/image_rect", 5,  &MyClass::depth_cb, &pub_class);

   // Spin
  ros::spin ();
}
