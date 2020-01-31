#include <iostream>
//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

ros::Publisher pub_KbStatus;
ros::Publisher pub_KbKey;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Capture_key_event");
  ros::NodeHandle nh;
  // Create a ROS publisher
  pub_KbStatus = nh.advertise<std_msgs::UInt8> ("Keyboard_status", 1);
  // Create a ROS publisher
  pub_KbKey = nh.advertise<std_msgs::Int8> ("Keyboard_key", 1);

  std_msgs::UInt8 status_msg;
  std_msgs::Int8 keycode_msg;

   char c = 'a';
  //pub_KbStatus.publish (status_msg);
  while ( c != 'e'){
   status_msg.data=false;
   std::cout << "Press 'q' to select a hole to analyze or 's' to select a region" << std::endl;

   std::cin >> c;
   //std::cout << c << std::endl;
   if ('q' == c) {
       status_msg.data=1;
       //std::cout << "si" << std::endl;
   } else if ('s' == c) {
			status_msg.data=2;
	 } else {
			status_msg.data = 3;
	 }
   keycode_msg.data=c;
   pub_KbStatus.publish (status_msg);
   pub_KbKey.publish (keycode_msg);
  }

}
