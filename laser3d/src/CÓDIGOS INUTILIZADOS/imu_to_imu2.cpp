#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

sensor_msgs::Imu first_msg;
using namespace std;

void imuCallback(const sensor_msgs::Imu& msg) {
  static tf::TransformBroadcaster br;


  double r=0,p=-3.14159/2,y=0;

  tf::Quaternion q_orig;
  quaternionMsgToTF(msg.orientation,q_orig);

  tf::Quaternion q_rot = tf::Quaternion(tf::Vector3(1,0,0),p*2); //Rotating -90º in Y
  tf::Quaternion q_rot2 = tf::Quaternion(tf::Vector3(0,0,1),p);
 //Rotating -90º in Y
  tf::Quaternion q_new = q_orig*q_rot2*q_rot; //New orientation

  tf::Transform transform2( q_new, tf::Vector3(0.0, 0.0, 0.595)); //0.595 = 0.42 (que estaba antes) + 0.175 (imu_link -> laser)
  br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "laser"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_to_2");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu_laser_pub", 10, &imuCallback);

  ros::spin();
  return 0;
}


