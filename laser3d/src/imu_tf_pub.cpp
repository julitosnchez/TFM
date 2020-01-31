#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

sensor_msgs::Imu first_msg;
unsigned int first = 0;
using namespace std;

void imuCallback(const sensor_msgs::Imu& msg) {
  static tf::TransformBroadcaster br;
  //if(first < 1) {
   first_msg = msg;
  //}

   /*
   double q2sqr = first_msg.orientation.y * first_msg.orientation.y;
   //double t0 = -2.0 * (q2sqr + data.q3 * data.q3) + 1.0;
   //double t1 = +2.0 * (data.q1 * data.q2 + data.q0 * data.q3);

   //ROLL (X-Axis rotation)
   double t3 = +2.0 * (first_msg.orientation.y * first_msg.orientation.z + first_msg.orientation.w * first_msg.orientation.x);
   double t4 = -2.0 * (first_msg.orientation.x * first_msg.orientation.x + q2sqr) + 1.0;
   double roll = atan2(t3, t4);

   //PITCH (Y-Axis rotation)
   double t2 = -2.0 * (first_msg.orientation.x * first_msg.orientation.z - first_msg.orientation.w * first_msg.orientation.y);
   t2 = t2 > 1.0 ? 1.0 : t2;
   t2 = t2 < -1.0 ? -1.0 : t2;
   double pitch = asin(t2);

   // yaw = atan2(t1, t0);
  double pi = 3.1415926535897;
  cout << "PITCH: " << (pitch*180.0)/(pi*1.0) << " " << "ROLL: " << (roll*180.0)/(pi*1.0) << endl;
  //tf::Transform transform1(tf::Quaternion(first_msg.orientation.x, first_msg.orientation.y, first_msg.orientation.z, first_msg.orientation.w), tf::Vector3(0.0, 0.0, 0.5));
  //br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "imu_link_bad"));
  //tf::Transform transform2(tf::Quaternion(first_msg.orientation.x, first_msg.orientation.y, first_msg.orientation.z, first_msg.orientation.w), tf::Vector3(0.0, 0.0, 0.5));
  //br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "imu_link"));
  tf::Quaternion q = tf::createQuaternionFromRPY(pitch, roll, 0);
  tf::Transform transform2( q, tf::Vector3(0.0, 0.0, 0.42));
  br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "imu_link")); */

  double r=3.14159/2,p=0,y=3.14159/2;

  tf::Quaternion q_orig;
  quaternionMsgToTF(msg.orientation,q_orig);

//  tf::Quaternion q_rot = tf::createQuaternionFromRPY(0,0,0); //Rotating -90º
  tf::Quaternion q_rot = tf::createQuaternionFromRPY(0,0,y); //Rotating -90º
  tf::Quaternion q_new = q_orig;//*q_rot; //New orientatio0r
  tf::Transform transform2( q_new, tf::Vector3(0.0, 0.0, 0.42));
  br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "imu_link"));


  first = first + 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_tf_pub");
  ros::NodeHandle nh;

//  ros::Subscriber sub = nh.subscribe("imu_laser_pub", 10, &imuCallback);

  ros::spin();
  return 0;
}


