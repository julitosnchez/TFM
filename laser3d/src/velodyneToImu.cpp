#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamixel_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub;

void stateCallback(const sensor_msgs::PointCloud2 & msg) {


  std::cout << "HE ENTRADO" << std::endl;
  sensor_msgs::PointCloud2 msg_imu;

  msg_imu = msg;

  msg_imu.header.frame_id = "base_link";

  pub.publish(msg_imu);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "velodyneToImu");
  ros::NodeHandle nh;


  pub = nh.advertise<sensor_msgs::PointCloud2>("new_topic",10);
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 10,  stateCallback);

  ros::spin();
  return 0;
}
