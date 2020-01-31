#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_to_laser");
  ros::NodeHandle nh;

  ros::Rate rate(50);

  tf::TransformBroadcaster broadcaster;

  while(nh.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.5,-0.5,0.5,0.5), tf::Vector3(0,0,0.175)),
        ros::Time::now(),"imu_link", "laser"));
    rate.sleep();
  }
}
