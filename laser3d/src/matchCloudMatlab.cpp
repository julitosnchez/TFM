#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamixel_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <time.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "matchCloudMatlab");
  ros::NodeHandle nh;


  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/matchCloud",10);

	ros::Duration(1).sleep();
  std_msgs::Bool b;

  b.data = 1;

  pub.publish(b);

  std::cout << "Publicando..." << std::endl;
	ros::Duration(1).sleep();
  return 0;
}
