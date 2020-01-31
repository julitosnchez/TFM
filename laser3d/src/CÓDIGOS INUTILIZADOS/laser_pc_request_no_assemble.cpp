#include <ros/ros.h>

#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetSpeed.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_motor_controller");
  ros::NodeHandle nh;

// The laser_scan_assembler accumulates laser scans by listening to the appropriate topic 
//   and accumulating messages in a ring buffer of a specific size. 
// When the assemble_scans service is called,
//   the contents of the current buffer that fall between two times are converted into a single cloud and returned.


  ros::Publisher pub_command = nh.advertise<std_msgs::Float64>("/laser_controller/command", 10);
 
  std_msgs::Float64 motor_pos; 
  dynamixel_msgs::JointStateConstPtr sharedPtr;

  // Check actual position and set next position command
  do{
    sharedPtr = ros::topic::waitForMessage<dynamixel_msgs::JointState>("/laser_controller/state", ros::Duration(0.2));	
  }while(sharedPtr == NULL);

  float offset = 0.6;
  if(sharedPtr->current_pos > 3.0) 
    motor_pos.data = 0.0+offset;
  else
    motor_pos.data = 3.1516+offset;

  // Set the speed of the motor
  ros::service::waitForService("/laser_controller/set_speed");
  ros::ServiceClient speed_client = nh.serviceClient<dynamixel_controllers::SetSpeed>("/laser_controller/set_speed");

  dynamixel_controllers::SetSpeed srv_req;
  nh.getParam("/laser_controller/joint_speed", srv_req.request.speed);
 
  if (speed_client.call(srv_req)) {
    std::cout << "Speed set in the motor: " << srv_req.request.speed << std::endl;
  } else {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }


  pub_command.publish(motor_pos);

  // check the motor has finished
  do{
    sharedPtr = ros::topic::waitForMessage<dynamixel_msgs::JointState>("/laser_controller/state", ros::Duration(0.3));	
  }while( fabs(sharedPtr->current_pos - motor_pos.data) > 0.01); // 0.5 degrees error accepted

  // assemble untill "NOW"
  return 0;
}

