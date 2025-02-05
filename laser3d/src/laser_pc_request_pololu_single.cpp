#include <ros/ros.h>

#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetSpeed.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

using namespace std;

bool end_rotation = false;
float position = 7.0;

void stateCallback(const sensor_msgs::Imu & msg) {
 
  position = msg.orientation_covariance[0];

}


void callback2(const std_msgs::Bool & end)
{
	end_rotation = end.data;
}

int getMovement()
{
	if(position >= -5 and position <= 5) return 1; //Initial case
  if(position >= 175 and position <= 185) return 2;
  return 3;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_pololu_controller");
  ros::NodeHandle nh;

// The laser_scan_assembler accumulates laser scans by listening to the appropriate topic 
//   and accumulating messages in a ring buffer of a specific size. 
// When the assemble_scans service is called,
//   the contents of the current buffer that fall between two times are converted into a single cloud and returned.

  ros::service::waitForService("assemble_scans");
  // Additional assembler for colored cloud and for Kinect

  ros::ServiceClient assemble_client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");


  ros::Publisher pub_command = nh.advertise<std_msgs::UInt8>("/start_move", 10);
  ros::Duration(1).sleep();
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/assembled_cloud", 10);
  ros::Duration(1).sleep();
  ros::Publisher pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2", 10);


  ros::Duration(1).sleep();
  ros::Subscriber sub = nh.subscribe("/end_rotation", 1000, callback2);
  ros::Duration(1).sleep();
  ros::Subscriber sub2 = nh.subscribe("/imu_laser_pub", 1000, stateCallback);

 
  std_msgs::UInt8 motor_pos;


  laser_assembler::AssembleScans srv;
  
  // assemble from "NOW"
  srv.request.begin = ros::Time::now();
  // command the motor to move
	unsigned int  giro = getMovement();
  while(giro == 3)
  { 
		giro = getMovement();
    ros::spinOnce();
  }

	motor_pos.data = giro;
  pub_command.publish(motor_pos);
  ros::Duration(2.0).sleep();


  while(not end_rotation) ros::spinOnce();
	end_rotation = false;

  // assemble untill "NOW"
  srv.request.end = ros::Time::now();

  if (assemble_client.call(srv)) {
    std::cout << "Got cloud with " <<  srv.response.cloud.points.size() << " points\n";
    pub_cloud.publish(srv.response.cloud);
    // if the node is going to finish, the publisher needs some time to send the data
    ros::Duration(0.5).sleep();
   
    ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points.", (int)srv.response.cloud.points.size() );

    // conversion to PointCloud2
    sensor_msgs::PointCloud2 pcloud2;
    if (!sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, pcloud2)) {
      ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
      return -1;
    } 
    pub_cloud2.publish(pcloud2);
    ros::Duration(1).sleep();
   
    // ADDED JUST BEFORE VTK STARTED FAILING
    //pcl::PointCloud<pcl::PointXYZRGB> save_cloud;
    //pcl_conversions::toPCL(pcloud2, save_cloud);
    //pcl::io::savePCDFileASCII ("test_pcd.pcd", save_cloud); 
  } 
  else {
    printf("Assembling service call failed\n");
  }


  return 0;
}

