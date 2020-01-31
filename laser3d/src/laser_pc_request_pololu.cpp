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
  ros::service::waitForService("assembled_point_cloud");
  ros::service::waitForService("assembled_point_cloud_kinect");

  ros::ServiceClient assemble_client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

	// Additional clients
  ros::ServiceClient assemble_client2 = nh.serviceClient<laser_assembler::AssembleScans>("assembled_point_cloud");
  ros::ServiceClient assemble_client3 = nh.serviceClient<laser_assembler::AssembleScans>("assembled_point_cloud_kinect");


  ros::Publisher pub_command = nh.advertise<std_msgs::UInt8>("/start_move", 10);
	ros::Duration(1).sleep();
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/assembled_cloud", 10);
	ros::Duration(1).sleep();
  ros::Publisher pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2", 10);


	ros::Duration(1).sleep();
  ros::Publisher pub_cloud_colored = nh.advertise<sensor_msgs::PointCloud>("/assembled_cloud_colored", 10);
	ros::Duration(1).sleep();
  ros::Publisher pub_cloud2_colored = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2_colored", 10);

	ros::Duration(1).sleep();
  ros::Publisher pub_cloud_kinect = nh.advertise<sensor_msgs::PointCloud>("/assembled_cloud_kinect", 10);
	ros::Duration(1).sleep();
  ros::Publisher pub_cloud2_kinect = nh.advertise<sensor_msgs::PointCloud2>("/assembled_cloud2_kinect", 10);



	ros::Duration(1).sleep();
	ros::Subscriber sub = nh.subscribe("/end_rotation", 1000, callback2);
	ros::Duration(1).sleep();
  ros::Subscriber sub2 = nh.subscribe("/imu_laser_pub", 1000, stateCallback);

 
  std_msgs::UInt8 motor_pos;


  laser_assembler::AssembleScans srv;
	laser_assembler::AssembleScans srv2;	
  laser_assembler::AssembleScans srv3;  
  
  // assemble from "NOW"
  srv.request.begin = ros::Time::now();
  srv2.request.begin = ros::Time::now();
  srv3.request.begin = ros::Time::now();
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
  srv3.request.end = ros::Time::now();
  srv2.request.end = ros::Time::now();
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
    ros::Duration(0.5).sleep();

    pcl::PointCloud<pcl::PointXYZ> save_cloud;
    pcl::fromROSMsg (pcloud2, save_cloud);
    pcl::io::savePCDFileASCII ("cloud.pcd", save_cloud); 
   
  } 
  else {
    printf("Assembling service call failed\n");
  }


  if (assemble_client2.call(srv2)) {
    std::cout << "Got cloud with " <<  srv2.response.cloud.points.size() << " points\n";
    pub_cloud_colored.publish(srv2.response.cloud);
    // if the node is going to finish, the publisher needs some time to send the data
    ros::Duration(0.5).sleep();
   
    ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points.", (int)srv2.response.cloud.points.size() );

    // conversion to PointCloud2
    sensor_msgs::PointCloud2 pcloud2;
    if (!sensor_msgs::convertPointCloudToPointCloud2(srv2.response.cloud, pcloud2)) {
      ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
      return -1;
    } 
    pub_cloud2_colored.publish(pcloud2);
    ros::Duration(0.5).sleep();
   
    // ADDED JUST BEFORE VTK STARTED FAILING
    pcl::PointCloud<pcl::PointXYZRGB> save_cloud;
    pcl::fromROSMsg (pcloud2, save_cloud);
    pcl::io::savePCDFileASCII ("colored_cloud.pcd", save_cloud); 
  } 
  else {
    printf("Assembling service call failed\n");
  }

if (assemble_client3.call(srv3)) {
    std::cout << "Got cloud with " <<  srv3.response.cloud.points.size() << " points\n";
    pub_cloud_kinect.publish(srv3.response.cloud);
    // if the node is going to finish, the publisher needs some time to send the data
    ros::Duration(0.5).sleep();
   
    ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points.", (int)srv2.response.cloud.points.size() );

    // conversion to PointCloud2
    sensor_msgs::PointCloud2 pcloud2;
    if (!sensor_msgs::convertPointCloudToPointCloud2(srv3.response.cloud, pcloud2)) {
      ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
      return -1;
    } 
    pub_cloud2_kinect.publish(pcloud2);
    ros::Duration(0.5).sleep();
   

  } 
  else {
    printf("Assembling service call failed\n");
  }






  return 0;
}

