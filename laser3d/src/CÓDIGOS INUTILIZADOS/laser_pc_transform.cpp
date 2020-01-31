#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Float64.h>

ros::Publisher pub;

using namespace std;


void printTf(tf::Transform tf) {
    tf::Vector3 tfVec;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    tfVec = tf.getOrigin();
    cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
    tfR = tf.getBasis();
    cout<<"orientation of child frame w/rt reference frame: "<<endl;
    tfVec = tfR.getRow(0);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
    tfVec = tfR.getRow(1);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;    
    tfVec = tfR.getRow(2);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl; 
    quat = tf.getRotation();
    cout<<"quaternion: " <<quat.x()<<", "<<quat.y()<<", "
            <<quat.z()<<", "<<quat.w()<<endl;   
}


void cloudCallback(const sensor_msgs::PointCloud2& msg) {

//	ros::Duration(3).sleep();
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_pc_transformer");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/velodyne_points", 10, &cloudCallback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/TransformedCloudToBase", 1);

  // Spin
  ros::spin ();

  return 0;
}



