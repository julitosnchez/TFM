#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>


using namespace std;
using namespace sensor_msgs;

double pi = 3.14159265359;
ros::Publisher publish_cloud_time;
ros::Publisher publish_cloud_time_colored;
ros::Publisher publish_cloud_time_kinect;
double ang = 0; 
int iterador = 0;


double degree2rad(const double & d)
{
	return pi * d / 180;
} 

void stateCallback(const sensor_msgs::Imu & msg) {
	//Wait for PointCloud and then publishing it in "Cloud_Timestamp" topic
	boost::shared_ptr<sensor_msgs::PointCloud const> sharedPtr;
  sensor_msgs::PointCloud msg_cloud;

  //-------------- Pruebas 21/11/19 --------------------------
	boost::shared_ptr<sensor_msgs::PointCloud const> sharedPtr2;
	sensor_msgs::PointCloud msg_cloud2;
	boost::shared_ptr<sensor_msgs::PointCloud const> sharedPtr3;
	sensor_msgs::PointCloud msg_cloud3;
	//---------------------------------------------------------

  static tf::TransformBroadcaster br;
  float position = msg.orientation_covariance[0];
  
//	ros::Time actual_time = ros::Time::now();
//	sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud>("velodyne_points_2", ros::Duration(0.35));

//  //-------------- Pruebas 21/11/19 -------------------------------------------------------------------------------
//	ros::Time actual_time2 = ros::Time::now();
//	sharedPtr2 = ros::topic::waitForMessage<sensor_msgs::PointCloud>("velodyne_colored_points_2", ros::Duration(0.6));
	ros::Time actual_time3 = ros::Time::now();
//	if(iterador++ % 3 == 0) // Cada 5 iteraciones
	sharedPtr3 = ros::topic::waitForMessage<sensor_msgs::PointCloud>("/kinect2/hd/points2", ros::Duration(0.5));
	//----------------------------------------------------------------------------------------------------------------

//		if(sharedPtr != NULL) {
//			msg_cloud = *sharedPtr;
//  		msg_cloud.header.stamp = actual_time;
//		}

//		if(sharedPtr2 != NULL) {
//			msg_cloud2 = *sharedPtr2;
//  		msg_cloud2.header.stamp = actual_time2;
//		}

		if(sharedPtr3 != NULL) {
			msg_cloud3 = *sharedPtr3;
  		msg_cloud3.header.stamp = actual_time3;
			publish_cloud_time_kinect.publish(msg_cloud3);
		}


  double r=3.14159/2,p=0,y=3.14159/2;

//  Quaternion (const Vector3 &axis, const tfScalar &angle)
  
//  tf::Vector3 vector(0.0027021,-0.0110964,-0.0489239); 
  tf::Vector3 vector(2.5543763e-04,-2.6544603e-04,-0.0462945);
	double angulo = ((position*-ang)/180.0);

//  cout << "ANGULO " << angulo << endl;

	tf::Quaternion quat_offset(vector,angulo);

  tf::Quaternion q_imu;


  quaternionMsgToTF(msg.orientation,q_imu);

	tf::Matrix3x3 m(q_imu);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);



  tf::Quaternion q_orig;//Quaternion (const tfScalar &yaw, const tfScalar &pitch, const tfScalar &roll) __attribute__((deprecated))

  q_orig.setRPY(roll,pitch,yaw); //Este es el que funciona con la INFORMACION de la IMU
//  q_orig.setRPY(0,0,0); // Sin informacion de la IMU -- Proposito: Orientacion prismas en la mina
 

  tf::Quaternion q_rot = tf::createQuaternionFromRPY(0,y,2*y); //Rotating -90º

	// *** NOTA *** : Ha cambiado la configuración del laser - imu. He tenido que rotarla 180º para cambiar el sentido ya que cuando inclinaba a un lado, la nube lo hacia al contrario....
  tf::Quaternion q_rot_2 = tf::createQuaternionFromRPY(2*y,0,0); //Rotating -90º  


//	// Publishing into "Cloud_Timestamp"
//  publish_cloud_time.publish(msg_cloud);
//	publish_cloud_time_colored.publish(msg_cloud2);



  // Transformation Base_Link - Imu_Link
  tf::Quaternion q_new = q_orig*q_rot*quat_offset*q_rot_2;
  tf::Transform transform2( q_new, tf::Vector3(0.0, 0.0, 0.2)); //0.69
  br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "imu_link"));

 
	// Transformation Imu_Link - Velodyne
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
	q.setRPY(degree2rad(-position), 0,0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu_link", "velodyne"));



	// --------------------- Para rotar tf de la KINECT ------------------------------------------------------------------
	tf::Transform transform_kinect;
	transform_kinect.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion q_kinect;

	q_kinect.setRPY(0,degree2rad(position),0); 
	transform_kinect.setRotation(q_kinect);
	br.sendTransform(tf::StampedTransform(transform_kinect, ros::Time::now(), "kinect_imu", "kinect2_ir_optical_frame"));

//	cout << "Iterador: " << iterador << endl;
//  ros::Duration(5).sleep();

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_tf_pos");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu_laser_pub", 10, &stateCallback);
  publish_cloud_time = nh.advertise<sensor_msgs::PointCloud>("/cloud_timestamp", 10);
  publish_cloud_time_colored = nh.advertise<sensor_msgs::PointCloud>("/cloud_timestamp_colored", 10);
  publish_cloud_time_kinect = nh.advertise<sensor_msgs::PointCloud>("/cloud_timestamp_kinect", 10);


  ros::spin();
  return 0;
}
