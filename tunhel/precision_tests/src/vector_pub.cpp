#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <limits>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


int main (int argc, char** argv)
{

        // Initialize ROS
        ros::init (argc, argv, "vector_publisher");
        ros::NodeHandle node;
        
        ros::Publisher vector_pub = node.advertise<visualization_msgs::Marker>( "Vector_Marker", 1 );
        ros::Publisher vector_pub1 = node.advertise<visualization_msgs::Marker>( "Vector_Marker1", 1 );
	ros::Publisher cyl_pub = node.advertise<visualization_msgs::Marker>( "Cyl_Marker", 1 );
	ros::Publisher cyl_pub1 = node.advertise<visualization_msgs::Marker>( "Cyl_Marker1", 1 );
	
	visualization_msgs::Marker est_arrow;
	visualization_msgs::Marker est_arrow1;
	visualization_msgs::Marker est_cyl;
	visualization_msgs::Marker est_cyl1;

	tf::Vector3 EstPos1(0.107989, 0.025968, 0.697476);
        tf::Vector3 N1(0.163197, 0.044724, 0.923171);
	//N1=-N1;
	double radio1=1.420171; //in cms

	tf::Vector3 EstPos2(0.023001, 0.008137, 0.711583);
        tf::Vector3 N2(-0.034925, -0.092557, 0.957157);
	double radio2=2.500000;

	est_arrow.ns = "Estimated_Vector1";
	est_arrow.type = visualization_msgs::Marker::ARROW;
	est_arrow.action = visualization_msgs::Marker::ADD;
	est_arrow.scale.x = 0.001;    // height
	est_arrow.scale.y = 0.005;        
	est_arrow.scale.z = 0.005;

	est_arrow.color.r = 1.0f;
	est_arrow.color.g = 0.0f;
	est_arrow.color.b = 0.0f;
	est_arrow.color.a = 1.0;

	est_arrow.points.resize(2);
	est_arrow.points[0].x=EstPos1[0];
	est_arrow.points[0].y=EstPos1[1];
	est_arrow.points[0].z=EstPos1[2];
	est_arrow.points[1].x=EstPos1[0]+N1[0];
	est_arrow.points[1].y=EstPos1[1]+N1[1];
	est_arrow.points[1].z=EstPos1[2]+N1[2];

	est_arrow.lifetime = ros::Duration();
       	est_arrow.header.frame_id = "camera_rgb_frame";

	est_arrow1.ns = "Estimated_Vector2";
	est_arrow1.type = visualization_msgs::Marker::ARROW;
	est_arrow1.action = visualization_msgs::Marker::ADD;
	est_arrow1.scale.x = 0.001;    // height
	est_arrow1.scale.y = 0.005;        
	est_arrow1.scale.z = 0.005;

	est_arrow1.color.r = 1.0f;
	est_arrow1.color.g = 0.0f;
	est_arrow1.color.b = 0.0f;
	est_arrow1.color.a = 1.0;

	est_arrow1.points.resize(2);
	est_arrow1.points[0].x=EstPos2[0];
	est_arrow1.points[0].y=EstPos2[1];
	est_arrow1.points[0].z=EstPos2[2];
	est_arrow1.points[1].x=EstPos2[0]+N2[0];
	est_arrow1.points[1].y=EstPos2[1]+N2[1];
	est_arrow1.points[1].z=EstPos2[2]+N2[2];

	est_arrow1.lifetime = ros::Duration();
       	est_arrow1.header.frame_id = "camera_rgb_frame";

	est_cyl.ns = "Estimated_Cylinder1";
	est_cyl.id = 0;
	est_cyl.type = visualization_msgs::Marker::CYLINDER;
	est_cyl.action = visualization_msgs::Marker::ADD;
	est_cyl.scale.x = radio1/100;    // height
	est_cyl.scale.y = radio1/100;        
	est_cyl.scale.z = 0.3;

	est_cyl.color.r = 0.0f;
	est_cyl.color.g = 1.0f;
	est_cyl.color.b = 0.0f;
	est_cyl.color.a = 1.0;

	est_cyl.pose.position.x = EstPos1[0];
        est_cyl.pose.position.y = EstPos1[1];
        est_cyl.pose.position.z = EstPos1[2];

	tf::Matrix3x3 rot(N1[0], N1[0], 0, 0, N1[1], N1[1], N1[2], 0, N1[2]);
	tf::Quaternion q;
	rot.getRotation(q);

	

	est_cyl.pose.orientation.x = q.x();
	est_cyl.pose.orientation.y = q.y();
        est_cyl.pose.orientation.z = q.z();
        est_cyl.pose.orientation.w = q.w();

	est_cyl.lifetime = ros::Duration();
       	est_cyl.header.frame_id = "camera_rgb_frame";

      	while(ros::ok())  {
		est_arrow.header.stamp = ros::Time::now();
		est_arrow1.header.stamp = ros::Time::now();
	        est_cyl.header.stamp = ros::Time::now();
		vector_pub.publish(est_arrow);
		vector_pub1.publish(est_arrow1);
		cyl_pub.publish(est_cyl);
		ros::Duration(0.5).sleep();

	}
        //ros::spin ();
};

