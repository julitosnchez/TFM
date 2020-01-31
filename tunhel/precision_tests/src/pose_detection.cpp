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
#include <ar_track_alvar_msgs/AlvarMarkers.h>



int main (int argc, char** argv)
{

        // Initialize ROS
        ros::init (argc, argv, "tagpose_reader");
        ros::NodeHandle node;
        
        tf::TransformListener listener;

          ros::Rate rate(10.0);
         // while (node.ok()){
              tf::StampedTransform transform1;
              tf::StampedTransform transform2;
              tf::StampedTransform transform3;
              tf::StampedTransform transform4;

              try{
                //ros::Time now = ros::Time::now();
                //listener.waitForTransform("/cylinder_tag", "/tag1", ros::Time(0), ros::Duration(0.5));
                //listener.lookupTransform("/cylinder_tag", "/tag1", ros::Time(0), transform);
                  listener.waitForTransform("/cylinder_tag", "/tag1", ros::Time(0), ros::Duration(0.5));
                  listener.lookupTransform("/cylinder_tag", "/tag1", ros::Time(0), transform1);

                  listener.waitForTransform("/cylinder_tag", "/tag2", ros::Time(0), ros::Duration(0.5));
                  listener.lookupTransform("/cylinder_tag", "/tag2", ros::Time(0), transform2);

                  listener.waitForTransform("/cylinder_tag", "/tag3", ros::Time(0), ros::Duration(0.5));
                  listener.lookupTransform("/cylinder_tag", "/tag3", ros::Time(0), transform3);

                  listener.waitForTransform("/cylinder_tag", "/tag0", ros::Time(0), ros::Duration(0.5));
                  listener.lookupTransform("/cylinder_tag", "/tag0", ros::Time(0), transform4);
              }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
              }

         tf::Quaternion q1= transform1.getRotation();
         tf::Quaternion q2= transform2.getRotation();
         tf::Quaternion q3= transform3.getRotation();
         tf::Quaternion q4= transform4.getRotation();


         //std::cout << "Translation y:" << transform.getOrigin().y() << std::endl;
         //std::cout << "Rotation: x=" << q.x() <<",y=" << q.y() <<",z=" << q.z()<< ",w=" << q.w() << std::endl;

         double rot_angle=q1.getAngleShortestPath();
         tf::Vector3 rot_axis1=q1.getAxis();
         tf::Matrix3x3 R1=transform1.getBasis();
         tf::Vector3 No1=R1.getColumn(2);
         tf::Vector3 N1(0,0,1);
         double ang_norm1=No1.angle(N1);
         tf::Vector3 pvect1=No1.cross(N1).normalized();
         //Comprobación
         double pesc1=acos(No1.dot(N1));

         double rot_angle2=q2.getAngleShortestPath();
         tf::Vector3 rot_axis2=q2.getAxis();
         tf::Matrix3x3 R2=transform2.getBasis();
         tf::Vector3 No2=R2.getColumn(2);
         tf::Vector3 N2(0,0,1);
         double ang_norm2=No2.angle(N2);
         tf::Vector3 pvect2=No2.cross(N2).normalized();

         double rot_angle3=q3.getAngleShortestPath();
         tf::Vector3 rot_axis3=q3.getAxis();
         tf::Matrix3x3 R3=transform3.getBasis();
         tf::Vector3 No3=R3.getColumn(2);
         tf::Vector3 N3(0,0,1);
         double ang_norm3=No3.angle(N3);
         tf::Vector3 pvect3=No3.cross(N3).normalized();

         double rot_angle4=q4.getAngleShortestPath();
         tf::Vector3 rot_axis4=q4.getAxis();
         tf::Matrix3x3 R4=transform4.getBasis();
         tf::Vector3 No4=R4.getColumn(2);
         tf::Vector3 N4(0,0,1);
         double ang_norm4=No4.angle(N4);
         tf::Vector3 pvect4=No4.cross(N4).normalized();

         //std::cout << "Axis Global: [" << rot_axis[0] << "," << rot_axis[1] << "," << rot_axis[2] << "]" << std::endl;
         //std::cout << "Angle Global:" << rot_angle*180/3.14159 << " deg" << std::endl;

         std::cout << "Normal Rot. Axis: [" << pvect1[0] << "," << pvect1[1] << "," << pvect1[2] << "]" << std::endl;
         std::cout << "Normal Rot. Angle:" << ang_norm1*180/3.14159 << "deg" << std::endl;
         //std::cout << "Ang esc:" << pesc*180/3.14159 << "deg" << std::endl;

         std::cout << "Normal Rot. Axis 2: [" << pvect2[0] << "," << pvect2[1] << "," << pvect2[2] << "]" << std::endl;
         std::cout << "Normal Rot. Angle 2:" << ang_norm2*180/3.14159 << "deg" << std::endl;

         std::cout << "Normal Rot. Axis 3: [" << pvect3[0] << "," << pvect3[1] << "," << pvect3[2] << "]" << std::endl;
         std::cout << "Normal Rot. Angle 3:" << ang_norm3*180/3.14159 << "deg" << std::endl;

         std::cout << "Normal Rot. Axis 4: [" << pvect4[0] << "," << pvect4[1] << "," << pvect4[2] << "]" << std::endl;
         std::cout << "Normal Rot. Angle 4:" << ang_norm4*180/3.14159 << "deg" << std::endl;
       //  }

         //Version David
         /*tf::StampedTransform transform1;
         tf::StampedTransform transform2;
         listener.waitForTransform("/tag1", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(0.5));
         listener.lookupTransform("/tag1", "/camera_rgb_optical_frame", ros::Time(0), transform1);

         listener.waitForTransform("/cylinder_tag", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(0.5));
         listener.lookupTransform("/cylinder_tag", "/camera_rgb_optical_frame", ros::Time(0), transform2);

         tf::Quaternion q1= transform1.getRotation();
         tf::Quaternion q2= transform2.getRotation();
         tf::Matrix3x3 R1=transform1.getBasis();
         tf::Vector3 V1=R1.getColumn(2);
         tf::Matrix3x3 R2=transform2.getBasis();
         tf::Vector3 V2=R2.getColumn(2);

         tf::Vector3 pvect2=V1.cross(V2);
         double pesc2=acos(V1.dot(V2));

         std::cout << "P.Vect2: [" << pvect2[0] << "," << pvect2[1] << "," << pvect2[2] << "]" << std::endl;
         std::cout << "Ang esc2:" << pesc2*180/3.14159 << "deg" << std::endl;*/




         return 0;
};
