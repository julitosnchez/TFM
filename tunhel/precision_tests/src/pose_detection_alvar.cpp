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
        ros::init (argc, argv, "tagpose_reader");
        ros::NodeHandle node;
        
        tf::TransformListener listener;
        tf::TransformListener listener1;
	

        int count=0;
        double angsum1=0, angsum2=0, angsum3=0, sum1x=0,sum1y=0,sum1z=0, sum2x=0,sum2y=0,sum2z=0, sum3x=0,sum3y=0,sum3z=0,angmin1=100000, angmin2=100000, angmin3=100000,angmax1=0, angant1=0, angmax2=0, angant2=0, angmax3=0, angant3=0;
        tf::Vector3 pvect1;
        double pesc1;
        tf::Vector3 translation1;
        tf::Vector3 translation2;
        tf::Vector3 translation3;

        ros::Rate rate(10.0);

        //Entrada datos estimación
	tf::Vector3 EstPos1(0.106875,-0.00471642,0.615898);
        tf::Vector3 N1(0.163197, 0.044724, 0.923171);
	double radio1=1.420171; //in cms

	/*tf::Vector3 EstPos2(0.023001, 0.008137, 0.711583);
        tf::Vector3 N2(-0.034925, -0.092557, 0.957157);
	double radio2=2.500000;*/
	

	while (node.ok()){
              tf::StampedTransform transform1;
	      tf::StampedTransform transform2;
	      tf::StampedTransform transform3;
	      
              try{
                ros::Time now = ros::Time::now();
                  listener.waitForTransform("/filter_marker174", "/camera_rgb_frame", now, ros::Duration(1.5));
                  listener.lookupTransform("/filter_marker174", "/camera_rgb_frame", ros::Time(0), transform1);
		  /*listener.waitForTransform("/filter_marker10", "/camera_rgb_frame", ros::Time(0), ros::Duration(0.5));
                  listener.lookupTransform("/filter_marker10", "/camera_rgb_frame", ros::Time(0), transform2);
		  listener.waitForTransform("/filter_marker10", "/filter_marker174", ros::Time(0), ros::Duration(0.5));
                  listener.lookupTransform("/filter_marker10", "/filter_marker174", ros::Time(0), transform3);*/
                  count=count +1;
                  if (count==10) break;
                  std::cout << count << std::endl;
                 }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
              }

         tf::Quaternion q1= transform1.getRotation();
	 //tf::Quaternion q2= transform2.getRotation();

         translation1=transform1.getOrigin();
         //translation2=transform2.getOrigin();
         
         sum1x=sum1x+translation1.x();
         sum1y=sum1y+translation1.y();
         sum1z=sum1z+translation1.z();
         //sum2x=sum2x+translation2.x();
         //sum2y=sum2y+translation2.y();
         //sum2z=sum2z+translation2.z();
   	 

         double rot_angle=q1.getAngleShortestPath();

         tf::Vector3 rot_axis1=q1.getAxis();
         tf::Matrix3x3 R1=transform1.getBasis();
         tf::Vector3 No1=R1.getColumn(2); //normal tag expresado respecto al sistema camara
	 //tf::Matrix3x3 R2=transform2.getBasis();
         //tf::Vector3 No2=R2.getColumn(2); //normal tag expresado respecto al sistema camara
	 	 
	 //std::cout << "Normal No1" << "[" << No1[0] << "," << No1[1] << "," << No1[2] << "]" << "deg" << std::endl;	 
         No1=-1*No1; // vector opuesto a la normal, entrante
	 //No2=-1*No2;
         double ang1=No1.angle(N1);
	 //double ang2=No2.angle(N2);
 	 //double ang3=No1.angle(No2);
	

         if (ang1<angmin1) {angmin1=ang1;}
	 if (ang1>angmax1) {angmax1=ang1;}

         //if (ang2<angmin2) {angmin2=ang2;}
	 //if (ang2>angmax2) {angmax2=ang2;}

         //if (ang3<angmin3) {angmin3=ang3;}
	 //if (ang3>angmax3) {angmax3=ang3;}

         angsum1=angsum1+ang1;
	 //angsum2=angsum2+ang2;
	 //angsum3=angsum3+ang3;
         pvect1=No1.cross(N1).normalized();
         //Comprobación
         pesc1=acos(No1.dot(N1));
         }


         //std::cout << "Axis Global: [" << rot_axis[0] << "," << rot_axis[1] << "," << rot_axis[2] << "]" << std::endl;
         //std::cout << "Angle Global:" << rot_angle*180/3.14159 << " deg" << std::endl;
         double ang_norm1=angsum1/(count-1);
 	 //double ang_norm2=angsum2/(count-1);
	 //double ang_norm3=angsum3/(count-1);

         tf::Vector3 Pos1(sum1x/(count-1), sum1y/(count-1), sum1z/(count-1));
	 //tf::Vector3 Pos2(sum2x/(count-1), sum2y/(count-1), sum2z/(count-1));
	 
         float erpos1=EstPos1.length()-Pos1.length();
         //float erpos2=EstPos2.length()-Pos2.length();
	 
         //std::cout << "Normal Rot. Axis: [" << pvect1[0] << "," << pvect1[1] << "," << pvect1[2] << "]" << std::endl;
         //std::cout << "Normal Rot. Angle:" << ang_norm1*180/3.14159 << "deg" << std::endl;
	 std::cout << "Hole 1" << std::endl;
         std::cout << "Error Angulo Medio:" << ang_norm1*180/3.14159 << " deg" << std::endl;
	 std::cout << "Error Angulo Maximo:" << angmax1*180/3.14159 << " deg" << std::endl;
	 std::cout << "Error Angulo Minimo:" << angmin1*180/3.14159 << " deg" << std::endl;
         std::cout << "Error Posición:" << fabs(erpos1*100) << " cms" << std::endl;
         std::cout << "Error Radio:" << fabs(radio1-3.5/2) << " cms" << std::endl;
	 std::cout << "----------------------------------" << std::endl << std::endl;
	 /*std::cout << "Hole 2" << std::endl;
         std::cout << "Error Angulo Medio:" << ang_norm2*180/3.14159 << " deg" << std::endl;
	 std::cout << "Error Angulo Maximo:" << angmax2*180/3.14159 << " deg" << std::endl;
	 std::cout << "Error Angulo Minimo:" << angmin2*180/3.14159 << " deg" << std::endl;
         std::cout << "Error Posición:" << fabs(erpos2*100) << " cms" << std::endl;
         std::cout << "Error Radio:" << fabs(radio2-3.5/2) << " cms" << std::endl;
	 std::cout << "----------------------------------" << std::endl << std::endl;
	 std::cout << "Comprobación" << std::endl;
	 std::cout << "Error Angulo Medio:" << ang_norm3*180/3.14159 << " deg" << std::endl;
	 std::cout << "Error Angulo Maximo:" << angmax3*180/3.14159 << " deg" << std::endl;
	 std::cout << "Error Angulo Minimo:" << angmin3*180/3.14159 << " deg" << std::endl;*/
         

         //std::cout << "Translation: [" << x << "," << y << "," << z << "]" << std::endl;

         return 0;
};





