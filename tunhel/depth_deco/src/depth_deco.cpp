#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>


int count=0;


void 
depth_cb (const sensor_msgs::Image msg)
{ count =count +1;
  std::cout << count << std::endl;
   

const int size = 4; 
std::ofstream file ("depth.txt");
   /*std::cout << msg.data.size() <<  "\t" <<  msg.step <<  "\t" <<  msg.height << "\t" <<  msg.encoding << "\t" <<  msg.is_bigendian << std::endl;
   for (int i=0; i<msg.height ;i++){
    for (int j=0; i<msg.width ;i++) 
      { 
	outputfile << (int)msg.data [i][j];
      }
	outputfile << endl;
   } 
    outputfile.close();*/

uint32_t pixel = 0;
if (count==48){
  for (int i=0; i < 4; i++)
     {  uint32_t byte= msg.data[i] << (8*i); 
	std::cout << (unsigned int)msg.data[i] << std::endl;	
        pixel= pixel + byte;
    }
    std::cout << pixel << std::endl;
    float *num = reinterpret_cast<float*>(&pixel);
    std::cout << *num << std::endl;

     file << *num << std::endl;

      file.close();
}
/*
float pixel;
if (count==1){
  for (int i=0; i < 4; i++)
     {float byte= msg.data[i] << 8*i; 	
       pixel= pixel + byte;
    }
  std::cout << pixel << std::endl;
}*/
	
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "capture");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("depth_input", 1, depth_cb);

   // Spin
  ros::spin ();
}
