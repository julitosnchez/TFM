#include <iostream>

#include <cstdlib>
#include <cstdio>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <fstream>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

#include <but_calibration_camera_velodyne/aux_class.h>
#include <string>
#include <vector> 

using namespace std;
using namespace cv;
using namespace pcl;
using namespace message_filters;
using namespace but_calibration_camera_velodyne;
using namespace io;

ros::Publisher pub;
int id = 0;

struct label {

	vector<PointXYZ> p;

	int r;
	int g;
	int b;
	string topic;
};

vector<label> labels;

PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

// ----------------------------------------------------------------------------------------------------------------------------------------
// Method for reading labels and RGB associated to every object that the neuronal net has learnt to classify.

void readLabels(const char * filename) {

	string line;
	ifstream myfile;
	myfile.open(filename);

  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {

			string r1(1,line[0]);string r2(1,line[1]);string r3(1,line[2]); string r; r = r1+r2+r3;
			string g1(1,line[4]);string g2(1,line[5]);string g3(1,line[6]); string g; g = g1+g2+g3;
			string b1(1,line[8]);string b2(1,line[9]);string b3(1,line[10]); string b; b = b1+b2+b3;

//			cout << "R: " << stoi(r) << endl;
//			cout << "G: " << stoi(g) << endl;
//			cout << "B: " << stoi(b) << end"l;

			string topic(line);
			topic.erase(0,12);

//			cout << topic << endl;

			label l;
			l.r = stoi(b); l.g = stoi(g); l.b = stoi(r);
			l.topic = topic;

			labels.push_back(l); 
    }
    myfile.close();
  }

  else cout << "Unable to open file";

//	for(int i=0;i<labels.size();i++)
//		cout << labels[i].r << " " << labels[i].g << " " << labels[i].b << " " << labels[i].topic << endl;

}

// ----------------------------------------------------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------------------------------------------------
// Method for extracting all the objects from a pointcloud (Object have colours meanwhile the rest of the cloud is BLACK (255,255,255))

bool isObject(float x,float y,float z,int r,int g,int b) {

	int alpha = 3;

	for(int i=0;i<labels.size();i++)
		if(r >= labels[i].r-alpha and r <= labels[i].r+alpha and g >= labels[i].g-alpha and g <= labels[i].g+alpha and b >= labels[i].b-alpha and b <= labels[i].b+alpha) {

			PointXYZ p1; p1.x = x; p1.y = y; p1.z = z;
			labels[i].p.push_back(p1);
			return true; //AQUI TENGO QUE METER COGIDO PARA METER X,Y,Z, EN UN VECTOR DE ALMACENAMIENTO GLOBAL	
		}
	return false;
}


// ----------------------------------------------------------------------------------------------------------------------------------------

void callback(string p) {

	int num_point = 0;

	cout << p << endl;


	PointCloud<PointXYZRGB> new_color_cloud;

	vector<PointCloud<PointXYZRGB> > v; 
	
	int r_int,g_int,b_int;

  for (PointCloud<PointXYZRGB>::iterator pt = cloud->begin(); pt < cloud->end(); pt++)
  {

			// unpack rgb into r/g/b
			uint32_t rgb = *reinterpret_cast<int*>(&(*pt).rgb);
			uint8_t r = (rgb >> 16) & 0x0000ff;
			uint8_t g = (rgb >> 8)  & 0x0000ff;
			uint8_t b = (rgb)       & 0x0000ff;

			r_int = int(r);
			g_int = int(g);
			b_int = int(b);

			if(isObject(pt->x,pt->y,pt->z,r_int,g_int,b_int)) {

				num_point++;

			}

  }

	for(int i=0;i<labels.size();i++)
		cout << labels[i].topic << " " << labels[i].p.size() << endl;


	// Localizamos la posicion del vector donde tenemos almacenados los objetos 'p'
	for(int i=0;i<labels.size();i++) {
		if(labels[i].topic.compare(p) == 0) {
						
			for(int j=0;j<labels[i].p.size();j++) {
				PointXYZRGB pt_rgb(labels[i].r,labels[i].g,labels[i].b);

				pt_rgb.x = labels[i].p[j].x;
				pt_rgb.y = labels[i].p[j].y;
				pt_rgb.z = labels[i].p[j].z;


				new_color_cloud.push_back(pt_rgb);

				num_point++;
			}
			
		}
	}

	cout << endl <<  "NUM POINT: " << num_point << endl; 

	sensor_msgs::PointCloud2 color_cloud2;
	toROSMsg(new_color_cloud, color_cloud2);
	color_cloud2.header.seq = id++;
	color_cloud2.header.stamp = ros::Time::now();
	color_cloud2.header.frame_id = "base_link";
	
	ros::Duration(2).sleep();
	cout << "Publishing..." << endl;
     pub.publish(color_cloud2);
	ros::Duration(8).sleep();
	

}



int main(int argc, char *argv[]) {


  ros::init(argc, argv, "getObject");
  ros::NodeHandle nh("~");
  string param;

  nh.getParam("param", param);
  ROS_INFO("Got parameter : %s", param.c_str());

  readLabels("/home/jsm/catkin_ws/src/but_velodyne/but_calibration_camera_velodyne/src/etiquetas.txt");

  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/jsm/.ros/extractedObjects.pcd", *cloud);

  string topic = "/" + param;

  cout << "Topic: " << topic << endl;

  pub = nh.advertise<sensor_msgs::PointCloud2>("/object", 1);

  callback(param);

  ros::spin();
  return 0;

}


