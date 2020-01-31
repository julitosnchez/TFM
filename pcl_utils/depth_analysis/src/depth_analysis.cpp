
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//openCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
// cv_bridge
#include <cv_bridge/cv_bridge.h>

int c=0;

using namespace cv;
using namespace std;

void depth_cb (const sensor_msgs::Image msg) { 
  c =c +1;
  std::cout << c << std::endl;
  cout << "TYPE: " << msg.encoding << endl;

  cv::Mat img, img_u1, img_u3;
  img=cv_bridge::toCvCopy(msg, "32FC1")->image;
  cout << "TYPE CV: " << img.type() << endl;

  //img.convertTo(img_u1, CV_8UC1);
  //img.convertTo(img_u3, CV_8UC3);
  Mat color_img;
  cvtColor( img, color_img, CV_GRAY2BGR );

  cout << "Sensor Image: " << msg.height  << "  " << msg.width << endl;
  cout << "Imagen B/W: " << img.rows  << "  " << img.cols << endl;
  cout << "Imagen color: " << color_img.rows  << "  " << color_img.cols << endl;

  Vec3b color_R(0,0,255);
  Vec3b color_G(0,255,0);
  unsigned int num_nan = 0;
  unsigned int num_zero = 0;
  unsigned int num_info = 0;

  if (c==5) {

    for(int y=0;y<img.rows;y++) {
      for(int x=0;x<img.cols;x++) {
//          cout << img_u3.at<Vec3b>(Point(x,y)) << endl;
    //      cout << img.at<float>(Point(x,y)) << endl;

        if (std::isnan(img.at<float>(Point(x,y)) ) ) {
//          color_img.at<Vec3b>(Point(x,y)) = color_R;
          num_nan++;
        }  
        else if (img.at<float>(Point(x,y))> 0) {
//          color_img.at<Vec3b>(Point(x,y)) = color_G;
          num_info++;
        }
        else {
//          color_img.at<Vec3b>(Point(x,y)) = Vec3b(255,0,0);
          num_zero++;
        }
      }
    }  
    namedWindow("Depth", CV_WINDOW_AUTOSIZE);
    namedWindow("color", CV_WINDOW_AUTOSIZE);
//    namedWindow("U3", CV_WINDOW_AUTOSIZE);

    imshow("Depth", img);
    imshow("color", color_img); 

    cout << "NAN: " << num_nan  << "  " << "ZERO: " << "  " << num_zero << "INFO: " << num_info  << endl;
    
    waitKey(0);  
  }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "capture");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("depth_input", 1, depth_cb);

   // Spin
  ros::spin ();
}
