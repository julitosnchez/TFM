#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <limits>
#include <vector>


//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <depth_deco/ROI_identifier.h>
#include "DEOpt/DEOpt.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>




using namespace cv;
using namespace std;

ros::Publisher pub_ROIlocation;
ros::Publisher pub_ROIcloud;
ros::Publisher pub_center;
ros::Publisher pub_HoleCloud;
ros::Publisher pub_ScanmCloud;
ros::Publisher pub_Cloud;
ros::Publisher pub_CylPoints;
ros::Publisher pub_Arrow;



std::vector<sensor_msgs::PointCloud2ConstPtr> cloudlist;
std::vector<sensor_msgs::ImageConstPtr> imglist;
bool check_hole=false;
bool regionSelected = false;
bool selectRegion = false;
Rect2d r; 
typedef pcl::PointXYZ PointT;
ofstream centersFile;


struct parameters {//blob detection parameter structure
    float thresholdStep;
    float minThreshold;
    float maxThreshold;
    size_t minRepeatability;
    float minDistBetweenBlobs;

    bool filterByColor;
    unsigned char blobColor;

    bool filterByArea;
    float minArea;
    float maxArea;

    bool filterByCircularity;
    float minCircularity;
    float maxCircularity;

    bool filterByInertia;
    //minInertiaRatio = 0.6;
    float minInertiaRatio;
    float maxInertiaRatio;

    bool filterByConvexity;
    //minConvexity = 0.8;
    float minConvexity;
    float maxConvexity;
} params;



struct CV_EXPORTS Center
{   Point2d location;
      double radius;
      double confidence;
  };




bool cloudSubset(sensor_msgs::PointCloud2ConstPtr cld_msg, Mat im_scan, Mat im_msg, depth_deco::ROI_identifier ROI_scan, depth_deco::ROI_identifier ROI_msg, int hole_id) //Point cloud hole segmentation and publishing (hole and surroundings)
{
    bool fp=false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ROI (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hole (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scanm (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (*cld_msg, *cloud);

    pcl::ExtractIndices<pcl::PointXYZ> extract_zoom;
    pcl::ExtractIndices<pcl::PointXYZ> extract_hole;
    pcl::ExtractIndices<pcl::PointXYZ> extract_rest;
     pcl::ExtractIndices<pcl::PointXYZ> extract_scanm;

    pcl::PointIndices::Ptr inliers_zoom (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers_hole (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers_rest (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers_scanm (new pcl::PointIndices ());

    Point center(ROI_scan.width/2,ROI_scan.height/2);

    double radius = ROI_scan.width/28;

    for (int i=0; i< im_scan.cols; i++){
           for (int j=0; j<im_scan.rows; j++){
             double distancia=sqrt(pow((i-center.x),2)+pow((j-center.y),2));

             if (distancia>radius*2){
                inliers_scanm->indices.push_back( (j+ROI_scan.y_offset)*512 + (i+ROI_scan.x_offset) );
             }
           }
       }


       for (int i=0; i< im_msg.cols; i++){
        for (int j=0; j<im_msg.rows; j++){
	  	
          inliers_zoom->indices.push_back( (j+ROI_msg.y_offset)*512 + (i+ROI_msg.x_offset) );
          Scalar intensity = im_msg.at<uchar>(i, j);
          //std::cerr << "Prueba img " << intensity.val[0] << std::endl;
          if (intensity.val[0]==255){
             inliers_hole->indices.push_back( (j+ROI_msg.y_offset)*512 + (i+ROI_msg.x_offset) );
          }
          else{
              inliers_rest->indices.push_back( (j+ROI_msg.y_offset)*512 + (i+ROI_msg.x_offset) );
          }
        }
    }


    extract_zoom.setInputCloud (cloud);
    extract_zoom.setIndices (inliers_zoom);
    extract_zoom.setNegative (false);
    extract_zoom.filter (*cloud_ROI);

    extract_hole.setInputCloud (cloud);
    extract_hole.setIndices (inliers_hole);
    extract_hole.setNegative (false);
    extract_hole.filter (*cloud_hole);

    extract_rest.setInputCloud (cloud);
    extract_rest.setIndices (inliers_rest);
    extract_rest.setNegative (false);
    extract_rest.filter (*cloud_rest);

    extract_scanm.setInputCloud (cloud);
    extract_scanm.setIndices (inliers_scanm);
    extract_scanm.setNegative (false);
    extract_scanm.filter (*cloud_scanm);

    /*pcl::PointCloud<pcl::Normal>::Ptr norm_cloud;
    norm_cloud=calc_normals(cloud_hole);
    normalsVis(cloud_hole,norm_cloud);*/

    sensor_msgs::PointCloud2 cloud_ROI_msg;
    pcl::toROSMsg (*cloud_ROI, cloud_ROI_msg);
    cloud_ROI_msg.header.frame_id = "kinect2_ir_optical_frame";
    pub_ROIcloud.publish (cloud_ROI_msg);

    sensor_msgs::PointCloud2 cloud_Hole_msg;
    pcl::toROSMsg (*cloud_hole, cloud_Hole_msg);
    cloud_ROI_msg.header.frame_id = "kinect2_ir_optical_frame";
    pub_HoleCloud.publish (cloud_Hole_msg);

    sensor_msgs::PointCloud2 cloud_Scanm_msg;
    pcl::toROSMsg (*cloud_scanm, cloud_Scanm_msg);
    cloud_ROI_msg.header.frame_id = "kinect2_ir_optical_frame";
    pub_ScanmCloud.publish (cloud_Scanm_msg);

    //cylinderModel_normals(cloud_ROI);
    //CylMarker();


     cloud_ROI_msg.header.frame_id = "kinect2_ir_optical_frame";
    pub_Cloud.publish (cld_msg);

    double z;
    double minz=1000000;
    double maxz=0;	
    for (int i=0;i<cloud_ROI->points.size();i++){

        z=cloud_ROI->points[i].z;
        if ((!isnan(z))&&(minz>z)){minz=z;}
	else if ((!isnan(z))&&(maxz<z)){maxz=z;}
    }

    double center_x=cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].x;
    double center_y=cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].y;
    double center_z=minz;
    double distance=sqrt(center_x*center_x+center_y*center_y+center_z*center_z);
    std::cerr << "Location (x,y,z): [" << center_x <<","<< center_y <<","<< center_z <<"]"<< std::endl;
    std::cerr << "Distance:" << distance <<"meters"<< std::endl;
    centersFile << cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].x << "," << cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].y << "," <<minz<< endl;	

    geometry_msgs::Point center_msg;
    center_msg.x=center_x;
    center_msg.y=center_y;
    center_msg.z=center_z;
    pub_center.publish (center_msg);

    
		visualization_msgs::Marker loc_vector;
		geometry_msgs::Point initial_point,end_point;

    loc_vector.header.frame_id = "/kinect2_ir_optical_frame";
    loc_vector.header.stamp = ros::Time();
    loc_vector.id = 0;
    loc_vector.type = visualization_msgs::Marker::ARROW;
    loc_vector.action = visualization_msgs::Marker::ADD;


    initial_point.x = 0;
		initial_point.y = 0;
    initial_point.z = 0;
    //loc_vector.points[0].z = 0;

		loc_vector.points.push_back(initial_point);

    end_point.x = center_x;
    end_point.y = center_y;
    end_point.z = center_z;

		loc_vector.points.push_back(end_point);

    loc_vector.scale.x = 0.03;
    loc_vector.scale.y = 0.03;
		loc_vector.scale.z = 0.03;

    loc_vector.color.a = 1.0;
    loc_vector.color.r = 0.0;
    loc_vector.color.g = 1.0;
    loc_vector.color.b = 0.0;

    loc_vector.lifetime = ros::Duration();
    pub_Arrow.publish(loc_vector);

/*
    if ((!isnan(cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].x))&&(!isnan(cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].y))&&(!isnan(minz))){
	

        //Writing whole cloud .pcd
        std::ostringstream os;
        os << "cloud" << hole_id+1 << ".pcd";
        string va=os.str();
        const char *filenam=va.c_str();
				cout << "NOMBRE FICHERO: " << filenam << endl;
        pcl::io::savePCDFileASCII (filenam, *cloud);
				cout << "DESPUES DE PCD" << endl;

        //Writing rest cloud .pcd
        std::ostringstream osv;
        osv << "rest" << hole_id+1 << ".pcd";
        string varv=osv.str();
        const char *filenamev=varv.c_str();
				cout << "NOMBRE FICHERO: " << filenam << endl;
        pcl::io::savePCDFileASCII (filenamev, *cloud_rest);
				cout << "DESPUES DE PCD2" << endl;

        //Writing hole cloud .pcd
        std::ostringstream oss;
        oss << "hole" << hole_id+1 << ".pcd";
        string var=oss.str();
        const char *filename=var.c_str();
				cout << "NOMBRE FICHERO: " << filenam << endl;
        pcl::io::savePCDFileASCII (filename, *cloud_hole);
				cout << "DESPUES DE PCD3" << endl;
        //std::cerr << "Saved " << cloud_hole->points.size () << " data  points to hole.pcd." << std::endl;
        std::cerr << "Hole points: " << cloud_hole->points.size() << endl;

	/Filtering ROI cloud to extract entrance wall rests
	pcl::PassThrough<pcl::PointXYZ> pass;
    	pass.setInputCloud (cloud_ROI);
    	pass.setFilterFieldName ("z");
    	pass.setFilterLimits (minz+0.01, maxz-0.015);
    	//pass.setFilterLimitsNegative (true);
    	pass.filter (*cloud_ROI);
	
	//Writing ROI cloud .pcd
        std::ostringstream ost;
        ost << "roi" << hole_id+1 << ".pcd";
        string vart=ost.str();
        const char *filenamet=vart.c_str();
				cout << "NOMBRE FICHERO: " << filenam << endl;
        pcl::io::savePCDFileASCII (filenamet, *cloud_ROI);
				cout << "DESPUES DE PCD4" << endl;

        //Writing sacn matching cloud .pcd
        std::ostringstream osz;
        osz << "scanm" << hole_id+1 << ".pcd";
        string varz=osz.str();
        const char *filenamez=varz.c_str();
				cout << "NOMBRE FICHERO: " << filenam << endl;
        pcl::io::savePCDFileASCII (filenamez, *cloud_scanm);
				cout << "DESPUES DE PCD5" << endl;

        
    }
    else {

	std::cerr << "Hole discarded " << endl;
        fp=true; } */

    std::cout << "__________________________"<<endl<<endl;

    return fp;
}



Mat CircleDetection(Mat res_image) //Hough circle detection & mask the interior of the detected circle

{
        vector<Vec3f> circles;
        Mat image=res_image.clone();

        erode(image, image, 0, Point(-1,-1), 2, 1, 1 );
        dilate(image, image, 0, Point(-1,-1), 2, 1, 1 );
        Canny( image, image, 5, 70, 3 );
        //namedWindow("Canny", CV_WINDOW_AUTOSIZE);
        //imshow("Canny", image);
        moveWindow("Canny", 512+375,360);
        HoughCircles(image, circles, CV_HOUGH_GRADIENT,1,20, 180, 10,0,200);
        //std::cout << "Found circles:" <<circles.size() << std::endl;

        Mat maskedImage=res_image;
        Mat mask(image.size(),image.type());
        mask.setTo(cv::Scalar(0));

        // Draw circles
        for( size_t i = 0; i < circles.size(); i++ )
           {
           Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

           int radius = cvRound(circles[i][2]);

           // draw the circle outline
           circle( mask, center, radius, Scalar(255), -1, 8, 0 );
           res_image.copyTo(maskedImage,mask);
           ;

        }
        return maskedImage;
}



void findBlobs(InputArray _image, InputArray _binaryImage, std::vector<Center> &centers)
{
    //CV_INSTRUMENT_REGION()

    Mat image = _image.getMat(), binaryImage = _binaryImage.getMat();
    (void)image;
    centers.clear();

    std::vector < std::vector<Point> > contours;
    Mat tmpBinaryImage = binaryImage.clone();
    findContours(tmpBinaryImage, contours, RETR_LIST, CHAIN_APPROX_NONE);
//    namedWindow("canny", CV_WINDOW_AUTOSIZE);
//    imshow("canny", tmpBinaryImage);
//    waitKey(0);

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Center center;
        center.confidence = 1;
        Moments moms = moments(Mat(contours[contourIdx]));

        if (params.filterByArea)
        {
            double area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
        }

        if (params.filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = arcLength(Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
        }

        if (params.filterByInertia)
        {
            double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
            const double eps = 1e-2;
            double ratio;
            if (denominator > eps)
            {
                double cosmin = (moms.mu20 - moms.mu02) / denominator;
                double sinmin = 2 * moms.mu11 / denominator;
                double cosmax = -cosmin;
                double sinmax = -sinmin;

                double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
                double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
                ratio = imin / imax;
            }
            else
            {
                ratio = 1;
            }

            if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
                continue;

            center.confidence = ratio * ratio;
        }

        if (params.filterByConvexity)
        {
            std::vector < Point > hull;
            convexHull(Mat(contours[contourIdx]), hull);
            double area = contourArea(Mat(contours[contourIdx]));
            double hullArea = contourArea(Mat(hull));
            double ratio = area / hullArea;
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
        }

        if(moms.m00 == 0.0)
            continue;
        center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (params.filterByColor)
        {
            if (binaryImage.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)) != params.blobColor)
                continue;
        }

        //compute blob radius
        {
            std::vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                Point2d pt = contours[contourIdx][pointIdx];
                dists.push_back(norm(center.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        centers.push_back(center);
//        std::cout << "Contour center " << center.location << endl;
//        std::cout << "Area=" <<moms.m00<<  std::endl;
//        std::cout << "_______________________________________________"<<  std::endl;

    }
}




std::vector<KeyPoint> BlobDetection(Mat image, double dist)
{

        std::vector<KeyPoint> keypoints;
        std::vector < std::vector<Center> > centers;

        //Detection parameters
        // Binarization thresholds
        params.minThreshold = 0;
        params.maxThreshold = 255;
        params.thresholdStep=10;
        params.minRepeatability = 2;
        params.minDistBetweenBlobs = 10;

        //Filter by color
        params.filterByColor = true;
        params.blobColor = 255;

        // Filter by Area.
        params.filterByArea = true;
        //params.minArea = 20;
        //params.maxArea = 500;
        params.minArea = -7*dist/1000+24;
        params.maxArea = 195.4*(sin(dist/1000-3.1416))+18.34*(pow((dist/1000-10),2))-846;

        // Filter by Circularity
        params.filterByCircularity = false;
        params.minCircularity = 0.55;
        params.maxCircularity = std::numeric_limits<float>::max();

        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.85;
        params.maxConvexity = std::numeric_limits<float>::max();

        // Filter by Inertia
        params.filterByInertia = false;
        params.minInertiaRatio = 0.1;
        params.maxInertiaRatio = std::numeric_limits<float>::max();



        //Blob detection (sending binarized images)

        for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
          {
           Mat binarizedImage;
           threshold(image, binarizedImage, thresh, 255, THRESH_BINARY);
           /*namedWindow("Binarized", CV_WINDOW_AUTOSIZE);
           imshow("Binarized", binarizedImage);
           waitKey(0);*/

           std::vector < Center > curCenters;
           std::vector < std::vector<Center> > newCenters;
           findBlobs(binarizedImage, binarizedImage, curCenters);

           for (size_t i = 0; i < curCenters.size(); i++)
                {
                bool isNew = true;
                for (size_t j = 0; j < centers.size(); j++)
                    {
                        double dist = norm(centers[j][ centers[j].size() / 2 ].location - curCenters[i].location);
                        isNew = dist >= params.minDistBetweenBlobs && dist >= centers[j][ centers[j].size() / 2 ].radius && dist >= curCenters[i].radius;
                        if (!isNew)
                        {
                            centers[j].push_back(curCenters[i]);

                            size_t k = centers[j].size() - 1;
                            while( k > 0 && centers[j][k].radius < centers[j][k-1].radius )
                            {
                                centers[j][k] = centers[j][k-1];
                                k--;
                            }
                            centers[j][k] = curCenters[i];

                            break;
                        }
                    }
                    if (isNew) {
                        newCenters.push_back(std::vector<Center> (1, curCenters[i]));
                    }
                }
            std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));

        }

        for (size_t i = 0; i < centers.size(); i++)
         {

                if (centers[i].size() < params.minRepeatability)
                    continue;
                Point2d sumPoint(0, 0);
                double normalizer = 0;
                for (size_t j = 0; j < centers[i].size(); j++)
                {
                    sumPoint += centers[i][j].confidence * centers[i][j].location;
                    normalizer += centers[i][j].confidence;
                }
                sumPoint *= (1. / normalizer);

                KeyPoint kpt(sumPoint, (float)(centers[i][0].radius) * 2.0f);
                keypoints.push_back(kpt);
         }
return keypoints;
}




std::vector<double> scaleDepth(Mat img, float range) // parameter obtention to scale a depth image(greyscale) between a distance range referring to the average to focus on

{	//Average, min, max pixel calculation on ROI
        float sum=0;
        float avg=0;
        int pixel_info=0;
        double min, max;
        Mat mask = img>0 & img<4000;

        cv::minMaxLoc(img, &min, &max, NULL, NULL, mask);

        for(int j=0;j<img.rows;j++)
           {
            for (int i=0;i<img.cols;i++)
            { if (img.at<float>(j,i)>0)
                {pixel_info=pixel_info +1;}

             sum= sum + img.at<float>(j,i);
            }
           }
        avg= sum / pixel_info;

//        if ((max-avg)>=(avg-min))
//            range=max-avg;
//        else
//            range=avg-min;

        // Converting ROI to 8bit and scaling distances between 0 and 255
        double alpha=255/(2*range);
        double beta=255/((avg+range)/(range-avg)+1);

        std::vector<double> conVparams(5);
        conVparams[0]=alpha;
        conVparams[1]=beta;
        conVparams[2]=avg;
        conVparams[3]=min;
        conVparams[4]=max;
        return conVparams;
}


cv::Rect ROI_w_limit_comprobation (Mat img, float size, KeyPoint kypt) //ROI selection over holes considering the borders not to step out the image

{
    int cols=img.cols;
    int rows=img.rows;
    Rect rectangle;
    float x_ini=kypt.pt.x-size/2;
    float y_ini=kypt.pt.y-size/2;
    float x_size=size;
    float y_size=size;

    if (kypt.pt.x-size/2 < 0)
        {x_ini=0;
         x_size=size/2+kypt.pt.x;}

    if (kypt.pt.y-size/2 < 0)
        {y_ini=0;
         y_size=size/2+kypt.pt.y;}

    if (kypt.pt.x+size/2 > cols)
        {x_ini=kypt.pt.x-size/2;
         x_size=size/2+cols-kypt.pt.x;}

    if (kypt.pt.y+size/2 > rows)
        {y_ini=kypt.pt.y-size/2;
         y_size=size/2+rows-kypt.pt.y;}

    rectangle=Rect(x_ini,y_ini,x_size,y_size);

    return rectangle;
}



void processing (const sensor_msgs::Image msg, sensor_msgs::PointCloud2ConstPtr cloud_msg)// 2D/3D processing to detect blast holes (main)
{


        /*ros::Time begin = ros::Time::now();*/
        depth_deco::ROI_identifier ROI_msg;
        depth_deco::ROI_identifier ROI_scan;



         //std::cout << "Cloud seq:" << cloud_msg->header.seq  << endl<< endl<< endl;
         //std::cout << "Depth seq:" << msg.header.seq  << endl<< endl<< endl;
         std::cout << "__________________________"<< endl;


         //Conversion of ros msg, scale image
         Mat idepth, idepth_scaled, mirror;

			
         idepth=cv_bridge::toCvCopy(msg, "32FC1")->image; //conversion
         //flip(mirror,idepth,1);
         imwrite( "../../images/Gray_Image.jpg", idepth );

         // Scaling to full depth resolution

         std::vector<double> scale_params(5);
         float range=200;
         std::cout << "Depth Image Scaling Info"<< std::endl << std::endl;
         scale_params=scaleDepth(idepth,range);  //scale param obtention to a distance range
         std::cout << "Average:" << scale_params[2] << std::endl;
         std::cout << "Min:" << scale_params[3] << std::endl;
         std::cout << "Max:" << scale_params[4] << std::endl<<endl;
         std::cout << "__________________________"<< endl;
         idepth.convertTo(idepth_scaled, CV_8UC1,scale_params[0], scale_params[1]);//scaling

        //namedWindow("Depth", CV_WINDOW_AUTOSIZE); //idepth_scaled= scaled original image
        //imshow("Depth", idepth_scaled);
			  bool fromCenter = false;


        //Hole detection
        std::vector<KeyPoint> holes;
				Mat idepth_scaled_clone = idepth_scaled.clone();

				if(regionSelected)
					idepth_scaled = idepth_scaled(r);

        holes=BlobDetection(idepth_scaled, scale_params[2]);

        std::cout << "Found holes:" <<holes.size() << std::endl<<std::endl;
        std::cout << "__________________________"<< endl;

        Mat idepth_scaled_copy=idepth_scaled.clone();

        // Draw detected holes in ROI and complete image.
       Mat img_with_keypoints;
       drawKeypoints( idepth_scaled, holes, img_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

       namedWindow("Holes Detected", CV_WINDOW_AUTOSIZE);
       //moveWindow("Holes",0,idepth_scaled.rows+160);

       for (int i=0; i<holes.size();i++){

           ostringstream convert;
           convert << i;
           string s = convert.str();
           //std::cout << "Points" << holes[i].pt <<std::endl;


           putText(img_with_keypoints, s, holes[i].pt, FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
       }


			 if(selectRegion){

		     r = selectROI(idepth_scaled_clone,fromCenter);

				 selectRegion = false;
				 regionSelected = true;
		  	 // Crop image
		  }

			 imshow("Holes Detected",img_with_keypoints);

       waitKey(500);


       if ((holes.size()!=0) && (check_hole==true)) {
        //cout << "check_hole" << endl;
        check_hole=false;
        int h_i;
        cout << "Select a hole " << endl;
        cin >> h_i;
        //cout << "The value you entered is " << h_i;

        //waitKey(0);
        std::cout << "Hole " << h_i <<":" << endl;
        //std::cout << "Hole " << h_i <<":" << holes[h_i].pt <<"(pixels)"<< endl;
        //std::cout << "Size" <<":" << holes[h_i].size <<"pxl"<< endl;

        //ROIs (scan and hole isolating) in each hole, identifying them in pointcloud and identification of each hole's pixels
        int roi_width=holes[h_i].size*1.2;
        int roi_height=holes[h_i].size*1.2;
        Rect rectzoom =ROI_w_limit_comprobation(idepth_scaled, roi_width, holes[h_i]);
        //Rect rectzoom=Rect(holes[h_i].pt.x-roi_width/2,holes[h_i].pt.y-roi_height/2,roi_width,roi_height);
        ROI_msg.x_offset=(uint32_t)holes[h_i].pt.x-roi_width/2 + r.x;  //cout << "X_OFFSET: " << ROI_msg.x_offset << " + " << r.x << " = " << (uint32_t)ROI_msg.x_offset+r.x << endl;
        ROI_msg.y_offset=(uint32_t)holes[h_i].pt.y-roi_height/2 +r.y;  //cout << "Y_OFFSET: " << ROI_msg.y_offset << " + " << r.y << " = " << (uint32_t)ROI_msg.y_offset+r.y << endl;
        ROI_msg.height=(uint32_t)roi_height;
        ROI_msg.width=(uint32_t)roi_width;
        pub_ROIlocation.publish (ROI_msg);

        int roi_scansize=holes[h_i].size*1;
        Rect scanzoom =ROI_w_limit_comprobation(idepth_scaled, roi_scansize, holes[h_i]);
        //Rect scanzoom=Rect(holes[h_i].pt.x-roi_scansize/2,holes[h_i].pt.y-roi_scansize/2,roi_scansize,roi_scansize);
        ROI_scan.x_offset=(uint32_t)holes[h_i].pt.x-roi_scansize/2; 
        ROI_scan.y_offset=(uint32_t)holes[h_i].pt.y-roi_scansize/2; 
        ROI_scan.height=(uint32_t)roi_scansize;
        ROI_scan.width=(uint32_t)roi_scansize;

        Mat zoom = idepth_scaled(rectzoom);
        //namedWindow("Zoom", CV_WINDOW_AUTOSIZE);
        //imshow("Zoom", zoom);

        Mat im_scan = idepth_scaled(scanzoom);
        //namedWindow("Scan Zoom", CV_WINDOW_AUTOSIZE);
        //imshow("Scan Zoom", im_scan);

        //equalize each hole image
        Mat zoomeq;
        equalizeHist(zoom,zoomeq);
        //histogram_calculation(zoom);
        namedWindow("Equalized Zoom", CV_WINDOW_AUTOSIZE);
        imshow("Equalized Zoom", zoomeq);


        //highlight each selected hole
        rectangle(idepth_scaled_copy, rectzoom, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow("ROI scaled", idepth_scaled_copy);
        idepth_scaled_copy=idepth_scaled.clone();

        //detect circle and isolate it
        Mat maskedZoom= CircleDetection(zoomeq);
        //namedWindow("Circled Hole", CV_WINDOW_AUTOSIZE);
        //imshow("Circled Hole", maskedZoom );


        // floodfill each isolated hole from the center
        Point seedpoint=cvPoint(roi_width/2, roi_height/2);
        Scalar inten=zoom.at<uchar>(roi_width/2, roi_height/2);
        //std::cout << "Greyscale center value:"<< inten.val[0] << endl;
        floodFill(maskedZoom,seedpoint,Scalar(255),0, 40,255, 4 | FLOODFILL_FIXED_RANGE);
        maskedZoom.convertTo(maskedZoom, CV_8UC1);
        //namedWindow("Zoom FloodFill", CV_WINDOW_AUTOSIZE);
        //imshow("Zoom FloodFill", maskedZoom);
        bool fp=cloudSubset(cloud_msg,im_scan, maskedZoom, ROI_scan, ROI_msg,h_i); //point cloud crop
        //if (fp==true){holes.erase(holes.begin()+h_i);}

        centersFile.close();
        waitKey(0);

       }
       else {
        std::cout << "No holes found" << std::endl<<std::endl;

       }
     //ros::Time end = ros::Time::now();
     //std::cout << "Time " << end-begin << endl;

}

void kbinput_cb(std_msgs::UInt8 status)

{
    if (status.data==1) {
        cout << "dentro" << endl;

        check_hole=true;
    } else if (status.data == 2)
				selectRegion = true;

}


/*void kb_key_cb (std_msgs::Int8 key)

{
    if (key.data==113) {
        cout << "dentro" << endl;

        check_hole=true;}

}*/

void cloud_cb (sensor_msgs::PointCloud2ConstPtr cloud_msg)// point cloud callback: kinect point cloud buffer
{
   sensor_msgs::Image im_msg;
   cloudlist.push_back(cloud_msg);
   for (int i=0;i<imglist.size();i++){
        if (imglist[i]->header.seq==cloud_msg->header.seq){
           im_msg=*imglist[i];
           cloudlist.clear();
           imglist.clear();
           processing(im_msg,cloud_msg);
        }

   }
}




void depth_cb (const sensor_msgs::ImageConstPtr im_msg)//depth image callback: kinect depth image buffer
{

    sensor_msgs::PointCloud2ConstPtr cloud_msg;
   imglist.push_back(im_msg);
   for (int i=0;i<cloudlist.size();i++){
       if (cloudlist[i]->header.seq==im_msg->header.seq){
           cloud_msg=cloudlist[i];
           cloudlist.clear();
           imglist.clear();
           processing(*im_msg,cloud_msg);
        }
   }
}





int main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "Hole_detection_node");
        ros::NodeHandle nh;
        centersFile.open("/home/jcl/Matlab2017b/DE_cylinder_modelling/centers.csv");

        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub = nh.subscribe ("/kinect2/sd/image_depth", 1, depth_cb);
        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub2 = nh.subscribe ("/kinect2/hd/points", 1, cloud_cb);
        // Create a ROS subscriber for the keyboard_input
        ros::Subscriber sub3 = nh.subscribe ("/Keyboard_status", 1, kbinput_cb);
        // Create a ROS subscriber for the keyboard_input
        //ros::Subscriber sub4 = nh.subscribe ("Keyboard_key", 1, kb_key_cb);



        // Create a ROS publisher for each hole ROI identifier
        pub_ROIlocation = nh.advertise<depth_deco::ROI_identifier> ("ROI_location", 1);
        // Create a ROS publisher for the ROI cloud
        pub_ROIcloud = nh.advertise<sensor_msgs::PointCloud2> ("ROICloud", 1);
        // Create a ROS publisher for authentic center point of each hole
        pub_center = nh.advertise<geometry_msgs::Point> ("HoleCenter", 1);
        // Create a ROS publisher for authentic Hole points
        pub_HoleCloud = nh.advertise<sensor_msgs::PointCloud2> ("HoleCloud", 1);
        // Create a ROS publisher for the Rest cloud
        pub_Cloud = nh.advertise<sensor_msgs::PointCloud2> ("Cloud", 1);
        // Create a ROS publisher for the Scan Matching cloud
        pub_ScanmCloud = nh.advertise<sensor_msgs::PointCloud2> ("ScanmCloud", 1);
        // Create a ROS publisher for the Cylinder obtained through RANSAC
        pub_CylPoints = nh.advertise<sensor_msgs::PointCloud2> ("CylinderPoints", 1);
        // Create a ROS publisher for DE-Optmimized Cylinder Model
        pub_Arrow = nh.advertise<visualization_msgs::Marker> ("CenterVector", 1);
        // Spin
        ros::spin ();
}

