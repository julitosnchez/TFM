#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <limits>

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


int distancia=2; //images took at 1,2 & 3 meters distance. Distance = 0, whole image

using namespace cv;
using namespace std;

ros::Publisher pub_ROIlocation;
ros::Publisher pub_ROIcloud;
ros::Publisher pub_HoleCloud;
ros::Publisher pub_RestCloud;
ros::Publisher pub_CylPoints;
ros::Publisher pub_CylModel;


std::vector<sensor_msgs::PointCloud2ConstPtr> cloudlist;
std::vector<sensor_msgs::ImageConstPtr> imglist;

ofstream outputFile;

void histogram_calculation (Mat image)//Calculate & plot a greyscale histogram
{
        // Histogram calculation variables
        Mat hist;
        int histSize = 256;
        float range[] = { 0, 256 };
        const float* histRange = { range };
        bool uniform= true;
        bool accumulate = false;

        //Histogram Draw variables
        int hist_w = 256; int hist_h = 400;
        int bin_w = cvRound((double)hist_w / histSize);
        Mat histImage(hist_h, hist_w, CV_8UC1, Scalar(0, 0, 0));


        calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

        //cout << "hist: " << hist << endl;

        normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

        //cout << "hist: " << hist << endl;

        // Draw
        for (int i = 1; i < histSize; i++)
            {
                line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i - 1))), Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))), Scalar(255, 0, 0), 2, 8, 0);
            }

        namedWindow("Histogram", CV_WINDOW_AUTOSIZE);
        imshow("Histogram", histImage);
        /*moveWindow("Histogram", 512,350); */
}

pcl::PointCloud<pcl::Normal>::Ptr calc_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //point cloud normals calculation
{   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (5);
    ne.compute (*cloud_normals);

return (cloud_normals);
}

void normalsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)//visualization on PCLViewer of a point cloud normal vectors
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "Hole cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.02, "Normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Hole cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "Hole cloud2");

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void cylinderModel (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //Cylinder model obtention and publishing from a ransac segmented point cloud
{
    //Objets
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    //Datasets
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
    sensor_msgs::PointCloud2 msg_cylinder;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud);


    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cld_cylinder);
    if (cld_cylinder->points.empty ())
        std::cout << "Can't find the cylindrical component." << std::endl;
      else
      {
              std::cout << "PointCloud representing the cylindrical component: " << cld_cylinder->points.size () << " data points." << std::endl;
              pcl::toROSMsg (*cld_cylinder, msg_cylinder);
              pub_CylPoints.publish (msg_cylinder);

              visualization_msgs::Marker cyl_marker;
              cyl_marker.header.frame_id = "/world";
              cyl_marker.header.stamp = ros::Time::now();
              //cyl_marker.ns = "basic_shapes" + boost::lexical_cast<std::string>(it);
              cyl_marker.id = 0;
              cyl_marker.type = visualization_msgs::Marker::CYLINDER;
              //cyl_marker.action = visualization_msgs::Marker::ADD;
              cyl_marker.pose.position.x = coefficients_cylinder->values[0];
              cyl_marker.pose.position.y = coefficients_cylinder->values[1];
              cyl_marker.pose.position.z = coefficients_cylinder->values[2];
              cyl_marker.pose.orientation.x = coefficients_cylinder->values[3];
              cyl_marker.pose.orientation.y = coefficients_cylinder->values[4];
              cyl_marker.pose.orientation.z = coefficients_cylinder->values[5];
              cyl_marker.pose.orientation.w = 1.0;
              cyl_marker.scale.x = coefficients_cylinder->values[6];
              cyl_marker.scale.y = coefficients_cylinder->values[6];
              cyl_marker.scale.z = coefficients_cylinder->values[6];
              cyl_marker.color.r = 0.0f;
              cyl_marker.color.g = 1.0f;
              cyl_marker.color.b = 0.0f;
              cyl_marker.color.a = 1.0;
              cyl_marker.lifetime = ros::Duration();
              pub_CylModel.publish(cyl_marker);
      }

}

void cylinderModel_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //Cylinder model obtention and publishing from a ransac segmented point cloud using normals
{
    //Objets
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    //Datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
    sensor_msgs::PointCloud2 msg_cylinder;

    //Normal calculation
    cloud_normals=calc_normals(cloud);


    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);


    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cld_cylinder);
    if (cld_cylinder->points.empty ())
        std::cout << "Can't find the cylindrical component." << std::endl;
      else
      {
              std::cout << "PointCloud representing the cylindrical component: " << cld_cylinder->points.size () << " data points." << std::endl;
              pcl::toROSMsg (*cld_cylinder, msg_cylinder);
              pub_CylPoints.publish (msg_cylinder);

              visualization_msgs::Marker cyl_marker;
              cyl_marker.header.frame_id = "/world";
              cyl_marker.header.stamp = ros::Time::now();
              //cyl_marker.ns = "basic_shapes" + boost::lexical_cast<std::string>(it);
              cyl_marker.id = 0;
              cyl_marker.type = visualization_msgs::Marker::CYLINDER;
              //cyl_marker.action = visualization_msgs::Marker::ADD;
              cyl_marker.pose.position.x = coefficients_cylinder->values[0];
              cyl_marker.pose.position.y = coefficients_cylinder->values[1];
              cyl_marker.pose.position.z = coefficients_cylinder->values[2];
              cyl_marker.pose.orientation.x = coefficients_cylinder->values[3];
              cyl_marker.pose.orientation.y = coefficients_cylinder->values[4];
              cyl_marker.pose.orientation.z = coefficients_cylinder->values[5];
              cyl_marker.pose.orientation.w = 1.0;
              cyl_marker.scale.x = coefficients_cylinder->values[6];
              cyl_marker.scale.y = coefficients_cylinder->values[6];
              cyl_marker.scale.z = coefficients_cylinder->values[6];
              cyl_marker.color.r = 0.0f;
              cyl_marker.color.g = 1.0f;
              cyl_marker.color.b = 0.0f;
              cyl_marker.color.a = 1.0;
              cyl_marker.lifetime = ros::Duration();
              pub_CylModel.publish(cyl_marker);
      }

}

void CylMarker()  //create a cylinder marker to plot in rviz
{
    visualization_msgs::Marker cyl_marker;
    cyl_marker.header.frame_id = "/world";
    cyl_marker.header.stamp = ros::Time::now();
    //cyl_marker.ns = "basic_shapes" + boost::lexical_cast<std::string>(it);
    cyl_marker.id = 0;
    cyl_marker.type = visualization_msgs::Marker::CYLINDER;
    //cyl_marker.action = visualization_msgs::Marker::ADD;
    cyl_marker.pose.position.x = -0.312775;
    cyl_marker.pose.position.y = -0.456309;
    cyl_marker.pose.position.z = 1.512;
    cyl_marker.pose.orientation.x = -0.0561457;
    cyl_marker.pose.orientation.y = -0.014914;
    cyl_marker.pose.orientation.z = 0.847883;
    cyl_marker.pose.orientation.w = 1.0;
    cyl_marker.scale.x = 0.1;
    cyl_marker.scale.y = 0.1;
    cyl_marker.scale.z = 0.2;
    cyl_marker.color.r = 0.0f;
    cyl_marker.color.g = 1.0f;
    cyl_marker.color.b = 0.0f;
    cyl_marker.color.a = 1.0;
    cyl_marker.lifetime = ros::Duration();
    pub_CylModel.publish(cyl_marker);

}











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



void cloudSubset(sensor_msgs::PointCloud2ConstPtr cld_msg, Mat im_msg, depth_deco::ROI_identifier ROI_msg, int hole_id) //Point cloud hole segmentation and publishing (hole and surroundings)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ROI (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hole (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (*cld_msg, *cloud);

    pcl::ExtractIndices<pcl::PointXYZ> extract_zoom;
    pcl::ExtractIndices<pcl::PointXYZ> extract_hole;
    pcl::ExtractIndices<pcl::PointXYZ> extract_rest;

    pcl::PointIndices::Ptr inliers_zoom (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers_hole (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers_rest (new pcl::PointIndices ());

    for (int i=0; i< im_msg.cols; i++){
        for (int j=0; j<im_msg.rows; j++){
          inliers_zoom->indices.push_back( (j+ROI_msg.y_offset)*512 + (i+ROI_msg.x_offset) );
          Scalar intensity = im_msg.at<uchar>(i, j);
          //std::cerr << "Prueba img " << intensity.val[0] << std::endl;
          if (intensity.val[0]==255){
             inliers_hole->indices.push_back( (j+ROI_msg.y_offset)*512 + (i+ROI_msg.x_offset) );
          }
          else if (intensity.val[0]==0){
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

    /*pcl::PointCloud<pcl::Normal>::Ptr norm_cloud;
    norm_cloud=calc_normals(cloud_hole);
    normalsVis(cloud_hole,norm_cloud);*/

    sensor_msgs::PointCloud2 cloud_ROI_msg;
    pcl::toROSMsg (*cloud_ROI, cloud_ROI_msg);
    cloud_ROI_msg.header.frame_id = "world";
    pub_ROIcloud.publish (cloud_ROI_msg);

    sensor_msgs::PointCloud2 cloud_Hole_msg;
    pcl::toROSMsg (*cloud_hole, cloud_Hole_msg);
    cloud_ROI_msg.header.frame_id = "world";
    pub_HoleCloud.publish (cloud_Hole_msg);
    CylMarker();


    sensor_msgs::PointCloud2 cloud_Rest_msg;
    pcl::toROSMsg (*cloud_rest, cloud_Rest_msg);
    cloud_ROI_msg.header.frame_id = "world";
    pub_RestCloud.publish (cloud_Rest_msg);

    std::ostringstream oss;
    oss << "hole" << hole_id+1 << ".pcd";
    string var=oss.str();
    const char *filename=var.c_str();
    pcl::io::savePCDFileASCII (filename, *cloud_hole);
    //std::cerr << "Saved " << cloud_hole->points.size () << " data  points to hole.pcd." << std::endl;
    //pcl::io::savePCDFileASCII ("rest.pcd", *cloud_rest);
    //std::cerr << "Saved " << cloud_rest->points.size () << " data  points to rest.pcd." << std::endl;


    double sum=0;
    double Pz, avgz;
    for (int i=0;i<cloud_rest->points.size();i++){

        Pz=cloud_rest->points[i].z;
        if (!isnan(Pz)){

            sum+=Pz;
        }
    }
    avgz=sum/cloud_rest->points.size();
    std::cerr << "Hole center(x,y,z): [" << cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].x <<","<< cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].y <<","<< avgz <<"]"<< std::endl;
    std::cout << "__________________________"<< endl<<endl<<endl;

    outputFile << cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].x << "," << cloud_ROI->points[ROI_msg.height*ROI_msg.width/2].y << "," <<avgz << endl;
}



Mat CircleDetection(Mat res_image) //Hough circle detection & mask the interior of the detected circle

{
        vector<Vec3f> circles;
        Mat image=res_image.clone();

        erode(image, image, 0, Point(-1,-1), 2, 1, 1 );
        dilate(image, image, 0, Point(-1,-1), 2, 1, 1 );
        /*Canny( image, image, 5, 70, 3 );
        namedWindow("Canny", CV_WINDOW_AUTOSIZE);
        imshow("Canny", image);
        moveWindow("Canny", 512+375,360);*/
        HoughCircles(image, circles, CV_HOUGH_GRADIENT,1,20, 180, 10,3,20);
        std::cout << "Found circles:" <<circles.size() << std::endl;

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
           return maskedImage;
        }
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
        std::cout << "Distancia"<< dist<< std::endl;
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

        std::cout << "Average:" << avg << std::endl;
        std::cout << "Min:" << min << std::endl;
        std::cout << "Max:" << max << std::endl<<endl;

        // Converting ROI to 8bit and scaling distances between 0 and 255
        double alpha=255/(2*range);
        double beta=255/((avg+range)/(range-avg)+1);

        std::vector<double> conVparams(3);
        conVparams[0]=alpha;
        conVparams[1]=beta;
        conVparams[2]=avg;
        return conVparams;
}



void processing (const sensor_msgs::Image msg, std_msgs::Header h_img)// 2D/3D processing to detect blast holes (main)
{
        /*ros::Time begin = ros::Time::now();*/
        depth_deco::ROI_identifier ROI_msg;

        //Capture point cloud corresponding with depth image
        sensor_msgs::PointCloud2ConstPtr cloud_msg;

        for (int i=0;i<cloudlist.size();i++){
            std_msgs::Header h_cld = cloudlist[i]->header;
            if (h_img.seq==h_cld.seq){
                std::cout << "Depth seq:"<<h_img.seq  << endl;
                cloud_msg=cloudlist[i];
            }
        }
         std_msgs::Header h_cld = cloud_msg->header;
         std::cout << "Cloud seq:" << h_cld.seq  << endl<< endl<< endl;
         std::cout << "__________________________"<< endl;


         //Conversion of ros msg, flip, scale image
         Mat idepth, idepth_show, idepth_scaled;
         Rect rect;
         Mat roiImg, ROI_scaled;
         Mat mirror;

         mirror=cv_bridge::toCvCopy(msg, "32FC1")->image; //conversion
         flip(mirror,idepth,1);

         std::vector<double> alphabeta(2);
         float range=200;
         std::cout << "Depth Image Scaling Info" << std::endl;
         alphabeta=scaleDepth(idepth,range);  //scale param obtention to a distance range
         std::cout << "__________________________"<< endl;
         idepth.convertTo(idepth_scaled, CV_8UC1,alphabeta[0], alphabeta[1]);//scaling


        //ROI Selection and Scaling
        if (distancia==1){
          rect=Rect(0,0,512,300); //ROI 1m_parado.bag
          }
        else if (distancia==2){
          rect=Rect(78,20,380,225);  // ROI 2m_parado.bag
          }
        else if (distancia==3){
          rect=Rect(134,160,200,120);  // ROI 3m_parado.bag
          }



        Point2d ini_ROI(rect.x,rect.y);
        if (distancia==0){
            roiImg = idepth;
        }
        else{
            roiImg = idepth(rect);
            rectangle(idepth_scaled, rect, CV_RGB(255, 0, 0), 3, 8, 0);
        }


        std::cout << "ROI Image Scaling Info" << std::endl;
        alphabeta=scaleDepth(roiImg,range);
        double avg_dist=alphabeta[2];
        std::cout << "__________________________"<< endl;
        roiImg.convertTo(ROI_scaled, CV_8UC1,alphabeta[0], alphabeta[1]);//scaling selected roi to 0-255 grayscale

        namedWindow("Depth", CV_WINDOW_AUTOSIZE); //idepth_scaled= scaled original image
        imshow("Depth", idepth_scaled);

        namedWindow("ROI scaled", CV_WINDOW_AUTOSIZE);//ROI_scaled= scaled selected roi
        imshow("ROI scaled", ROI_scaled);
        moveWindow("ROI scaled",idepth_scaled.cols,0);

        //Hole detection
        std::vector<KeyPoint> holes;
        holes=BlobDetection(ROI_scaled, avg_dist);


        std::vector<KeyPoint> holes_ori(holes.size());

        std::cout << "Total number of found holes:" <<holes.size() << std::endl<<std::endl;
        std::cout << "__________________________"<< endl;
        Mat ROI_scaled_copy=ROI_scaled.clone();

        std::cout << "Press Key for Individual Hole Processing" << endl << endl;

        for (int i=0; i<holes.size();i++){

                  waitKey(0);

                  //Traslate detected holes to original image
                  holes_ori[i]=holes[i];
                  holes_ori[i].pt.x=ini_ROI.x+holes[i].pt.x;
                  holes_ori[i].pt.y=ini_ROI.y+holes[i].pt.y;

                  std::cout << "Hole " << i+1 <<":" << holes[i].pt <<"(pixels)"<< endl;
                  std::cout << "Size" <<":" << holes[i].size <<"pxl"<< endl;

                  //ROI in each hole, identifying them in pointcloud and identification of each hole's pixels
                  Rect rectzoom=Rect(holes[i].pt.x-holes[i].size,holes[i].pt.y-holes[i].size,2*holes[i].size,2*holes[i].size);
                  ROI_msg.x_offset=(uint32_t)idepth.cols-holes_ori[i].pt.x-holes[i].size;
                  ROI_msg.y_offset=(uint32_t)holes_ori[i].pt.y-holes[i].size;
                  ROI_msg.height=(uint32_t)2*holes[i].size;
                  ROI_msg.width=(uint32_t)2*holes[i].size;
                  pub_ROIlocation.publish (ROI_msg);


                  Mat zoom = ROI_scaled(rectzoom);
                  namedWindow("Zoom", CV_WINDOW_AUTOSIZE);
                  imshow("Zoom", zoom);
                  moveWindow("Zoom",idepth_scaled.cols + ROI_scaled.cols,0);

                    //equalize each hole image
                  Mat zoomeq;
                  equalizeHist(zoom,zoomeq);
                  /*histogram_calculation(zoom);*/
                  namedWindow("Equalized Zoom", CV_WINDOW_AUTOSIZE);
                  imshow("Equalized Zoom", zoomeq);
                  moveWindow("Equalized Zoom",idepth_scaled.cols + ROI_scaled.cols,190);

                    //highlight each selected hole
                  rectangle(ROI_scaled_copy, rectzoom, CV_RGB(255, 0, 0), 3, 8, 0);
                  imshow("ROI scaled", ROI_scaled_copy);
                  ROI_scaled_copy=ROI_scaled.clone();

                    //detect circle and isolate it
                  Mat maskedZoom= CircleDetection(zoomeq);
                  namedWindow("Circled Hole", CV_WINDOW_AUTOSIZE);

                  imshow("Circled Hole", maskedZoom );
                  moveWindow("Circled Hole",idepth_scaled.cols + ROI_scaled.cols,zoomeq.rows*20);


                    // floodfill each isolated hole from the center
                  Point seedpoint=cvPoint(holes[i].size, holes[i].size);
                  Scalar inten=zoom.at<uchar>(holes[i].size,holes[i].size);
                  std::cout << "Greyscale center value:"<< inten.val[0] << endl;
                  floodFill(maskedZoom,seedpoint,Scalar(255),0, 40,255, 4 | FLOODFILL_FIXED_RANGE);
                  maskedZoom.convertTo(maskedZoom, CV_8UC1);
                  namedWindow("Zoom FloodFill", CV_WINDOW_AUTOSIZE);
                  imshow("Zoom FloodFill", maskedZoom);
                  moveWindow("Zoom FloodFill",idepth_scaled.cols+ROI_scaled.cols,zoomeq.rows*26);
                  cloudSubset(cloud_msg,maskedZoom,ROI_msg,i); //point cloud crop
        }


        // Draw detected holes in ROI and complete image.
        Mat img_with_keypoints;
        drawKeypoints( idepth_scaled, holes_ori, img_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        namedWindow("Holes", CV_WINDOW_AUTOSIZE);
        imshow("Holes", img_with_keypoints );
        moveWindow("Holes",idepth_scaled.cols+ROI_scaled.cols,ROI_scaled.cols);
        waitKey(0);


     //ros::Time end = ros::Time::now();
     //std::cout << "Time " << end-begin << endl;

}




void cloud_cb (sensor_msgs::PointCloud2ConstPtr cloud_msg)// point cloud callback: kinect point cloud buffer
{
   cloudlist.push_back(cloud_msg);
}




void depth_cb (const sensor_msgs::ImageConstPtr im_msg)//depth image callback: kinect depth image buffer
{
    imglist.push_back(im_msg);
    std_msgs::Header h_img;
    depth_deco::ROI_identifier ROI_msg;

    if (imglist.size()==4){
        h_img = imglist[1]->header;
        ROI_msg.seq=h_img.seq;
        processing(*imglist[1],h_img);
    }
}




int main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "Hole_detection_node");
        ros::NodeHandle nh;
        outputFile.open("/home/jcl/Matlab2017b/DE_cylinder_modelling/centers.csv");

        // DEOptmization Service

        ros::ServiceClient client =nh.serviceClient<DEOpt::DEOpt>("DEOpt");

        /*DEOpt::DEOpt srv;
        srv.request.N_DIM =9;
        srv.request.POP = 100;
        srv.request.MAX_GENERATIONS = 800;
        if (client.call(srv))
        {
          ROS_INFO("Sum: %ld", (long int)srv.response.sum);
        }
        else
        {
          ROS_ERROR("Failed to call the optimization server");
          return 1;
        }*/




        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub = nh.subscribe ("/kinect2/sd/image_depth", 1, depth_cb);

        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub2 = nh.subscribe ("/kinect2/hd/points", 1, cloud_cb);

        if (distancia!=0){
        std::cout << "Execute " <<distancia << "m_parado.bag file" << endl<< endl;
        }
        // Create a ROS publisher for each hole ROI identifier
        pub_ROIlocation = nh.advertise<depth_deco::ROI_identifier> ("ROI_location", 1);

        // Create a ROS publisher for the ROI cloud
        pub_ROIcloud = nh.advertise<sensor_msgs::PointCloud2> ("ROICloud", 1);

        // Create a ROS publisher for authentic Hole points
        pub_HoleCloud = nh.advertise<sensor_msgs::PointCloud2> ("HoleCloud", 1);

        // Create a ROS publisher for the Rest cloud (ROI without Hole)
        pub_RestCloud = nh.advertise<sensor_msgs::PointCloud2> ("RestCloud", 1);

        // Create a ROS publisher for the Cylinder obtained through RANSAC
        //pub_CylPoints = nh.advertise<sensor_msgs::PointCloud2> ("CylinderPoints", 1);

        // Create a ROS publisher for DE-Optmimized Cylinder Model
        pub_CylModel = nh.advertise<visualization_msgs::Marker> ("CylinderModel", 1);

        // Spin
        ros::spin ();
}
