/*
 * Calibration3DMarker.cpp
 *
 *  Created on: 2.4.2014
 *      Author: ivelas
 */

#include "but_calibration_camera_velodyne/Calibration3DMarker.h"

#include <ros/assert.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Calibration.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include <but_calibration_camera_velodyne/Image.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>




using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;


float A,B,C,D; // Variables globales que definen los coeficientes del plano

//struct Spheres {

//	vector<PointXYZ> spheres_centers;
//  vector<float> radiuses;
//};


namespace but_calibration_camera_velodyne {

Calibration3DMarker::Calibration3DMarker(cv::Mat _frame_gray, cv::Mat _P, ::PointCloud<Velodyne::Point> _pc,
                                         float _circ_distance, float _radius) :
    frame_gray(_frame_gray), P(_P), pc(_pc), circ_distance(_circ_distance), radius(_radius)
{

   

  Velodyne::Velodyne scan(pc);

  scan.save("nube_girada.pcd");

  scan.getRings();
  scan.intensityByRangeDiff();
  PointCloud<Velodyne::Point> visible_cloud;
  scan.project(P, Rect(0, 0, 1920, 1080), &visible_cloud);
  
  Velodyne::Velodyne visible_scan(visible_cloud);
  visible_scan.save("vis_scan.pcd"); // Nube completa (Pared y plano frontal)
  
  //********  Calculando plano sobre nube entera (Nota: El plano que calcula es el de la pared) *********************************************************************************************************
  PointCloud<PointXYZ>::Ptr vis_scan_ptr(visible_scan.toPointsXYZ());

  SampleConsensusModelPlane<PointXYZ>::Ptr model_pl(new ::SampleConsensusModelPlane<PointXYZ>(vis_scan_ptr));

  RandomSampleConsensus<PointXYZ> ransac2(model_pl);
  ransac2.setDistanceThreshold(0.02); 
  ransac2.computeModel();

  std::vector<int> inliers_indicies2; // Calculamos los INLIERS con el fin de obtener los OUTLIERS que son los que nos interesan. Nota: Inliers <=> Pared
	ransac2.getInliers(inliers_indicies2);


			//---------- Extrayendo OUTLIERS para QUITAR pared --------
			std::vector<int> outliers;
			::pcl::PointCloud<Velodyne::Point> nf = visible_scan.getPointCloud();

				Velodyne::Velodyne visible_scan_no_wall; // Como vamos a reducir el tamaño de la nube, quedandonos con Outliers...cremos nueva "visivle_scan" 
																								// para arrastrar los valores de intensidad de los puntos que nos interesan. Nota: Necesitamos intensidad para el posterior filtrado (Bordes)
				for(int i=0;i<(vis_scan_ptr->width) * (vis_scan_ptr->height);i++) {
					std::vector<int>::iterator it = std::find(inliers_indicies2.begin(), inliers_indicies2.end(), i);
					//					if(nf.at(i).x != vis_scan_ptr->at(i).x or nf.at(i).y != vis_scan_ptr->at(i).y or nf.at(i).z != vis_scan_ptr->at(i).z)
					//						cout << "Mal asunto amigo" << endl;
					if(it == inliers_indicies2.end()) { // NO ha encontrado inlier, con lo cual  es OUTLIER
						outliers.push_back(i);
						visible_scan_no_wall.push_back(nf.at(i));
					}
				}	

				cout << "Inliers: " << inliers_indicies2.size() << " " << "Outliers: " << outliers.size() << endl;
				cout << "Y el total son " << (vis_scan_ptr->width) * (vis_scan_ptr->height)  << " puntos" << std::endl;
				cout << "La nueva nube con intensidades  " << visible_scan_no_wall.size() << endl;

				copyPointCloud<PointXYZ>(*vis_scan_ptr, outliers, *vis_scan_ptr);

				cout << "La nueva nube con outliers solo tiene " << (vis_scan_ptr->width) * (vis_scan_ptr->height)  << " puntos" << std::endl << std::endl;



	// ******************** Plano para OUTLIERS (Corcho) **************************************************************************************************************************************************
	SampleConsensusModelPlane<PointXYZ>::Ptr model_pl3(new ::SampleConsensusModelPlane<PointXYZ>(vis_scan_ptr));

  RandomSampleConsensus<PointXYZ> ransac3(model_pl3);
  ransac3.setDistanceThreshold(0.01); 
  ransac3.computeModel();

  Eigen::VectorXf coeff3;
  ransac3.getModelCoefficients (coeff3);
  std::cout << "Plano patron (A,B,C,D): "<< coeff3[0] <<","<< coeff3[1] <<"," <<coeff3[2] << "," << coeff3[3] << std::endl;

	A = coeff3[0];
	B = coeff3[1];
	C = coeff3[2];
	D = coeff3[3];

		vector<int> inliers;  //Ahora, nos quedamos con los INLIERS que son los que contienen el plano patron, el resto es basura
		ransac3.getInliers(inliers);

		copyPointCloud<PointXYZ>(*vis_scan_ptr, inliers, *vis_scan_ptr); // Reducimos y nos quedamos con PATRON

		cout << "La nueva nube con plano patron solo tiene " << (vis_scan_ptr->width) * (vis_scan_ptr->height)  << " puntos" << std::endl;	

		Velodyne::Velodyne visible_scan_patron;
		::pcl::PointCloud<Velodyne::Point> nf2 = visible_scan_no_wall.getPointCloud();
		for(int i=0;i<inliers.size();i++)
			visible_scan_patron.push_back(nf2.at(inliers[i]));

		cout << "La nueva nube con plano patron (Intensidades)  tiene " << visible_scan_patron.size()  << " puntos" << std::endl;	


		visible_scan_patron.save("cloud_final_d_intensidad.pcd");


		std::ostringstream ostia;
		ostia << "cloud_final_d.pcd"; // Guardando nube con outliers
		string vatia=ostia.str();
		const char *filenamtia=vatia.c_str();
		pcl::io::savePCDFileASCII (filenamtia, *vis_scan_ptr);


//-------------------------------------------------------------------------------------------------------------------------------------------------- 		
	visible_scan.normalizeIntensity();
  visible_scan.save("vis_scan_norm.pcd");
  Velodyne::Velodyne thresholded_scan = visible_scan.threshold(0.2); //AQUI 0.2

  PointCloud<PointXYZ>::Ptr xyz_cloud_ptr(thresholded_scan.toPointsXYZ());

	thresholded_scan.save("cloud_filtered_intensidad.pcd");


//Writing velodyne .pcd
	 std::ostringstream ost;
/*sensor_msgs::PointCloud2 msg_pc;
	toROSMsg (pc, msg_pc);	*/
		ost << "velodyne_final.pcd";
		string vat=ost.str();
		const char *filenamt=vat.c_str();
		pcl::io::savePCDFileASCII (filenamt, *xyz_cloud_ptr);

		SampleConsensusModelPlane<PointXYZ>::Ptr model_p(new ::SampleConsensusModelPlane<PointXYZ>(xyz_cloud_ptr));

		RandomSampleConsensus<PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(0.02); 
		ransac.computeModel();

		std::vector<int> inliers_indicies;
		ransac.getInliers(inliers_indicies);

		//		Eigen::VectorXf coeff;
		//		ransac.getModelCoefficients (coeff);
		////		cout << "\nPlano: A ="<< coeff[0] <<";B ="<< coeff[1] <<";C =" <<coeff[2] << ";D =" << coeff[3] << endl;


		copyPointCloud<PointXYZ>(*xyz_cloud_ptr, inliers_indicies, plane);


 //Writing plane .pcd
  std::ostringstream os;
  os << "plane.pcd";
  string va=os.str();
  const char *filenam=va.c_str();
  pcl::io::savePCDFileASCII (filenam, plane);


  // ---------------- REMOVE LINES ----------------

  for (int i = 0; i < 2; i++)
  {
    PointCloud<PointXYZ>::Ptr plane_ptr(new PointCloud<PointXYZ>(plane));
    SampleConsensusModelLine<PointXYZ>::Ptr model_l(
        new SampleConsensusModelLine<PointXYZ>(plane_ptr));
    RandomSampleConsensus<PointXYZ> ransac_l(model_l);
    ransac_l.setDistanceThreshold(0.005);
    ransac_l.computeModel();
    vector<int> line_inliers;
    ransac_l.getInliers(line_inliers);
    if (line_inliers.empty())
    {
      continue;
    }
    PointCloud<PointXYZ> plane_no_line;
    remove_inliers(*plane_ptr, line_inliers, plane_no_line);
    plane = plane_no_line;
  }

 //Writing plane .pcd
        std::ostringstream osp;
        osp << "plane_noline.pcd";
        string vap=osp.str();
        const char *filenamp=vap.c_str();
        pcl::io::savePCDFileASCII (filenamp, plane);

}

bool Calibration3DMarker::detectCirclesInImage(vector<Point2f> &centers, vector<float> &radiuses)
{
  Image::Image img(frame_gray);
  /*Image::Image img_edge(img.computeEdgeImage());
  return img_edge.detect4Circles(Calibration3DMarker::CANNY_THRESH, Calibration3DMarker::CENTER_THRESH_DISTANCE, centers, radiuses);*/
  return img.detect4Blobs(centers, radiuses);
  }

bool Calibration3DMarker::detectCirclesInPointCloud(vector<Point3f> &centers, vector<float> &radiuses, PointCloud<PointXYZ> & detectionPC)
{
  PointCloud<PointXYZ>::Ptr detection_cloud(new PointCloud<PointXYZ>);
  *detection_cloud += this->plane;

/* Rotating -M_PI/2 to compensate the rotation done in calibration-node. JULIO */
//  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//	transform_2.translation() << 0.0, 0.0, 0.0;
//	transform_2.rotate (Eigen::AngleAxisf (-M_PI/2, Eigen::Vector3f::UnitX()));
//	PointCloud<PointXYZ>::Ptr transformed_cloud (new PointCloud<PointXYZ> ());

//	pcl::transformPointCloud (*detection_cloud, *transformed_cloud, transform_2);
//	*detection_cloud = *transformed_cloud;


//Velodyne::Velodyne(*detection_cloud).transform(0, 0, 0, M_PI / 2, 0, 0);
//	PointCloud<PointXYZ> pcng(V);

  //Writing plane .pcd
        std::ostringstream os;
        os << "plane.pcd";
        string va=os.str();
        const char *filenam=va.c_str();
        pcl::io::savePCDFileASCII (filenam, *detection_cloud);

  float tolerance = 0.03; // 3cm
  int round = 1;
  vector<PointXYZ> spheres_centers;
  bool detected = false;
  for (int iterations = 0; iterations < 64; iterations++)
  {
    cerr << endl << " =========== ROUND " << round++ << " =========== "
     << endl << endl;
     cerr << "detection_cloud size: " << detection_cloud->size() << endl;
    spheres_centers = detect4spheres(detection_cloud, radiuses);
    
    std::cout << "Detected 3D circles: " << spheres_centers.size() << std::endl;
    if (spheres_centers.size() == 4)
    {
      std::cout << "Centro 1: ("<< spheres_centers[0].x <<","<< spheres_centers[0].y <<"," <<spheres_centers[0].z << ")" << std::endl;
	    std::cout << "Centro 2: ("<< spheres_centers[1].x <<","<< spheres_centers[1].y <<"," <<spheres_centers[1].z << ")" << std::endl;
	    std::cout << "Centro 3: ("<< spheres_centers[2].x <<","<< spheres_centers[2].y <<"," <<spheres_centers[2].z << ")" << std::endl;
	    std::cout << "Centro 4: ("<< spheres_centers[3].x <<","<< spheres_centers[3].y <<"," <<spheres_centers[3].z << ")" << std::endl;
      order4spheres(spheres_centers);
      /*cerr << "ordered centers: " << endl;
       for (size_t i = 0; i < spheres_centers.size(); i++) {
       cerr << spheres_centers[i] << endl;
       }*/
      if (verify4spheres(spheres_centers, this->circ_distance, tolerance))
      {
				cout << "Refinando los centros de las esferas con el plano" << endl;
        spheres_centers = refine4centers(spheres_centers, detection_cloud);
        detected = true;
        break;
      }
    }
    vector<PointXYZ> possible_centers = generate_possible_centers(spheres_centers, this->circ_distance);
    //generate_possible_points(this->plane, detection_cloud, possible_centers, this->circ_distance, 0.01);
  }

  if (!detected)
  {
    return false;
  }

  for (size_t i = 0; i < spheres_centers.size(); i++)
  {
    centers.push_back(Point3f(spheres_centers[i].x, spheres_centers[i].y, spheres_centers[i].z));
  }

	detectionPC = *detection_cloud;

  return true;
}

vector<PointXYZ> Calibration3DMarker::detect4spheres(PointCloud<PointXYZ>::Ptr plane,
                                                          vector<float> &radiuses)
{

  radiuses.clear();
  vector<PointXYZ> centers;
  std::vector<int> inliers_indicies;
  PointCloud<PointXYZ> *four_spheres = new PointCloud<PointXYZ>();
  float tolerance = 0.01;
  for (int i = 0; i < 4; i++)
  {
    SampleConsensusModelSphere<PointXYZ>::Ptr model_s(
        new SampleConsensusModelSphere<PointXYZ>(plane));
    model_s->setRadiusLimits(0.09, 0.11);
    RandomSampleConsensus<PointXYZ> ransac_sphere(model_s);
    ransac_sphere.setDistanceThreshold(tolerance);
    ransac_sphere.computeModel();
    inliers_indicies.clear();
    ransac_sphere.getInliers(inliers_indicies);

    if (inliers_indicies.size() == 0)
    {
      continue;
    }
    Eigen::VectorXf coeficients;
    ransac_sphere.getModelCoefficients(coeficients);
    //cerr << i + 1 << ". circle: " << coeficients << endl << endl;

    PointCloud<PointXYZ>::Ptr outliers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr inliers(new PointCloud<PointXYZ>);
    remove_inliers<PointXYZ>(*plane, inliers_indicies, *outliers);
    copyPointCloud<PointXYZ>(*plane, inliers_indicies, *inliers);
    plane = outliers;
//    Velodyne.view(plane);

    *four_spheres += *inliers;
    PointXYZ middle(coeficients(0), coeficients(1), coeficients(2));
    four_spheres->push_back(middle);
    centers.push_back(middle);

    float radius = coeficients(3);
    radiuses.push_back(radius);


//		cout << "RADIUS SPHERE " << radius << endl;
  }
  PointCloud<PointXYZ>::Ptr four_spheres_ptr(four_spheres);
  return centers;
}

/*
 * Indexes of circles in marker:
 *
 * 0 1
 * 2 3
 */
bool orderX(PointXYZ p1, PointXYZ p2)
{
  return p1.x < p2.x;
}
bool orderY(PointXYZ p1, PointXYZ p2)
{
  return p1.y < p2.y;
}

void Calibration3DMarker::order4spheres(vector<PointXYZ> &spheres_centers)
{
  ROS_ASSERT(spheres_centers.size() == 4);
  sort(spheres_centers.begin(), spheres_centers.end(), orderY);
  sort(spheres_centers.begin(), spheres_centers.begin() + 2, orderX);
  sort(spheres_centers.begin() + 2, spheres_centers.begin() + 4, orderX);
}


float euclid_dist(const PointXYZ p1, const PointXYZ p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}


bool Calibration3DMarker::verify4spheres(const vector<PointXYZ> &spheres_centers, float straight_distance, float delta)
{                                       //  (spheres_centers,                      this->circ_distance,       tolerance
  ROS_ASSERT(spheres_centers.size() == 4); 

  vector<pair<int, int> > neighbour_indexes;
  neighbour_indexes.push_back(pair<int, int>(0, 1));
  neighbour_indexes.push_back(pair<int, int>(1, 3));
  neighbour_indexes.push_back(pair<int, int>(3, 2));
  neighbour_indexes.push_back(pair<int, int>(2, 0));

  bool res = true;
  for (vector<pair<int, int> >::iterator neighbors = neighbour_indexes.begin(); neighbors < neighbour_indexes.end();
      neighbors++)
  {
    float error = abs(euclid_dist(spheres_centers[neighbors->first], spheres_centers[neighbors->second]) - straight_distance);
//    cerr << "error: " << error << endl;
    if (error > delta)
        res = false;
  }
  return res;
}

/*
 * All points around the all found centers:
 * x x x
 * x   x
 * x x x
 */
vector<PointXYZ> Calibration3DMarker::generate_possible_centers(const vector<PointXYZ> &spheres_centers,
                                                                     float straight_distance)
{
  vector<PointXYZ> possible_centers;

  for (vector<PointXYZ>::const_iterator c = spheres_centers.begin(); c < spheres_centers.end(); c++)
  {
    for (int dx = -1; dx <= 1; dx++)
    {
      for (int dy = -1; dy <= 1; dy++)
      {
        if (dx || dy)
        { // omitting found center (may be false detection)
          PointXYZ new_center = *c;
          new_center.x += dx * straight_distance;
          new_center.y += dy * straight_distance;
          possible_centers.push_back(new_center);
        }
      }
    }
  }

  return possible_centers;
}

void Calibration3DMarker::generate_possible_points(PointCloud<PointXYZ> &plane,
                                                   PointCloud<PointXYZ>::Ptr detection_cloud,
                                                   const vector<PointXYZ> &possible_centers, float radius,
                                                   float tolerance)
{

  detection_cloud->clear();
  for (PointCloud<PointXYZ>::iterator pt = plane.begin(); pt < plane.end(); pt++)
  {
    int votes = 0;
    for (vector<PointXYZ>::const_iterator center = possible_centers.begin(); center < possible_centers.end();
        center++)
    {
      if (euclid_dist(*pt, *center) < radius + tolerance)
      {
        votes++;
      }
    }
    if (votes > 0)
    {
      detection_cloud->push_back(*pt);
    }
  }
}

vector<PointXYZ> Calibration3DMarker::refine4centers(vector<PointXYZ> centers, PointCloud<PointXYZ>::Ptr detection_cloud)
{

  float z_coord = 0;
  for (PointCloud<PointXYZ>::iterator pt = detection_cloud->begin(); pt < detection_cloud->end(); pt++)
  {
    z_coord += pt->z;
  }
  z_coord /= detection_cloud->size();

  for (vector<PointXYZ>::iterator c = centers.begin(); c < centers.end(); c++)
  {
    c->z = z_coord;
  }

  return centers;
}

vector<PointXYZ> Calibration3DMarker::refine4centers_nuestro(vector<PointXYZ> centers, PointCloud<PointXYZ>::Ptr detection_cloud)
{

  float alpha;

  for (vector<PointXYZ>::iterator c = centers.begin(); c < centers.end(); c++)
  {

		alpha = (-A*c->x-B*c->y-C*c->z-D)/(A*A+B*B+C*C);

		cout << "X antes era " << c->x << " y ahora es " << c->x+A*alpha << endl;
		cout << "Y antes era " << c->y << " y ahora es " << c->y+B*alpha << endl;
		cout << "Z antes era " << c->z << " y ahora es " << c->z+C*alpha << endl;
		
		c->x = c->x+A*alpha;
	  c->y = c->y+B*alpha;
    c->z = c->z+C*alpha;
  }

  return centers;
}

}
