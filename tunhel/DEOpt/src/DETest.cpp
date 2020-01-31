// Differential Evolution Test Program
// Based on algorithms developed by Dr. Rainer Storn & Kenneth Price
// Written By: Lester E. Godwin
//             PushCorp, Inc.
//             Dallas, Texas
//             972-840-0208 x102
//             godwin@pushcorp.com
// Created: 6/8/98
// Last Modified: 6/8/98
// Revision: 1.0

#include <stdio.h>
#include "DESolver.h"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    

// Polynomial fitting problem
class PolynomialSolver : public DESolver
{
public:
        PolynomialSolver(int dim,int pop) : DESolver(dim,pop), count(0) {;}
        double EnergyFunction(double trial[],bool &bAtSolution);

private:
        int count;
};

double PolynomialSolver::EnergyFunction(double *trial,bool &bAtSolution)
{	
	//solution[0]=Cx cylinder centre
        //solution[1]=Cy
	//solution[2]=Cz
	//solution[3]=u cylinder vector
	//solution[4]=v
	//solution[5]=w
	//solution[6]=radius
	
	/*Qx=-0.312775;
        Qy=-0.456309;
        Qz=1.505;*/
	double value = 0;
        double Sx,Sy,Sz,sum,sum_desv, Px,Py,Pz, mod_qpxu, mod_u, temp, avg, desv, desvest, result=0;

        sum=0;
        sum_desv=0;
        vector <double> dists(hole_cloud->points.size ());

	std::cerr << "Centro trial: [" << trial[0]<< "," << trial[1]<< "," << trial[2] << "]" << std::endl;
        std::cerr << "Vector trial: [" << trial[3]<< "," << trial[4]<< "," << trial[5] << "]" << std::endl;

        for(uint i = 0; i < hole_cloud->points.size (); ++i) {

            Px=hole_cloud->points[i].x;
            Py=hole_cloud->points[i].y;
            Pz=hole_cloud->points[i].z;

            Sx=Px-trial[0];
            Sy=Py-trial[1];
            Sz=Pz-trial[2];

	    mod_qpxu=sqrt(pow(Sy*trial[5]-Sz*trial[4],2)+pow(Sz*trial[3]-Sx*trial[5],2)+pow(Sx*trial[4]-Sy*trial[3],2));
            mod_u=sqrt(pow(trial[3],2)+pow(trial[4],2)+pow(trial[5],2));
            temp=mod_qpxu/mod_u;
            dists[i]=temp;
            //std::cerr << "Distancia:" << temp << std::endl;
            sum+=temp;
         }

         avg=sum/hole_cloud->points.size ();
         //std::cerr << "Radio:" << avg << std::endl;
	 
         for(int j=0;j<dists.size();j++){
            desv=pow(dists[j]-avg,2);
            sum_desv += desv;
         }

         desvest=sqrt(sum_desv/dists.size());
         result=desvest;

         
        if (count++ % nPop == 0)
                printf("%d %lf\n",count / nPop + 1,Energy());

        return(result);
}

#define N_DIM 6
#define N_POP 3
#define MAX_GENERATIONS	10


void cloud_cb (sensor_msgs::PointCloud2ConstPtr cloud_msg)
{ 
  pcl::fromROSMsg (*cloud_msg, *hole_cloud);
  
  double Px=0,Py=0,Pz=0;

  double minPx=10000,minPy=10000;
  double maxPx=-10000,maxPy=-10000;

  for(uint i = 0; i < hole_cloud->points.size (); ++i) {

            Px=hole_cloud->points[i].x;
		if (Px<minPx) minPx=Px;
                if (Px>maxPx) maxPx=Px;
            Py=hole_cloud->points[i].y;
		if (Py<minPy) minPy=Py;
                if (Py>maxPy) maxPy=Py;
            /*Pz=hole_cloud->points[i].z;
		if (Pz<minPz) minPz=Pz;
                if (Pz>maxPz) maxPz=Pz;*/
  }

  std::cerr<<"Px["<<minPx<<","<<maxPx<<"]"<< endl;
  std::cerr<<"Py["<<minPy<<","<<maxPy<<"]"<< endl;	
   
  double min[N_DIM];
  double max[N_DIM];
  int i;

  PolynomialSolver solver(N_DIM,N_POP);

  max[0] = maxPx;
  min[0] = minPx;
  max[1] = maxPy;
  min[1] = minPy;
  max[2] = 1.2;
  min[2] = 1.7;
  for (i=3;i<5;i++)
        {
                max[i] =  0.15;
                min[i] = -0.15;
        }
  max[5] =  1;
  min[5] = 0.8;
  
  printf("Setup\n\n");	
  solver.Setup(min,max,stBest1Exp,0.9,1.0);

  printf("Solving...\n\n");
  solver.Solve(MAX_GENERATIONS);

  double *solution = solver.Solution();



  std::cerr<<"Best solution"<< endl;
  std::cerr<<"Hole Center:["<< solution[0] << "," << solution[1] << "," << solution[2] << "]" << endl;

  std::cerr<<"Cylinder vector:["<< solution[3] << "," << solution[4] << "," << solution[5] << "]" << endl;

//std::cerr<<"Radius:"<< solution[6] << endl;

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "DE_node");
  ros::NodeHandle nh;

  //Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/HoleCloud", 1, cloud_cb);

  // Spin
  ros::spin ();
}
