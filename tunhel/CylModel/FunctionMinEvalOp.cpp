#include <ecf/ECF.h>
#include "FunctionMinEvalOp.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

void FunctionMinEvalOp::registerParameters(StateP state)
{
	state->getRegistry()->registerEntry("function", (voidP) (new uint(1)), ECF::UINT);
}


bool FunctionMinEvalOp::initialize(StateP state)
{
	voidP sptr = state->getRegistry()->getEntry("function"); // get parameter value
	iFunction_ = *((uint*) sptr.get()); // convert from voidP to user defined type

        return true;
}


FitnessP FunctionMinEvalOp::evaluate(IndividualP individual)
{
	// evaluation creates a new fitness object using a smart pointer
	// in our case, we try to minimize the function value, so we use FitnessMin fitness (for minimization problems)
	FitnessP fitness (new FitnessMin);

	// we define FloatingPoint as the only genotype (in the configuration file)
	FloatingPoint::FloatingPoint* gen = (FloatingPoint::FloatingPoint*) individual->getGenotype().get();
        FloatingPoint::FloatingPoint* gen1 = (FloatingPoint::FloatingPoint*) individual->getGenotype().get();
        FloatingPoint::FloatingPoint* gen2 = (FloatingPoint::FloatingPoint*) individual->getGenotype().get();
        // (you can also use boost smart pointers:)
	//FloatingPointP gen = boost::dynamic_pointer_cast<FloatingPoint::FloatingPoint> (individual->getGenotype());

	// alternative encoding: Binary Genotype
        //Binary::Binary* gen = (Binary::Binary*) individual->getGenotype().get();
	//BinaryP gen = boost::dynamic_pointer_cast<Binary::Binary> (individual->getGenotype());

	// we implement the fitness function 'as is', without any translation
	// the number of variables is read from the genotype itself (size of 'realValue' vactor)
        double value = 0;
        double u,v,w,Qx,Qy,Qz,Sx,Sy,Sz,sum,sum_desv, Px,Py,Pz, mod_qpxu, mod_u, temp, avg, desv, desvest;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> ("hole.pcd", *cloud);

        u = gen->realValue[0];
        v = gen1->realValue[0];
        w = gen2->realValue[0];

        Qx=-0.312775;
        Qy=-0.456309;
        Qz=1.505;
        sum=0;
        sum_desv=0;
        vector <double> dists(cloud->points.size ());

        for(uint i = 0; i < cloud->points.size (); ++i) {

            Px=cloud->points[i].x;
            Py=cloud->points[i].y;
            Pz=cloud->points[i].z;

            Sx=Px-Qx;
            Sy=Py-Qy;
            Sz=Pz-Qz;

            mod_qpxu=sqrt(pow(Sy*w-Sz*v,2)+pow(Sz*u-Sx*w,2)+pow(Sx*v-Sy*u,2));
            mod_u=sqrt(pow(u,2)+pow(v,2)+pow(w,2));
            temp=mod_qpxu/mod_u;
            dists[i]=temp;
            sum+=temp;
         }

         avg=sum/cloud->points.size ();
         std::cerr << "Radio:" << avg << std::endl;

         for(int j=0;j<dists.size();j++){
            desv=pow(dists[j]-avg,2);
            sum_desv += desv;
         }

         desvest=sqrt(sum_desv/dists.size());
         value=desvest;

         fitness->setValue(value);
	return fitness;
}
