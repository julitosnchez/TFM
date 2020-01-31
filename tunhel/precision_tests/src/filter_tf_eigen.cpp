#include <ros/ros.h>   
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

int main(int argc, char **argv) {
  ros::init(argc, argv, "filtro1_tf");
  ros::NodeHandle nh;
                                 
  unsigned int size = 500; 
  // For quaternion averaging
  Eigen::Matrix<double, 4, 1> quat; 
  Eigen::Matrix4d M;                     

  std::vector<tf::StampedTransform> tf_window;
  tf_window.resize(size);
  tf::StampedTransform transform;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;

  unsigned int count = 0;
  unsigned int start = 0;



  while(ros::ok())  {
    
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/camera_rgb_frame", "/ar_marker_10", now, ros::Duration(15));
    listener.lookupTransform( "/camera_rgb_frame", "/ar_marker_10", ros::Time(0), transform);
    
    tf_window[count] = transform;
    
    count= count+1;
    std::cout << count << std::endl;
    if (count > size-1) {
      count=0;  
      start = 1;
    }
    if (start) {
      tf::StampedTransform transf_res;
      transform.setIdentity();
      for (int i=0; i<size; i++) { // se suman posiciones y orientaciones

        transform.setOrigin(transform.getOrigin() + tf_window[i].getOrigin());
	tf::Quaternion qo= tf_window[i].getRotation();
        std::cout << "Qo: " << qo.x() << "," << qo.y() << "," << qo.z() << "," << qo.w() << "]" << std::endl;
	quat << qo.x(), qo.y(), qo.z(), qo.w();	
        //transform.setRotation(transform.getRotation()*tf_window[i].getRotation() );
	M = M + (quat*quat.transpose()) * (1/size);

      }

         
      
      

      // Average Quat Eigen
	Eigen::EigenSolver<Eigen::Matrix4d> egv(M, true);
	double eigen_value;
	unsigned int index_max_eigenvalue = 0;
	double max_eigenvalue = 0.0;
	for (unsigned int k=0; k<M.rows(); k++) {  
        	eigen_value = std::real(egv.eigenvalues().col(0)[k]);
		if ( eigen_value > max_eigenvalue ) {
	 	     index_max_eigenvalue = k;
	 	     max_eigenvalue = eigen_value;
	 	}
	}	

      tf::Quaternion q( tfScalar(std::real(egv.eigenvectors().col(index_max_eigenvalue)[0])),
                            tfScalar(std::real(egv.eigenvectors().col(index_max_eigenvalue)[1])),
                            tfScalar(std::real(egv.eigenvectors().col(index_max_eigenvalue)[2])),
                            tfScalar(std::real(egv.eigenvectors().col(index_max_eigenvalue)[3])) );


      /*//Average Quat Slerp
      tf::Quaternion q;//(0,0,0,1);	
      q.setRPY(0,0,0);
      q.slerp(transform.getRotation(), (float)1/(float)size);*/


      			

      std::cout << "RESULTADO: " << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "]" << std::endl;
      transf_res.setOrigin(transform.getOrigin()/size);
      transf_res.setRotation(q); 
      br.sendTransform(tf::StampedTransform(transf_res, ros::Time::now(), "/camera_rgb_frame", "/filter_marker10" ) );
    }
  }
  return 0;
}
