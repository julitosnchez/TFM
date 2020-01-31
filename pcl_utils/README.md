# pcl_utils 
pcl_utils metapackage that contains PCL based or PCL\_ROS based (indigo) helpers and examples, which include:

1. cloud\_io:

read\_pcd\_pcl: reading a cloud saved as a .pcd file and constantly publishes it by a ROS topic as a PCL type (PointXYZRGB).


read\_pcd\_pcl: reading a cloud saved as a .pcd file and constantly publishes it by a ROS topic as a ROS type (PointCloud2).


read\_multi\_pcd\: multi-pcd file reader. Execute a loop that reads PDC files with increasing naming and publishes it by a ROS topic.


png2image: reads an image in sensor_msgs::Image format using OpenCV reader and converter. then publishes it in a ROS topic. Also publishes a topic with default calibration parameters for Kinect. 


save\_pcd: listens to a ROS topic (PointCloud2) and saves the information in .pcd files with increasing naming.


camera\_kinect\_info\_pub: publishes a ROS topic with default calibration parameters for Kinect as sensor_msgs::CameraInfo. 


camera\_xtion\_info\_pub: publishes a ROS topic with default calibration parameters for Xtion as sensor_msgs::CameraInfo. 


2. pcl\_euc\_seg:

Contains 2 examples of euclidean segmentation of objects using the PCL library. Two different launch files are provided, one of which also performs an octomap computing. Besides, examples PCDs, bag files and Rviz config files are included.


