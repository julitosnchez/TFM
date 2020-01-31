Software to perform 3D laser scans with a dynamixel motor and a laser mounted on top of it. Be aware that the assembling of the information is done based on the assumption that the laser turns exactly around the axis of the motor. Also, modify system transformations and configuration files as needed.

There are two versions (in different branches) of the software, the simplest one is asynchronous. Based on the assumptions that the motor spins at a constant velocity and the laser data adquisition is perfectly done with a constant frequency. Therefore, the needed transformation to assemble the laser readings are known in advance. This version is ready for a 2D Hokuyo laser (http://wiki.ros.org/hokuyo_node/).

In the second version the information is synchronised using the tf package and ROS timestamps through laser assembler node (http://wiki.ros.org/laser_assembler). This version is both ready to be used with a 2D Hokuyo and a 3D Velodyne (http://wiki.ros.org/velodyne).

Choose the version you need according to your Ubuntu/ROS version and needs.

