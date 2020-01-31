cmake_minimum_required(VERSION 2.8.3)
project(laser3d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  std_msgs
  dynamixel_msgs
  tf
  sensor_msgs
  pcl_ros
	message_generation
)

## System dependencies are found with CMake's conventions
find_package( SDL2 REQUIRED )
find_package( PCL 1.7 REQUIRED )
find_package( Boost 1.40 COMPONENTS )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   pololu_pos_timestamp.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(object_detection
#   src/${PROJECT_NAME}/object_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(bin_laser_pc_assemble_req src/laser_pc_request.cpp)

#add_executable(bin_pc_tran src/laser_pc_transform.cpp)

add_executable(bin_motor_state_pub_velodyne src/motor_tf_pub_velodyne.cpp)

add_executable(bin_point_cloud_converter_node src/pc_to_pc2.cpp)
add_executable(bin_imu_pub src/imu_tf_pub.cpp)
add_executable(bin_velodyne_imu src/velodyneToImu.cpp)

#add_executable(bin_laser_turn_req src/laser_pc_request_no_assemble.cpp)
add_executable(bin_pololu_plus_cloud src/pololu_plus_cloud.cpp)
add_executable(bin_imu_to_laser src/imu_to_laser.cpp)
#add_executable(bin_imu_to_imu2 src/imu_to_imu2.cpp)
#add_executable(bin_laser_matlab src/laser_cloud_matlab.cpp)
add_executable(bin_match_cloud src/matchCloudMatlab.cpp)
add_executable(bin_laser_pc_assemble_req_pololu src/laser_pc_request_pololu.cpp)
add_executable(bin_laser_pc_assemble_req_pololu_single src/laser_pc_request_pololu_single.cpp)
add_executable(bin_laser_pc_assemble_req_pololu_color src/laser_pc_request_pololu_colored.cpp)
add_executable(bin_prueba_param src/prueba_parametros.cpp)



## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(object_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(bin_pc_tran ${catkin_LIBRARIES} )

#target_link_libraries(bin_laser_pc_assemble_req ${catkin_LIBRARIES} )
target_link_libraries(bin_motor_state_pub_velodyne ${catkin_LIBRARIES} )
target_link_libraries(bin_point_cloud_converter_node
  ${catkin_LIBRARIES}
)
target_link_libraries(bin_imu_pub                    ${catkin_LIBRARIES} )
target_link_libraries(bin_pololu_plus_cloud                    ${catkin_LIBRARIES} )
#target_link_libraries(bin_laser_turn_req             ${catkin_LIBRARIES} )
target_link_libraries(bin_velodyne_imu             ${catkin_LIBRARIES} )
target_link_libraries(bin_imu_to_laser            ${catkin_LIBRARIES} )
#target_link_libraries(bin_imu_to_imu2                    ${catkin_LIBRARIES} )
#target_link_libraries(bin_laser_matlab                    ${catkin_LIBRARIES} )
target_link_libraries(bin_match_cloud                    ${catkin_LIBRARIES} )
target_link_libraries(bin_laser_pc_assemble_req_pololu                    ${catkin_LIBRARIES} )
target_link_libraries(bin_laser_pc_assemble_req_pololu_single                    ${catkin_LIBRARIES} )
target_link_libraries(bin_laser_pc_assemble_req_pololu_color                    ${catkin_LIBRARIES} )
target_link_libraries(bin_prueba_param                    ${catkin_LIBRARIES} )



