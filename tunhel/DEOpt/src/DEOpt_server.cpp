#include "ros/ros.h"
#include "DEOpt/DEOpt.h"

bool optimize(DEOpt::DEOpt::Request  &req,
         DEOpt::DEOpt::Response &res)
{
  res.sum = req.N_DIM + req.POP;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.N_DIM, (long int)req.POP);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DEOpt_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("DEOpt", optimize);
  ROS_INFO("Ready to optimize");
  ros::spin();

  return 0;
}
