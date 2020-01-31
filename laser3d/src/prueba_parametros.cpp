#include<ros/ros.h>
#include<iostream>

using namespace std;

 int main(int argc, char *argv[])
 {  
    std::string param;
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh("~");
    nh.getParam("param", param);
    ROS_INFO("Got parameter : %s", param.c_str());

    if(param.compare("blue") == 0)
    {
        cout << "blue " << endl;
    }
    else if(param.compare("green") == 0)
    {
        cout << "green " << endl;
    }
    else
    {
        cout << "Don't run anything !! " << endl;
    }
    return 0;
}
