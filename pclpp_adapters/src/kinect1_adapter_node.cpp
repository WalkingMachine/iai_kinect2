#include "../include/pclpp_adapters/kinect1_adapter_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xtion_adapter");
    pclpp_adapters::Kinect1AdapterNodelet kinect1Adapter;


    ROS_INFO("HELLO WORLD -- kinect1");

    return kinect1Adapter.main(argc, argv);
}

