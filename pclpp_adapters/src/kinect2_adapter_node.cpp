#include "../include/pclpp_adapters/kinect2_adapter_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xtion_adapter");
    pclpp_adapters::Kinect2AdapterNodelet kinect2Adapter;


    ROS_INFO("HELLO WORLD -- xtion");

    return kinect2Adapter.main(argc, argv);
}
