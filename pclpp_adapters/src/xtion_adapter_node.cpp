#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xtion_adapter");
    pclpp_adapters::XtionAdapterNodelet xtionAdapter;


    ROS_INFO("HELLO WORLD -- xtion");

    return xtionAdapter.main(argc, argv);
}
