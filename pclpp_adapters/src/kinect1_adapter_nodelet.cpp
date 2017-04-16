#include "../include/pclpp_adapters/kinect1_adapter_nodelet.h"

namespace pclpp_adapters {

    Kinect1AdapterNodelet::Kinect1AdapterNodelet() {}

    Kinect1AdapterNodelet::~Kinect1AdapterNodelet() {}

    void Kinect1AdapterNodelet::onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        NODELET_INFO("Kinect1AdapterNodelet initialized");
    }

}

int main(int argc, char **argv)
{
    ROS_INFO("HELLO WORLD --- kinect1");
}
