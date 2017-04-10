#include "../include/pclpp_adapters/kinect2_adapter_nodelet.h"

namespace pclpp_adapters {

    Kinect2AdapterNodelet::Kinect2AdapterNodelet() {}

    Kinect2AdapterNodelet::~Kinect2AdapterNodelet() {}

    void Kinect2AdapterNodelet::onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        NODELET_INFO("Kinect2AdapterNodelet initialized");
    }

}
