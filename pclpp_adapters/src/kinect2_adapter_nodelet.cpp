#include "../include/pclpp_adapters/kinect2_adapter_nodelet.h"

#include <libfreenect2/libfreenect2.hpp>

namespace pclpp_adapters {

    Kinect2AdapterNodelet::Kinect2AdapterNodelet() {};

    Kinect2AdapterNodelet::~Kinect2AdapterNodelet() {}

    void Kinect2AdapterNodelet::onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        NODELET_INFO("Kinect2AdapterNodelet initialized");

        //std::cout << "1";
        //libfreenect2::Freenect2 freenect2;
        //std::cout << "2";
        //libfreenect2::Freenect2Device *dev = 0;
        //std::cout << "3";
        //libfreenect2::PacketPipeline *pipeline = 0;
        //std::cout << "4";
        //std::cout << freenect2.enumerateDevices();

    }

    int Kinect2AdapterNodelet::main(int argc, char **argv) {

        ROS_INFO("Kinect2AdapterNodelet initialized as node");
        return 0;
    }

}

