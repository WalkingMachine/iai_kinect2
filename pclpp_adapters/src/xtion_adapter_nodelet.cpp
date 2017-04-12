#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"

#include <libfreenect2/libfreenect2.hpp>

namespace pclpp_adapters {

    XtionAdapterNodelet::XtionAdapterNodelet() {}

    XtionAdapterNodelet::~XtionAdapterNodelet() {}

    void XtionAdapterNodelet::onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        NODELET_INFO("XtionAdapterNodelet initialized");

        //lp.reset(new openni2_wrapper::OpenNI2Driver(getNodeHandle(), getPrivateNodeHandle()));
    }

}