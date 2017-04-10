#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"



namespace pclpp_adapters {

    XtionAdapterNodelet::XtionAdapterNodelet() {}

    XtionAdapterNodelet::~XtionAdapterNodelet() {}

    void XtionAdapterNodelet::onInit() {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        NODELET_INFO("XtionAdapterNodelet initialized");
    }

}