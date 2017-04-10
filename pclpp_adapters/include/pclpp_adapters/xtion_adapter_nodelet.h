#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>


#ifndef PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
#define PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H

namespace pclpp_adapters {

    class XtionAdapterNodelet : public nodelet::Nodelet {

    public:
        XtionAdapterNodelet();
        ~XtionAdapterNodelet();

        void onInit();
    };

    PLUGINLIB_DECLARE_CLASS(pclpp_adapters, XtionAdapterNodelet, pclpp_adapters::XtionAdapterNodelet, nodelet::Nodelet)
}

#endif //PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
