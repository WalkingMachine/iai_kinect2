#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#ifndef PCL_PREPROCESSING_KINECT1_ADAPTER_NODELET_H
#define PCL_PREPROCESSING_KINECT1_ADAPTER_NODELET_H

namespace pclpp_adapters {

    class Kinect1AdapterNodelet : public nodelet::Nodelet {
    public:
        Kinect1AdapterNodelet();
        ~Kinect1AdapterNodelet();

        void onInit();
    };

    PLUGINLIB_DECLARE_CLASS(pclpp_adapters, Kinect1AdapterNodelet, pclpp_adapters::Kinect1AdapterNodelet, nodelet::Nodelet)
}

#endif //PCL_PREPROCESSING_KINECT1_ADAPTER_NODELET_H
