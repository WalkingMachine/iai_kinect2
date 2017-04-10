#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#ifndef PCL_PREPROCESSING_KINECT2_ADAPTER_NODELET_H
#define PCL_PREPROCESSING_KINECT2_ADAPTER_NODELET_H

namespace pclpp_adapters {

    class Kinect2AdapterNodelet : public nodelet::Nodelet {
    public:
        Kinect2AdapterNodelet();
        ~Kinect2AdapterNodelet();

        void onInit();
    };

    PLUGINLIB_DECLARE_CLASS(pclpp_adapters, Kinect2AdapterNodelet, pclpp_adapters::Kinect2AdapterNodelet, nodelet::Nodelet)
}

#endif //PCL_PREPROCESSING_KINECT2_ADAPTER_NODELET_H
