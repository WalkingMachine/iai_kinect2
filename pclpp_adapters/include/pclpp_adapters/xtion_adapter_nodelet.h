#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <openni2_camera/openni2_driver.h>

#include <cv.hpp>

#ifndef PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
#define PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H


namespace pclpp_adapters {

    class XtionAdapterNodelet : public nodelet::Nodelet {

    public:
        XtionAdapterNodelet();
        ~XtionAdapterNodelet();

    private:
        void onInit();

        //boost::shared_ptr<openni2_wrapper::OpenNI2Driver> lp;
    };

    PLUGINLIB_DECLARE_CLASS(pclpp_adapters, XtionAdapterNodelet, pclpp_adapters::XtionAdapterNodelet, nodelet::Nodelet)
}

#endif //PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
