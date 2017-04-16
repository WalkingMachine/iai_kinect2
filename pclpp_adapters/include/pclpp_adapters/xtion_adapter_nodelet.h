#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


#include <openni2_camera/openni2_driver.h>

#include <cv.hpp>

#ifndef PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
#define PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H


namespace pclpp_adapters {

    class XtionAdapterNodelet : public nodelet::Nodelet {

    public:
        XtionAdapterNodelet();
        ~XtionAdapterNodelet();

        int main(int argc, char **argv);

    private:

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

        void onInit();
        void nodeletCallback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                      const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
        void nodeCallback(sensor_msgs::Image::ConstPtr imageColor, sensor_msgs::Image::ConstPtr depth,
                          sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);

        const uint32_t queueSize;
        const bool useExact, useCompressed;
        std::string imageRGBTopic, imageRGBCameraInfoTopic, imageDepthTopic, imageDepthCameraInfoTopic;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::SubscriberFilter *subImageColor, *subImageDepth;
        message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
        message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
    };

    PLUGINLIB_DECLARE_CLASS(pclpp_adapters, XtionAdapterNodelet, pclpp_adapters::XtionAdapterNodelet, nodelet::Nodelet)
}

#endif //PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
