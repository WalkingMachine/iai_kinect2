#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"


/*
 *
 * http://ros-users.122217.n3.nabble.com/debugging-nodelets-td2370662.html
 *
 */

namespace pclpp_adapters {

    XtionAdapterNodelet::XtionAdapterNodelet()
    : nh("~"), it(nh), queueSize(10), useExact(true), useCompressed(false)
    {
        nh.getParam("/xtion_adapter/topic_rgb_image", imageRGBTopic);
        nh.getParam("/xtion_adapter/topic_rgb_camera_info", imageRGBCameraInfoTopic);
        nh.getParam("/xtion_adapter/topic_depth_image", imageDepthTopic);
        nh.getParam("/xtion_adapter/topic_depth_camera_info", imageDepthCameraInfoTopic);
    }

    XtionAdapterNodelet::~XtionAdapterNodelet() {}

    void XtionAdapterNodelet::onInit() {

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, imageRGBTopic, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, imageDepthTopic, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, imageRGBCameraInfoTopic, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, imageDepthCameraInfoTopic, queueSize);

        syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
        syncApproximate->registerCallback(boost::bind(&XtionAdapterNodelet::nodeletCallback, this, _1, _2, _3, _4));

        NODELET_INFO("XtionAdapterNodelet initialized");
    }

    void XtionAdapterNodelet::nodeletCallback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
        NODELET_INFO("CALLBACK %p %p %p %p", imageColor, imageDepth, cameraInfoColor, cameraInfoDepth);
    }

    void XtionAdapterNodelet::nodeCallback(sensor_msgs::Image::ConstPtr imageColor, sensor_msgs::Image::ConstPtr imageDepth,
                                           sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
        NODELET_INFO("CALLBACK %p %p %p %p", imageColor, imageDepth, cameraInfoColor, cameraInfoDepth);
    }


    int XtionAdapterNodelet::main(int argc, char **argv) {


        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, imageRGBTopic, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, imageDepthTopic, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, imageRGBCameraInfoTopic, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, imageDepthCameraInfoTopic, queueSize);

        syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
        syncApproximate->registerCallback(boost::bind(&XtionAdapterNodelet::nodeCallback, this, _1, _2, _3, _4));
        ROS_INFO("XtionAdapterNodelet initialized as node");

        ros::spin();

        return 0;
    }
}

