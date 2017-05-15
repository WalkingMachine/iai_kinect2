
#ifndef WM_PCL_PREPROCESSING_VTK_VIEWER_NODELET_H
#define WM_PCL_PREPROCESSING_VTK_VIEWER_NODELET_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
//#include <kinect2_bridge/kinect2_definitions.h>

namespace pclpp_viewer {

    class VTKViewerNodelet : public nodelet::Nodelet {

    public:

        enum Mode
        {
            IMAGE = 0,
            CLOUD,
            BOTH
        };

        VTKViewerNodelet(const bool useExact = false, const bool useCompressed = false);

        ~VTKViewerNodelet();

        int main(int argc, char **argv);

    private:
        void onInit();
        void run(const Mode mode);
        void start(const Mode mode);
        void stop();
        void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                      const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                      const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
        void imageViewer();
        void cloudViewer();
        void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *);
        void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;
        void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;
        void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
        void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out);
        void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
        void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored);
        void createLookup(size_t width, size_t height);

        std::mutex lock;

        std::string topicColor, topicDepth;
        const bool useExact, useCompressed;

        bool updateImage, updateCloud;
        bool save;
        bool running;
        size_t frame;
        const uint32_t queueSize;

        cv::Mat color, depth;
        cv::Mat cameraMatrixColor, cameraMatrixDepth;
        cv::Mat lookupX, lookupY;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

        ros::NodeHandle nh;
        ros::AsyncSpinner spinner;
        image_transport::ImageTransport it;
        image_transport::SubscriberFilter *subImageColor, *subImageDepth;
        message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

        message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
        message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

        std::thread imageViewerThread;
        Mode mode;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
        pcl::PCDWriter writer;
        std::ostringstream oss;
        std::vector<int> params;

    };

    PLUGINLIB_DECLARE_CLASS(pclpp_viewer, VTKViewerNodelet, pclpp_viewer::VTKViewerNodelet, nodelet::Nodelet);
}

#endif //WM_PCL_PREPROCESSING_VTK_VIEWER_NODELET_H
