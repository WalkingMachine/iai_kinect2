#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <boost/scoped_ptr.hpp>

#include <openni2_camera/openni2_driver.h>

#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
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
        void driverNodeletCallback(const sensor_msgs::Image::ConstPtr imageColor,
                                   const sensor_msgs::Image::ConstPtr imageDepth,
                                   const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                   const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
        void driverNodeCallback(sensor_msgs::Image::ConstPtr imageColor, sensor_msgs::Image::ConstPtr depth,
                                sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);

        void processCVMessages();
        void processPCLMessages();
        void prepareMake3D();
        void processPCL1();
        void processPCL2();

        const uint32_t queueSize;
        const bool useExact, useCompressed;
        bool preparedMake3D;
        std::string imageRGBTopic, imageRGBCameraInfoTopic, imageDepthTopic,
                imageDepthCameraInfoTopic, imageRGBDriverTopic, imageRGBCameraInfoDriverTopic,
                imageDepthDriverTopic, imageDepthCameraInfoDriverTopic, cvImageDepthTopic, cvImageRGBTopic,
                pclPointCloudTopic;

        ros::NodeHandle nh;
        image_transport::Publisher  pubImageColor, pubImageDepth, pubCvImageColor, pubCvImageDepth;
        ros::Publisher pubCameraInfoColor, pubCameraInfoDepth;
        boost::scoped_ptr<pcl_ros::Publisher< pcl::PointXYZRGB > > pubPointCloud;

        image_transport::ImageTransport it;
        boost::scoped_ptr<image_transport::SubscriberFilter> subImageColor, subImageDepth;
        boost::scoped_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > subCameraInfoColor, subCameraInfoDepth;
        boost::scoped_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > syncApproximate;

        sensor_msgs::Image::ConstPtr imageRGB, imageDepth;
        sensor_msgs::CameraInfo::ConstPtr imageRGBCameraInfo, imageDepthCameraInfo, imageDepthScaledCameraInfo;
        boost::shared_ptr<sensor_msgs::Image> imageScaledDepth;
        boost::shared_ptr<sensor_msgs::CameraInfo> imageScaledDepthCameraInfo;

        cv_bridge::CvImageConstPtr cvImageColor, cvImageDepth;
        cv_bridge::CvImagePtr cvImageScaledDepth;
        //Eigen::Matrix<float, 1280, 1> colmap;
        //Eigen::Matrix<float, 1024, 1> rowmap;

        const static int w = 1280;
        const static int h = 1024;
        const static int len_1d = h * w;
        boost::scoped_ptr<cv::Mat> m_x_map;
        boost::scoped_ptr<cv::Mat> m_x_mapping;
        boost::scoped_ptr<cv::Mat> m_y_map;
        boost::scoped_ptr<cv::Mat> m_y_mapping;
        boost::scoped_ptr<cv::Mat> big_mat;
        boost::scoped_ptr<cv::Mat> image_color;
        boost::scoped_ptr<cv::Mat> m_depth;
        boost::scoped_ptr<cv::Mat> m_filler;
        boost::scoped_ptr<cv::Mat> m_padding;
        boost::scoped_ptr<cv::Mat> m_channels_as_one_float;
        boost::scoped_ptr<cv::Mat> m_pt_cloud;
        boost::scoped_ptr<cv::Mat> m_concat_result;

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pclPointCloud;
    };

    PLUGINLIB_DECLARE_CLASS(pclwpp_adapters, XtionAdapterNodelet, pclpp_adapters::XtionAdapterNodelet, nodelet::Nodelet)
}

#endif //PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
