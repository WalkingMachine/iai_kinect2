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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#ifndef PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
#define PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H


namespace pclpp_adapters {

    class XtionAdapterNodelet : public nodelet::Nodelet {

    public:
        XtionAdapterNodelet(const char *adapter_name = (const char *) "unknown_adapter",
                            boost::shared_ptr<ros::NodeHandle> &n_h = *(boost::shared_ptr<ros::NodeHandle> *)NULL,
                            image_transport::ImageTransport &i_t = *(image_transport::ImageTransport *)NULL);
        ~XtionAdapterNodelet();

        int main(int argc, char **argv);

    private:

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

        void onInit();
        void driverNodeletCallback(const sensor_msgs::Image::ConstPtr imageColor,
                                   const sensor_msgs::Image::ConstPtr imageDepth,
                                   const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                   const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
        void driverNodeCallback(sensor_msgs::Image::ConstPtr imageColor,
                                sensor_msgs::Image::ConstPtr depth,
                                sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);

        void processCVMessages();
        void processPCLMessages();
        void prepareMake3D();
        void processPCL1();
        void processPCL2();
        void publishTf();

        const uint32_t queueSize;
        const bool useExact, useCompressed;
        bool preparedMake3D, pointCloudAssigned;
        const std::string adapterName, driverName;
        const std::string calibrationPath;
        const std::string adapterPubFrameId;
        const std::string imageRGBTopic, imageRGBCameraInfoTopic,
                imageDepthTopic, imageDepthCameraInfoTopic,
                imageIrTopic, imageIrCameraInfoTopic,
                imageRGBDriverTopic, imageRGBCameraInfoDriverTopic,
                imageDepthDriverTopic, imageDepthCameraInfoDriverTopic,
                imageIrDriverTopic, imageIrCameraInfoDriverTopic,
                pclPointCloudTopic;

        boost::shared_ptr<ros::NodeHandle> nh;
        const image_transport::TransportHints hints;
        image_transport::Publisher  pubImageColor, pubImageDepth, pubImageIr;
        ros::Publisher pubCameraInfoColor, pubCameraInfoDepth, pubCameraInfoIr;
        pcl_ros::Publisher<pcl::PointXYZRGB> pubPointCloud;



        image_transport::ImageTransport it;
        const boost::scoped_ptr<image_transport::SubscriberFilter> subImageColor, subImageDepth, subImageIr;
        const boost::scoped_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > subCameraInfoColor,
                subCameraInfoDepth, subCameraInfoIr;
        const boost::scoped_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > syncApproximate;

        sensor_msgs::ImagePtr imageRGB, imageDepth, imageIr;
        sensor_msgs::CameraInfoPtr imageRGBCameraInfo, imageDepthCameraInfo,
                imageDepthScaledCameraInfo, imageIrCameraInfo;
        sensor_msgs::Image imageScaledDepth;
        sensor_msgs::CameraInfo imageScaledDepthCameraInfo;

        cv_bridge::CvImageConstPtr cvImageColor, cvImageDepth;
        cv_bridge::CvImagePtr cvImageScaledDepth;
        //Eigen::Matrix<float, 1280, 1> colmap;
        //Eigen::Matrix<float, 1024, 1> rowmap;

        const static int w = 1280;
        const static int h = 1024;
        const static int len_1d = h * w;
        boost::shared_ptr<cv::Mat> m_x_map;
        boost::shared_ptr<cv::Mat> m_x_mapping;
        boost::shared_ptr<cv::Mat> m_y_map;
        boost::shared_ptr<cv::Mat> m_y_mapping;
        boost::shared_ptr<cv::Mat> big_mat;
        boost::shared_ptr<cv::Mat> m_color;
        boost::shared_ptr<cv::Mat> m_depth;
        boost::shared_ptr<cv::Mat> m_filler;
        boost::shared_ptr<cv::Mat> m_padding;
        boost::shared_ptr<cv::Mat> m_channels_as_one_float;
        boost::shared_ptr<cv::Mat> m_pt_cloud;
        boost::shared_ptr<cv::Mat> m_pointcloud_points;
        boost::shared_ptr<std::vector<cv::Mat>> concat_matrices;

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pclPointCloud;

        const bool useStaticTf, useTfTuning;
        const double staticTfX, staticTfY, staticTfZ, staticTfRoll, staticTfPitch, staticTfYaw;
        const double tfTuningX, tfTuningY, tfTuningZ, tfTuningRoll, tfTuningPitch, tfTuningYaw;

        boost::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
        boost::shared_ptr<geometry_msgs::TransformStamped> tfStamped;
        boost::shared_ptr<tf2::Quaternion> tfQuaternion;
    };


}

//PLUGINLIB_DECLARE_CLASS(pclpp_adapters, XtionAdapterNodelet, pclpp_adapters::XtionAdapterNodelet, nodelet::Nodelet)

#endif //PCL_PREPROCESSING_XTION_ADAPTER_NODELET_H
