#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <opencv-3.2.0-dev/opencv2/core/types.hpp>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <mutex>
#include <opencv-3.2.0-dev/opencv2/core/mat.hpp>
#include <image_transport/subscriber_filter.h>
#include <opencv-3.2.0-dev/opencv2/imgproc.hpp>

#ifndef PCLPP_XTION_CALIBRATION_NODELET_H
#define PCLPP_XTION_CALIBRATION_NODELET_H

namespace pclpp_calibration {

    enum Mode
    {
        RECORD,
        CALIBRATE
    };

    enum Source
    {
        COLOR,
        IR,
        SYNC
    };

    class XtionCalibrationNodelet : public nodelet::Nodelet {
    public:

        XtionCalibrationNodelet();
        ~XtionCalibrationNodelet();

        int main(int argc, char **argv);

        void run();

    private:

        void onInit();

        void startRecord();
        void stopRecord();
        void convertIr(const cv::Mat &ir, cv::Mat &grey);
        void findMinMax(const cv::Mat &ir, const std::vector<cv::Point2f> &pointsIr);
        void findMinMax(const cv::Mat &ir);
        void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageIr,
                      const sensor_msgs::Image::ConstPtr imageDepth);
        void display();
        void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;
        void store(const cv::Mat &color, const cv::Mat &ir, const cv::Mat &irGrey, const cv::Mat &depth,
                   const std::vector<cv::Point2f> &pointsColor, std::vector<cv::Point2f> &pointsIr);


        bool circleBoard;
        int circleFlags;

        cv::Size boardDims;
        float boardSize;
        Source mode;

        std::string path;
        std::string topicColor, topicIr, topicDepth;
        std::mutex lock;

        bool update;
        bool foundColor, foundIr;
        cv::Mat color, ir, irGrey, depth;

        size_t frame;
        std::vector<int> params;

        std::vector<cv::Point3f> board;
        std::vector<cv::Point2f> pointsColor, pointsIr;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ColorIrDepthSyncPolicy;
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner;
        image_transport::ImageTransport it;
        image_transport::SubscriberFilter *subImageColor, *subImageIr, *subImageDepth;
        message_filters::Synchronizer<ColorIrDepthSyncPolicy> *sync;

        int minIr, maxIr;
        cv::Ptr<cv::CLAHE> clahe;

    };

}

#endif //PCLPP_XTION_CALIBRATION_NODELET_H
