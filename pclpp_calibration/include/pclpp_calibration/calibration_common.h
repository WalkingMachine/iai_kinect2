
#ifndef PCLPP_CALIBRATION_COMMON_H
#define PCLPP_CALIBRATION_COMMON_H

#include "xtion_calibration_nodelet.h"
#include <opencv-3.2.0-dev/opencv2/core/types.hpp>
#include <fstream>

namespace pclpp_calibration {

    class CameraCalibration {
    public:

        CameraCalibration(const std::string &path, const Source mode, const bool circleBoard,
                          const cv::Size &boardDims, const float boardSize, const bool rational);
        ~CameraCalibration();
        bool restore();
        void calibrate();


    private:

        bool readFiles(const std::vector<std::string> &files, const std::string &ext,
                       std::vector<std::vector<cv::Point2f> > &points) const;
        bool checkSyncPointsOrder();
        void calibrateIntrinsics(const cv::Size &size, const std::vector<std::vector<cv::Point3f> > &pointsBoard,
                                 const std::vector<std::vector<cv::Point2f> > &points, cv::Mat &cameraMatrix,
                                 cv::Mat &distortion, cv::Mat &rotation, cv::Mat &projection,
                                 std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs);
        void calibrateExtrinsics();
        void storeCalibration();
        bool loadCalibration();


        const bool circleBoard;
        const cv::Size boardDims;
        const float boardSize;
        const int flags;

        const Source mode;
        const std::string path;

        std::vector<cv::Point3f> board;

        std::vector<std::vector<cv::Point3f> > pointsBoard;
        std::vector<std::vector<cv::Point2f> > pointsColor;
        std::vector<std::vector<cv::Point2f> > pointsIr;

        cv::Size sizeColor;
        cv::Size sizeIr;

        cv::Mat cameraMatrixColor, distortionColor, rotationColor, translationColor, projectionColor;
        cv::Mat cameraMatrixIr, distortionIr, rotationIr, translationIr, projectionIr;
        cv::Mat rotation, translation, essential, fundamental, disparity;

        std::vector<cv::Mat> rvecsColor, tvecsColor;
        std::vector<cv::Mat> rvecsIr, tvecsIr;
    };

    class DepthCalibration {
    public:
        DepthCalibration(const std::string &path, const cv::Size &boardDims, const float boardSize);
        ~DepthCalibration();
        bool restore();
        void calibrate();

    private:

        void compareDists(const std::vector<double> &imageDists, const std::vector<double> &depthDists) const;
        void computePointDists(const cv::Mat &normal, const double distance, const cv::Mat &region,
                               const cv::Rect &roi, std::vector<double> &depthDists, std::vector<double> &imageDists);
        double computeDistance(const cv::Point &pointImage, const cv::Mat &normal, const double distance) const;
        void getPlane(const size_t index, cv::Mat &normal, double &distance) const;
        void computeROI(const cv::Mat &depth, const std::vector<cv::Point2f> &points, cv::Mat &region,
                        cv::Rect &roi) const;
        bool readFiles(const std::vector<std::string> &files);
        bool loadCalibration();
        void storeCalibration(const double depthShift) const;

        const std::string path;

        std::vector<cv::Point3f> board;
        std::vector<std::vector<cv::Point2f> > points;
        std::vector<std::string> images;

        cv::Size size;

        cv::Mat cameraMatrix, distortion, rotation, translation;
        cv::Mat mapX, mapY;

        double fx, fy, cx, cy;

        std::ofstream plot;
    };

}

#endif //PCLPP_CALIBRATION_COMMON_H
