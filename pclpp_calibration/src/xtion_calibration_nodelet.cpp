#include <boost/algorithm/string.hpp>
#include <opencv-3.2.0-dev/opencv/cv.hpp>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include "../include/pclpp_calibration/xtion_calibration_nodelet.h"

namespace pclpp_calibration {

    XtionCalibrationNodelet::XtionCalibrationNodelet(std::string &calibration_name,
                                                     boost::shared_ptr<ros::NodeHandle> &n_h,
                                                     image_transport::ImageTransport &i_t)
            : calibrationName((&calibration_name != NULL) ? calibration_name : std::string("unknown calibration")),
              nh((&n_h != NULL) ? n_h : boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(calibrationName))),
              it((&i_t != NULL) ? i_t : image_transport::ImageTransport(*nh)),
              update(true), foundColor(false), foundIr(false), frame(0), spinner(0),
              minIr(0), maxIr(0x7FFF),
              calibSyncName(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_sync", std::string(""))),
              calibFileColor(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_file_color", std::string(""))),
              calibPointsColorFile(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_points_color", std::string(""))),
              calibPointsIrFile(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_points_ir", std::string(""))),
              calibFileIrGrey(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_file_ir_grey", std::string(""))),
              calibFileDepth(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_file_depth", std::string(""))),
              calibFileIr(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/definitions/calib_file_ir", std::string(""))),
              calibBoardDims(ros::param::param<std::vector<int>>(
                      "/pcl_preprocessing" + calibrationName + "/calibration/board_dims", {})),
              boardSize(ros::param::param<float>(
                      "/pcl_preprocessing" + calibrationName + "/calibration/board_size", 0.0f)),
              calibSourceMode(ros::param::param<std::string>(
                      "/pcl_preprocessing" + calibrationName + "/calibration/source_mode", std::string()))
    {
        boardDims = cv::Size(calibBoardDims[0], calibBoardDims[1]);
        if (boost::to_upper_copy(calibSourceMode) == "COLOR") {
            mode = COLOR;
        }
        else if (boost::to_upper_copy(calibSourceMode) == "IR") {
            mode = IR;
        }
        else if (boost::to_upper_copy(calibSourceMode) == "SYNC") {
            mode = SYNC;
        }
    }

    XtionCalibrationNodelet::~XtionCalibrationNodelet() {}

    int XtionCalibrationNodelet::main(int argviewer_namec, char **argv) {


        return 0;
    }

    void XtionCalibrationNodelet::onInit() {

    }

    void XtionCalibrationNodelet::run() {
        startRecord();
        display();
        stopRecord();
    }

    void XtionCalibrationNodelet::startRecord() {
        ROS_INFO("Controls:\n"
                         "\n   [ESC, q] - Exit"
                         "\n [SPACE, s] - Save current frame"
                         "\n        [l] - decrease min and max value for IR value range"
                         "\n        [h] - increase min and max value for IR value range"
                         "\n        [1] - decrease min value for IR value range"
                         "\n        [2] - increase min value for IR value range"
                         "\n        [3] - decrease max value for IR value range"
                         "\n        [4] - increase max value for IR value range");

        image_transport::TransportHints hints("compressed");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, 4, hints);
        subImageIr = new image_transport::SubscriberFilter(it, topicIr, 4, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, 4, hints);

        sync = new message_filters::Synchronizer<ColorIrDepthSyncPolicy>(
                ColorIrDepthSyncPolicy(4), *subImageColor, *subImageIr, *subImageDepth);
        sync->registerCallback(boost::bind(&XtionCalibrationNodelet::callback, this, _1, _2, _3));

        spinner.start();
    }

    void XtionCalibrationNodelet::stopRecord() {
        spinner.stop();

        delete sync;
        delete subImageColor;
        delete subImageIr;
        delete subImageDepth;
    }

    void XtionCalibrationNodelet::convertIr(const cv::Mat &ir, cv::Mat &grey) {
        const float factor = 255.0f / (maxIr - minIr);
        grey.create(ir.rows, ir.cols, CV_8U);

        #pragma omp parallel for
        for(size_t r = 0; r < (size_t)ir.rows; ++r)
        {
            const uint16_t *itI = ir.ptr<uint16_t>(r);
            uint8_t *itO = grey.ptr<uint8_t>(r);

            for(size_t c = 0; c < (size_t)ir.cols; ++c, ++itI, ++itO)
            {
                *itO = (uint8_t)std::min(std::max(*itI - minIr, 0) * factor, 255.0f);
            }
        }
        clahe->apply(grey, grey);
    }

    void XtionCalibrationNodelet::findMinMax(const cv::Mat &ir, const std::vector<cv::Point2f> &pointsIr) {
        minIr = 0xFFFF;
        maxIr = 0;
        for(size_t i = 0; i < pointsIr.size(); ++i)
        {
            const cv::Point2f &p = pointsIr[i];
            cv::Rect roi(std::max(0, (int)p.x - 2), std::max(0, (int)p.y - 2), 9, 9);
            roi.width = std::min(roi.width, ir.cols - roi.x);
            roi.height = std::min(roi.height, ir.rows - roi.y);

            findMinMax(ir(roi));
        }
    }

    void XtionCalibrationNodelet::findMinMax(const cv::Mat &ir) {
        for(size_t r = 0; r < (size_t)ir.rows; ++r)
        {
            const uint16_t *it = ir.ptr<uint16_t>((int)r);

            for(size_t c = 0; c < (size_t)ir.cols; ++c, ++it)
            {
                minIr = std::min(minIr, (int) * it);
                maxIr = std::max(maxIr, (int) * it);
            }
        }
    }

    void XtionCalibrationNodelet::callback(const sensor_msgs::Image::ConstPtr imageColor,
                                           const sensor_msgs::Image::ConstPtr imageIr,
                                           const sensor_msgs::Image::ConstPtr imageDepth) {
        std::vector<cv::Point2f> pointsColor, pointsIr;
        cv::Mat color, ir, irGrey, irScaled, depth;
        bool foundColor = false;
        bool foundIr = false;

        if(mode == COLOR || mode == SYNC)
        {
            readImage(imageColor, color);
        }
        if(mode == IR || mode == SYNC)
        {
            readImage(imageIr, ir);
            readImage(imageDepth, depth);
            cv::resize(ir, irScaled, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);

            convertIr(irScaled, irGrey);
        }

        if(circleBoard)
        {
            switch(mode)
            {
                case COLOR:
                    foundColor = cv::findCirclesGrid(color, boardDims, pointsColor, circleFlags);
                    break;
                case IR:
                    foundIr = cv::findCirclesGrid(irGrey, boardDims, pointsIr, circleFlags);
                    break;
                case SYNC:
                    foundColor = cv::findCirclesGrid(color, boardDims, pointsColor, circleFlags);
                    foundIr = cv::findCirclesGrid(irGrey, boardDims, pointsIr, circleFlags);
                    break;
            }
        }
        else
        {
            const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100, DBL_EPSILON);
            switch(mode)
            {
                case COLOR:
                    foundColor = cv::findChessboardCorners(color, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);
                    break;
                case IR:
                    foundIr = cv::findChessboardCorners(irGrey, boardDims, pointsIr, cv::CALIB_CB_ADAPTIVE_THRESH);
                    break;
                case SYNC:
                    foundColor = cv::findChessboardCorners(color, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);
                    foundIr = cv::findChessboardCorners(irGrey, boardDims, pointsIr, cv::CALIB_CB_ADAPTIVE_THRESH);
                    break;
            }
            if(foundColor)
            {
                cv::cornerSubPix(color, pointsColor, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
            }
            if(foundIr)
            {
                cv::cornerSubPix(irGrey, pointsIr, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
            }
        }

        if(foundIr)
        {
            // Update min and max ir value based on checkerboard values
            findMinMax(irScaled, pointsIr);
        }

        lock.lock();
        this->color = color;
        this->ir = ir;
        this->irGrey = irGrey;
        this->depth = depth;
        this->foundColor = foundColor;
        this->foundIr = foundIr;
        this->pointsColor = pointsColor;
        this->pointsIr = pointsIr;
        update = true;
        lock.unlock();
    }

    void XtionCalibrationNodelet::display() {
        std::vector<cv::Point2f> pointsColor, pointsIr;
        cv::Mat color, ir, irGrey, depth;
        cv::Mat colorDisp, irDisp;
        bool foundColor = false;
        bool foundIr = false;
        bool save = false;
        bool running = true;

        std::chrono::milliseconds duration(1);
        while(!update && ros::ok())
        {
            std::this_thread::sleep_for(duration);
        }

        for(; ros::ok() && running;)
        {
            if(update)
            {
                lock.lock();
                color = this->color;
                ir = this->ir;
                irGrey = this->irGrey;
                depth = this->depth;
                foundColor = this->foundColor;
                foundIr = this->foundIr;
                pointsColor = this->pointsColor;
                pointsIr = this->pointsIr;
                update = false;
                lock.unlock();

                if(mode == COLOR || mode == SYNC)
                {
                    cv::cvtColor(color, colorDisp, CV_GRAY2BGR);
                    cv::drawChessboardCorners(colorDisp, boardDims, pointsColor, foundColor);
                    //cv::resize(colorDisp, colorDisp, cv::Size(), 0.5, 0.5);
                    //cv::flip(colorDisp, colorDisp, 1);
                }
                if(mode == IR || mode == SYNC)
                {
                    cv::cvtColor(irGrey, irDisp, CV_GRAY2BGR);
                    cv::drawChessboardCorners(irDisp, boardDims, pointsIr, foundIr);
                    //cv::resize(irDisp, irDisp, cv::Size(), 0.5, 0.5);
                    //cv::flip(irDisp, irDisp, 1);
                }
            }

            switch(mode)
            {
                case COLOR:
                    cv::imshow("color", colorDisp);
                    break;
                case IR:
                    cv::imshow("ir", irDisp);
                    break;
                case SYNC:
                    cv::imshow("color", colorDisp);
                    cv::imshow("ir", irDisp);
                    break;
            }

            int key = cv::waitKey(10);
            switch(key & 0xFF)
            {
                case ' ':
                case 's':
                    save = true;
                    break;
                case 27:
                case 'q':
                    running = false;
                    break;
                case '1':
                    minIr = std::max(0, minIr - 100);
                    break;
                case '2':
                    minIr = std::min(maxIr - 1, minIr + 100);
                    break;
                case '3':
                    maxIr = std::max(minIr + 1, maxIr - 100);
                    break;
                case '4':
                    maxIr = std::min(0xFFFF, maxIr + 100);
                    break;
                case 'l':
                    minIr = std::max(0, minIr - 100);
                    maxIr = std::max(minIr + 1, maxIr - 100);
                    break;
                case 'h':
                    maxIr = std::min(0x7FFF, maxIr + 100);
                    minIr = std::min(maxIr - 1, minIr + 100);
                    break;
                default:
                    break;
            }

            if(save && ((mode == COLOR && foundColor) || (mode == IR && foundIr) || (mode == SYNC && foundColor && foundIr)))
            {
                store(color, ir, irGrey, depth, pointsColor, pointsIr);
                save = false;
            }
        }
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void XtionCalibrationNodelet::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void XtionCalibrationNodelet::store(const cv::Mat &color, const cv::Mat &ir, const cv::Mat &irGrey,
                                        const cv::Mat &depth, const std::vector<cv::Point2f> &pointsColor,
                                        std::vector<cv::Point2f> &pointsIr) {
        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(4) << frame++;
        const std::string frameNumber(oss.str());
        ROS_INFO("storing frame: %s", frameNumber.c_str());
        std::string base = path + frameNumber;

        for(size_t i = 0; i < pointsIr.size(); ++i)
        {
            pointsIr[i].x /= 2.0;
            pointsIr[i].y /= 2.0;
        }

        if(mode == SYNC)
        {
            base += calibSyncName;
        }

        if(mode == COLOR || mode == SYNC)
        {
            cv::imwrite(base + calibFileColor, color, params);

            cv::FileStorage file(base + calibPointsColorFile, cv::FileStorage::WRITE);
            file << "points" << pointsColor;
        }

        if(mode == IR || mode == SYNC)
        {
            cv::imwrite(base + calibFileIr, ir, params);
            cv::imwrite(base + calibFileIrGrey, irGrey, params);
            cv::imwrite(base + calibFileDepth, depth, params);

            cv::FileStorage file(base + calibPointsIrFile, cv::FileStorage::WRITE);
            file << "points" << pointsIr;
        }
    }
}
