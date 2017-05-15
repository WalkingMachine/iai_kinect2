#include <opencv-3.2.0-dev/opencv/cv.hpp>
#include <dirent.h>
#include "../include/pclpp_calibration/calibration_common.h"
#include "../../iai_kinect2/kinect2_calibration/include/kinect2_calibration/kinect2_calibration_definitions.h"
#include "../include/pclpp_calibration/calibration_definitions.h"

namespace pclpp_calibration {

    CameraCalibration::CameraCalibration(const std::string &path, const Source mode, const bool circleBoard,
                      const cv::Size &boardDims, const float boardSize, const bool rational)
        : circleBoard(circleBoard), boardDims(boardDims), boardSize(boardSize),
          flags(rational ? cv::CALIB_RATIONAL_MODEL : 0), mode(mode), path(path), sizeColor(1920, 1080),
          sizeIr(512, 424) {
        board.resize((unsigned long)boardDims.width * boardDims.height);
        for(size_t r = 0, i = 0; r < (size_t)boardDims.height; ++r)
        {
            for(size_t c = 0; c < (size_t)boardDims.width; ++c, ++i)
            {
                board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
            }
        }
    }

    CameraCalibration::~CameraCalibration() {}

    bool CameraCalibration::restore() {

        std::vector<std::string> filesSync;
        std::vector<std::string> filesColor;
        std::vector<std::string> filesIr;

        DIR *dp;
        struct dirent *dirp;
        size_t posColor, posIr, posSync;

        if((dp  = opendir(path.c_str())) ==  NULL)
        {
            ROS_ERROR("Error opening: %s", path.c_str());
            return false;
        }

        while((dirp = readdir(dp)) != NULL)
        {
            std::string filename = dirp->d_name;

            if(dirp->d_type != DT_REG)
            {
                continue;
            }

            posSync = filename.rfind(CALIB_SYNC);
            posColor = filename.rfind(CALIB_FILE_COLOR);

            if(posSync != std::string::npos)
            {
                if(posColor != std::string::npos)
                {
                    std::string frameName = filename.substr(0, posColor);
                    filesSync.push_back(frameName);
                    filesColor.push_back(frameName);
                    filesIr.push_back(frameName);
                }
                continue;
            }

            if(posColor != std::string::npos)
            {
                std::string frameName = filename.substr(0, posColor);
                filesColor.push_back(frameName);
                continue;
            }

            posIr = filename.rfind(CALIB_FILE_IR_GREY);
            if(posIr != std::string::npos)
            {
                std::string frameName = filename.substr(0, posIr);
                filesIr.push_back(frameName);
                continue;
            }
        }
        closedir(dp);

        std::sort(filesColor.begin(), filesColor.end());
        std::sort(filesIr.begin(), filesIr.end());
        std::sort(filesSync.begin(), filesSync.end());

        bool ret = true;
        switch(mode)
        {
            case COLOR:
                if(filesColor.empty())
                {
                    ROS_ERROR("no files found!");
                    return false;
                }
                pointsColor.resize(filesColor.size());
                pointsBoard.resize(filesColor.size(), board);
                ret = ret && readFiles(filesColor, CALIB_POINTS_COLOR, pointsColor);
                break;
            case IR:
                if(filesIr.empty())
                {
                    ROS_ERROR("no files found!");
                    return false;
                }
                pointsIr.resize(filesIr.size());
                pointsBoard.resize(filesIr.size(), board);
                ret = ret && readFiles(filesIr, CALIB_POINTS_IR, pointsIr);
                break;
            case SYNC:
                if(filesColor.empty() || filesIr.empty())
                {
                    ROS_ERROR("no files found!");
                    return false;
                }
                pointsColor.resize(filesColor.size());
                pointsIr.resize(filesSync.size());
                pointsColor.resize(filesSync.size());
                pointsBoard.resize(filesSync.size(), board);
                ret = ret && readFiles(filesSync, CALIB_POINTS_COLOR, pointsColor);
                ret = ret && readFiles(filesSync, CALIB_POINTS_IR, pointsIr);
                ret = ret && checkSyncPointsOrder();
                ret = ret && loadCalibration();
                break;
        }
        return ret;

    }

    void CameraCalibration::calibrate() {
        switch(mode)
        {
            case COLOR:
                calibrateIntrinsics(sizeColor, pointsBoard, pointsColor, cameraMatrixColor, distortionColor, rotationColor, projectionColor, rvecsColor, tvecsColor);
                break;
            case IR:
                calibrateIntrinsics(sizeIr, pointsBoard, pointsIr, cameraMatrixIr, distortionIr, rotationIr, projectionIr, rvecsIr, tvecsIr);
                break;
            case SYNC:
                calibrateExtrinsics();
                break;
        }
        storeCalibration();
    }

    bool CameraCalibration::readFiles(const std::vector<std::string> &files, const std::string &ext,
                   std::vector<std::vector<cv::Point2f> > &points) const {
        bool ret = true;
#pragma omp parallel for
        for(size_t i = 0; i < files.size(); ++i)
        {
            std::string pointsname = path + files[i] + ext;

#pragma omp critical
            ROS_INFO("restoring file: %s%s", files[i].c_str(), ext.c_str());

            cv::FileStorage file(pointsname, cv::FileStorage::READ);
            if(!file.isOpened())
            {
#pragma omp critical
                {
                    ret = false;
                    ROS_ERROR("couldn't open file: %s%s", files[i].c_str(), ext.c_str());
                }
            }
            else
            {
                file["points"] >> points[i];
            }
        }
        return ret;
    }

    bool CameraCalibration::checkSyncPointsOrder() {
        if(pointsColor.size() != pointsIr.size())
        {
            ROS_ERROR("number of detected color and ir patterns does not match!");
            return false;
        }

        for(size_t i = 0; i < pointsColor.size(); ++i)
        {
            const std::vector<cv::Point2f> &pColor = pointsColor[i];
            const std::vector<cv::Point2f> &pIr = pointsIr[i];

            if(pColor.front().y > pColor.back().y || pColor.front().x > pColor.back().x)
            {
                std::reverse(pointsColor[i].begin(), pointsColor[i].end());
            }

            if(pIr.front().y > pIr.back().y || pIr.front().x > pIr.back().x)
            {
                std::reverse(pointsIr[i].begin(), pointsIr[i].end());
            }
        }
        return true;
    }

    void CameraCalibration::calibrateIntrinsics(const cv::Size &size,
                                                const std::vector<std::vector<cv::Point3f> > &pointsBoard,
                                                const std::vector<std::vector<cv::Point2f> > &points,
                                                cv::Mat &cameraMatrix,
                                                cv::Mat &distortion, cv::Mat &rotation, cv::Mat &projection,
                                                std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs) {
        if(points.empty())
        {
            ROS_ERROR("no data for calibration provided!");
            return;
        }
        const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
        double error;

        ROS_INFO("calibrating intrinsics...");
        error = cv::calibrateCamera(pointsBoard, points, size, cameraMatrix, distortion, rvecs, tvecs, flags, termCriteria);
        ROS_INFO("re-projection error: %f\n", error);

        std::stringstream buffer;
        buffer << cameraMatrix;
        ROS_INFO("Camera Matrix:\n%s", buffer.str().c_str());
        buffer << distortion;
        ROS_INFO("Distortion Coeeficients:\n%s\n", buffer.str().c_str());
        rotation = cv::Mat::eye(3, 3, CV_64F);
        projection = cv::Mat::eye(4, 4, CV_64F);
        cameraMatrix.copyTo(projection(cv::Rect(0, 0, 3, 3)));
    }

    void CameraCalibration::calibrateExtrinsics() {
        if(pointsColor.size() != pointsIr.size())
        {
            ROS_ERROR("number of detected color and ir patterns does not match!");
            return;
        }
        if(pointsColor.empty() || pointsIr.empty())
        {
            ROS_ERROR("no data for calibration provided!");
            return;
        }
        const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
        double error;

        std::stringstream buffer;
        buffer << cameraMatrixColor;
        ROS_INFO("Camera Matrix Color:\n%s", buffer.str().c_str());
        buffer << distortionColor;
        ROS_INFO("Distortion Coeeficients Color:\n%s\n", buffer.str().c_str());
        buffer << cameraMatrixIr;
        ROS_INFO("Camera Matrix Ir:\n%s", buffer.str().c_str());
        buffer << distortionIr;
        ROS_INFO("Distortion Coeeficients Ir:\n%s\n", buffer.str().c_str());

        ROS_INFO("calibrating Color and Ir extrinsics...");
#if CV_MAJOR_VERSION == 2
        error = cv::stereoCalibrate(pointsBoard, pointsIr, pointsColor, cameraMatrixIr, distortionIr, cameraMatrixColor, distortionColor, sizeColor,
                                rotation, translation, essential, fundamental, termCriteria, cv::CALIB_FIX_INTRINSIC);
#elif CV_MAJOR_VERSION == 3
        error = cv::stereoCalibrate(pointsBoard, pointsIr, pointsColor, cameraMatrixIr, distortionIr, cameraMatrixColor, distortionColor, sizeColor,
                                    rotation, translation, essential, fundamental, cv::CALIB_FIX_INTRINSIC, termCriteria);
#endif
        ROS_INFO("re-projection error: %f\n", error);
        buffer << rotation;
        ROS_INFO("Rotation:\n%s", buffer.str().c_str());
        buffer << translation;
        ROS_INFO("Translation:\n%s", buffer.str().c_str());
        buffer << essential;
        ROS_INFO("Essential:\n%s", buffer.str().c_str());
        buffer << fundamental;
        ROS_INFO("Fundamental:\n%s\n", buffer.str().c_str());
    }

    void CameraCalibration::storeCalibration() {
        cv::FileStorage fs;

        switch(mode)
        {
            case SYNC:
                fs.open(path + CALIB_POSE, cv::FileStorage::WRITE);
                break;
            case COLOR:
                fs.open(path + CALIB_COLOR, cv::FileStorage::WRITE);
                break;
            case IR:
                fs.open(path + CALIB_IR, cv::FileStorage::WRITE);
                break;
        }

        if(!fs.isOpened())
        {
            ROS_ERROR("couldn't store calibration data!");
            return;
        }

        switch(mode)
        {
            case SYNC:
                fs << CALIB_ROTATION << rotation;
                fs << CALIB_TRANSLATION << translation;
                fs << CALIB_ESSENTIAL << essential;
                fs << CALIB_FUNDAMENTAL << fundamental;
                break;
            case COLOR:
                fs << CALIB_CAMERA_MATRIX << cameraMatrixColor;
                fs << CALIB_DISTORTION << distortionColor;
                fs << CALIB_ROTATION << rotationColor;
                fs << CALIB_PROJECTION << projectionColor;
                break;
            case IR:
                fs << CALIB_CAMERA_MATRIX << cameraMatrixIr;
                fs << CALIB_DISTORTION << distortionIr;
                fs << CALIB_ROTATION << rotationIr;
                fs << CALIB_PROJECTION << projectionIr;
                break;
        }
        fs.release();
    }

    bool CameraCalibration::loadCalibration() {
        cv::FileStorage fs;

        if(fs.open(path + CALIB_COLOR, cv::FileStorage::READ))
        {
            fs[CALIB_CAMERA_MATRIX] >> cameraMatrixColor;
            fs[CALIB_DISTORTION] >> distortionColor;
            fs[CALIB_ROTATION] >> rotationColor;
            fs[CALIB_PROJECTION] >> projectionColor;
            fs.release();
        }
        else
        {
            ROS_ERROR("couldn't load color calibration data!");
            return false;
        }

        if(fs.open(path + CALIB_IR, cv::FileStorage::READ))
        {
            fs[CALIB_CAMERA_MATRIX] >> cameraMatrixIr;
            fs[CALIB_DISTORTION] >> distortionIr;
            fs[CALIB_ROTATION] >> rotationIr;
            fs[CALIB_PROJECTION] >> projectionIr;
            fs.release();
        }
        else
        {
            ROS_ERROR("couldn't load ir calibration data!");
            return false;
        }

        return true;
    }

    DepthCalibration::DepthCalibration(const std::string &path, const cv::Size &boardDims, const float boardSize)
        : path(path), size(512, 424) {
        board.resize((unsigned long)boardDims.width * boardDims.height);
        for(size_t r = 0, i = 0; r < (size_t)boardDims.height; ++r)
        {
            for(size_t c = 0; c < (size_t)boardDims.width; ++c, ++i)
            {
                board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
            }
        }
    }

    DepthCalibration::~DepthCalibration() {}

    bool DepthCalibration::restore() {
        std::vector<std::string> files;

        DIR *dp;
        struct dirent *dirp;
        size_t pos;

        if((dp  = opendir(path.c_str())) ==  NULL)
        {
            ROS_ERROR("Error opening: %s", path.c_str());
            return false;
        }

        while((dirp = readdir(dp)) != NULL)
        {
            std::string filename = dirp->d_name;

            if(dirp->d_type != DT_REG)
            {
                continue;
            }

            /*pos = filename.rfind(CALIB_SYNC);
            if(pos != std::string::npos)
            {
              continue;
            }*/

            pos = filename.rfind(CALIB_FILE_IR_GREY);
            if(pos != std::string::npos)
            {
                std::string frameName = filename.substr(0, pos);
                files.push_back(frameName);
                continue;
            }
        }
        closedir(dp);

        std::sort(files.begin(), files.end());

        if(files.empty())
        {
            ROS_ERROR("no files found!");
            return false;
        }

        bool ret = readFiles(files);
        ret = ret && loadCalibration();

        if(ret)
        {
            cv::initUndistortRectifyMap(cameraMatrix, distortion, cv::Mat(), cameraMatrix, size, CV_32FC1, mapX, mapY);
            fx = cameraMatrix.at<double>(0, 0);
            fy = cameraMatrix.at<double>(1, 1);
            cx = cameraMatrix.at<double>(0, 2);
            cy = cameraMatrix.at<double>(1, 2);
        }
        return ret;
    }

    void DepthCalibration::calibrate() {
        plot.open(path + "plot.dat", std::ios_base::trunc);
        if(!plot.is_open())
        {
            ROS_ERROR("couldn't open 'plot.dat'!");
            return;
        }
        if(images.empty())
        {
            ROS_ERROR("no images found!");
            return;
        }

        plot << "# Columns:" << std::endl
             << "# 1: X" << std::endl
             << "# 2: Y" << std::endl
             << "# 3: computed depth" << std::endl
             << "# 4: measured depth" << std::endl
             << "# 5: difference between computed and measured depth" << std::endl;

        std::vector<double> depthDists, imageDists;
        for(size_t i = 0; i < images.size(); ++i)
        {
            ROS_INFO("frame: %s", images[i].c_str());
            plot << "# frame: " << images[i] << std::endl;

            cv::Mat depth, planeNormal, region;
            double planeDistance;
            cv::Rect roi;

            depth = cv::imread(images[i], cv::IMREAD_ANYDEPTH);
            if(depth.empty())
            {
                ROS_ERROR("couldn't load image '%s'!", images[i].c_str());
                return;
            }

            cv::remap(depth, depth, mapX, mapY, cv::INTER_NEAREST);
            computeROI(depth, points[i], region, roi);

            getPlane(i, planeNormal, planeDistance);

            computePointDists(planeNormal, planeDistance, region, roi, depthDists, imageDists);
        }
        compareDists(imageDists, depthDists);
    }

    void DepthCalibration::compareDists(const std::vector<double> &imageDists,
                                        const std::vector<double> &depthDists) const {
        if(imageDists.size() != depthDists.size())
        {
            ROS_ERROR("number of real and computed distance samples does not match!");
            return;
        }
        if(imageDists.empty() || depthDists.empty())
        {
            ROS_ERROR("no distance sample data!");
            return;
        }

        double avg = 0, sqavg = 0, var = 0, stddev = 0;
        std::vector<double> diffs(imageDists.size());

        for(size_t i = 0; i < imageDists.size(); ++i)
        {
            diffs[i] = imageDists[i] - depthDists[i];
            avg += diffs[i];
            sqavg += diffs[i] * diffs[i];
        }
        sqavg = sqrt(sqavg / imageDists.size());
        avg /= imageDists.size();

        for(size_t i = 0; i < imageDists.size(); ++i)
        {
            const double diff = diffs[i] - avg;
            var += diff * diff;
        }
        var =  var / (imageDists.size());
        stddev = sqrt(var);

        std::sort(diffs.begin(), diffs.end());
        ROS_INFO("stats on difference: \n\t\tavg: %f\n\t\tvar: %f\n\t\tstddev: %f\n\t\trms: %f\n\t\tmedian: %f\n",
                 avg, var, stddev, sqavg, diffs[diffs.size() / 2]);

        storeCalibration(avg * 1000.0);
    }

    void DepthCalibration::computePointDists(const cv::Mat &normal, const double distance, const cv::Mat &region,
                           const cv::Rect &roi, std::vector<double> &depthDists, std::vector<double> &imageDists) {
        for(int r = 0; r < region.rows; ++r)
        {
            const uint16_t *itD = region.ptr<uint16_t>(r);
            cv::Point p(roi.x, roi.y + r);

            for(int c = 0; c < region.cols; ++c, ++itD, ++p.x)
            {
                const double dDist = *itD / 1000.0;

                if(dDist < 0.1)
                {
                    continue;
                }

                const double iDist = computeDistance(p, normal, distance);
                const double diff = iDist - dDist;

                if(std::abs(diff) > 0.08)
                {
                    continue;
                }
                depthDists.push_back(dDist);
                imageDists.push_back(iDist);
                plot << p.x << ' ' << p.y << ' ' << iDist << ' ' << dDist << ' ' << diff << std::endl;
            }
        }
    }

    double DepthCalibration::computeDistance(const cv::Point &pointImage, const cv::Mat &normal,
                                             const double distance) const {
        cv::Mat point = cv::Mat(3, 1, CV_64F);

        point.at<double>(0) = (pointImage.x - cx) / fx;
        point.at<double>(1) = (pointImage.y - cy) / fy;
        point.at<double>(2) = 1;

        double t = distance / normal.dot(point);
        point = point * t;

        return point.at<double>(2);
    }

    void DepthCalibration::getPlane(const size_t index, cv::Mat &normal, double &distance) const {
        cv::Mat rvec, rotation, translation;
        //cv::solvePnP(board, points[index], cameraMatrix, distortion, rvec, translation, false, cv::EPNP);
#if CV_MAJOR_VERSION == 2
        cv::solvePnPRansac(board, points[index], cameraMatrix, distortion, rvec, translation, false, 300, 0.05, board.size(), cv::noArray(), cv::ITERATIVE);
#elif CV_MAJOR_VERSION == 3
        cv::solvePnPRansac(board, points[index], cameraMatrix, distortion, rvec, translation, false, 300, 0.05, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
#endif
        cv::Rodrigues(rvec, rotation);

        normal = cv::Mat(3, 1, CV_64F);
        normal.at<double>(0) = 0;
        normal.at<double>(1) = 0;
        normal.at<double>(2) = 1;
        normal = rotation * normal;
        distance = normal.dot(translation);
    }

    void DepthCalibration::computeROI(const cv::Mat &depth, const std::vector<cv::Point2f> &points, cv::Mat &region,
                    cv::Rect &roi) const {
        std::vector<cv::Point2f>  norm;
        std::vector<cv::Point> undist, hull;

        cv::undistortPoints(points, norm, cameraMatrix, distortion);
        undist.reserve(norm.size());

        for(size_t i = 0; i < norm.size(); ++i)
        {
            cv::Point p;
            p.x = (int)round(norm[i].x * fx + cx);
            p.y = (int)round(norm[i].y * fy + cy);
            if(p.x >= 0 && p.x < depth.cols && p.y >= 0 && p.y < depth.rows)
            {
                undist.push_back(p);
            }
        }

        roi = cv::boundingRect(undist);

        cv::Mat mask = cv::Mat::zeros(depth.rows, depth.cols, CV_8U);

        cv::convexHull(undist, hull);
        cv::fillConvexPoly(mask, hull, CV_RGB(255, 255, 255));

        cv::Mat tmp;
        depth.copyTo(tmp, mask);
        tmp(roi).copyTo(region);
    }

    bool DepthCalibration::readFiles(const std::vector<std::string> &files) {
        points.resize(files.size());
        images.resize(files.size());
        bool ret = true;

#pragma omp parallel for
        for(size_t i = 0; i < files.size(); ++i)
        {
            std::string pointsname = path + files[i] + CALIB_POINTS_IR;

#pragma omp critical
            ROS_INFO("restoring file: %s", files[i].c_str());

            cv::FileStorage file(pointsname, cv::FileStorage::READ);
            if(!file.isOpened())
            {
#pragma omp critical
                {
                    ROS_ERROR("couldn't read '%s'!", pointsname.c_str());
                    ret = false;
                }
            }
            else
            {
                file["points"] >> points[i];
                file.release();
                images[i] = path + files[i] + CALIB_FILE_DEPTH;
            }
        }
        return ret;
    }

    bool DepthCalibration::loadCalibration() {
        cv::FileStorage fs;

        if(fs.open(path + CALIB_IR, cv::FileStorage::READ))
        {
            fs[CALIB_CAMERA_MATRIX] >> cameraMatrix;
            fs[CALIB_DISTORTION] >> distortion;
            fs.release();
        }
        else
        {
            ROS_ERROR("couldn't read calibration '%s%s'!", path.c_str(), CALIB_IR);
            return false;
        }

        return true;
    }

    void DepthCalibration::storeCalibration(const double depthShift) const {
        cv::FileStorage fs;

        if(fs.open(path + CALIB_DEPTH, cv::FileStorage::WRITE))
        {
            fs << CALIB_DEPTH_SHIFT << depthShift;
            fs.release();
        }
        else
        {
            ROS_ERROR("couldn't store depth calibration!");
        }
    }
}