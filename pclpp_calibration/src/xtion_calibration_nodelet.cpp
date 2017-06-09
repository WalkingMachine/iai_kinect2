#include <boost/algorithm/string.hpp>
#include "../include/pclpp_calibration/xtion_calibration_nodelet.h"

namespace pclpp_calibration {

    XtionCalibrationNodelet::XtionCalibrationNodelet(std::string &calibration_name,
                                                     boost::shared_ptr<ros::NodeHandle> &n_h,
                                                     image_transport::ImageTransport &i_t)
            : calibrationName((&calibration_name != NULL) ? calibration_name : std::string("unknown calibration")),
              nh((&n_h != NULL) ? n_h : boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(calibrationName))),
              it((&i_t != NULL) ? i_t : image_transport::ImageTransport(*nh)),
              update(true), foundColor(false), foundIr(false), frame(0), spinner(0),
              minIr(0), maxIr(0x7FFF)

    {

        std::vector<int> board_dims;
        std::string source_mode_str;
        bool symmetric;

        //nh.getParam("/xtion_calibration/calibration_path", path);
        //nh.getParam("/xtion_calibration/calibration_symmetric", symmetric);
        //nh.getParam("/xtion_calibration/calibration_circle_board", circleBoard);
        //nh.getParam("/xtion_calibration/calibration_board_dims", board_dims);
        //nh.getParam("/xtion_calibration/calibration_board_size", boardSize);
        //nh.getParam("/xtion_calibration/calibration_source_mode", source_mode_str);
        //nh.getParam("/xtion_calibration/topic_rgb_image", topicColor);
        //nh.getParam("/xtion_calibration/topic_depth_image", topicDepth);
        //nh.getParam("/xtion_calibration/topic_ir_image", topicIr);

        boardDims = cv::Size(board_dims[0], board_dims[1]);
        if (boost::to_upper_copy(source_mode_str) == "COLOR") {
            mode = COLOR;
        }
        else if (boost::to_upper_copy(source_mode_str) == "IR") {
            mode = IR;
        }
        else if (boost::to_upper_copy(source_mode_str) == "SYNC") {
            mode = SYNC;
        }
    }

    XtionCalibrationNodelet::~XtionCalibrationNodelet() {}

    int XtionCalibrationNodelet::main(int argc, char **argv) {


        return 0;
    }

    void XtionCalibrationNodelet::onInit() {

    }
}