#include "../include/pclpp_calibration/xtion_calibration_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xtion_calibration");
    std::string path, topicColor;

    pclpp_calibration::XtionCalibrationNodelet xtionCalibration;
    return xtionCalibration.main(argc, argv);
}
