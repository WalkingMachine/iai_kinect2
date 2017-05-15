#include "../include/pclpp_calibration/kinect1_calibration_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect1_calibration");
    pclpp_calibration::Kinect1CalibrationNodelet kinect1Calibration;
    return kinect1Calibration.main(argc, argv);
}