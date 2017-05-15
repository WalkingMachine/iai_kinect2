#include "../include/pclpp_calibration/kinect2_calibration_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect2_calibration");
    pclpp_calibration::Kinect2CalibrationNodelet kinect2Calibration;
    return kinect2Calibration.main(argc, argv);
}
