
#include "../include/pclpp_calibration/xtion_calibration_nodelet.h"

int main(int argc, char **argv)
{
    int i_arg = 0;
    std::string calibration_name = "";
    while (i_arg < argc) {
        if (boost::starts_with(argv[i_arg], "__name:")) {
            size_t header_len = strlen("__name:=");
            calibration_name = std::string(argv[i_arg]);
            calibration_name = calibration_name.substr(header_len, calibration_name.length() - header_len);
        }
        i_arg++;
    }

    ros::init(argc, argv, calibration_name.c_str());
    boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle(calibration_name));
    image_transport::ImageTransport it(*nh);

    pclpp_calibration::XtionCalibrationNodelet xtionCalibration(calibration_name, nh, it);
    return xtionCalibration.main(argc, argv);
}
