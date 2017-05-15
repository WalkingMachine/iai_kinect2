#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#ifndef PCLPP_KINECT2_CALIBRATION_NODELET_H
#define PCLPP_KINECT2_CALIBRATION_NODELET_H

namespace pclpp_calibration {

    class Kinect2CalibrationNodelet : public nodelet::Nodelet {
    public:

        Kinect2CalibrationNodelet();
        ~Kinect2CalibrationNodelet();

        int main(int argc, char **argv);

    private:

        void onInit();
    };

}

#endif //PCLPP_KINECT2_CALIBRATION_NODELET_H
