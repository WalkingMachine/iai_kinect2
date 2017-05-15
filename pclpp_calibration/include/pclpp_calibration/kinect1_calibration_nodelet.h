#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#ifndef PCLPP_KINECT1_CALIBRATION_NODELET_H
#define PCLPP_KINECT1_CALIBRATION_NODELET_H

namespace pclpp_calibration {

    class Kinect1CalibrationNodelet : public nodelet::Nodelet {
    public:

        Kinect1CalibrationNodelet();
        ~Kinect1CalibrationNodelet();

        int main(int argc, char **argv);

    private:

        void onInit();
    };

}

#endif //PCLPP_KINECT1_CALIBRATION_NODELET_H
