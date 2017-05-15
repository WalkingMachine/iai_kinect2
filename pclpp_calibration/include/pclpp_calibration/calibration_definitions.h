//
// Created by localhost on 15/05/17.
//

#ifndef PCLPP_CALIBRATION_DEFINITIONS_H
#define PCLPP_CALIBRATION_DEFINITIONS_H

#define DEFAULT_NS          "kinect2"

#define TF_LINK             "_link"
#define TF_RGB_OPT_FRAME    "_rgb_optical_frame"
#define TF_IR_OPT_FRAME     "_ir_optical_frame"

#define TOPIC_HD            "/hd"
#define TOPIC_QHD           "/qhd"
#define TOPIC_SD            "/sd"

#define TOPIC_IMAGE_RECT    "_rect"
#define TOPIC_IMAGE_COLOR   "/image_color"
#define TOPIC_IMAGE_MONO    "/image_mono"
#define TOPIC_IMAGE_DEPTH   "/image_depth"
#define TOPIC_IMAGE_IR      "/image_ir"

#define TOPIC_COMPRESSED    "/compressed"
#define TOPIC_INFO          "/camera_info"

#define CALIB_COLOR         "calib_color.yaml"
#define CALIB_IR            "calib_ir.yaml"
#define CALIB_POSE          "calib_pose.yaml"
#define CALIB_DEPTH         "calib_depth.yaml"

#define CALIB_CAMERA_MATRIX "cameraMatrix"
#define CALIB_DISTORTION    "distortionCoefficients"
#define CALIB_ROTATION      "rotation"
#define CALIB_PROJECTION    "projection"
#define CALIB_TRANSLATION   "translation"
#define CALIB_ESSENTIAL     "essential"
#define CALIB_FUNDAMENTAL   "fundamental"
#define CALIB_DEPTH_SHIFT   "depthShift"

#endif //PCLPP_CALIBRATION_DEFINITIONS_H
