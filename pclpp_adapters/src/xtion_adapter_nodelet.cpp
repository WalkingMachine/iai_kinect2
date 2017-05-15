#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"

/*
 *
 * http://ros-users.122217.n3.nabble.com/debugging-nodelets-td2370662.html
 *
 */

namespace pclpp_adapters {

    XtionAdapterNodelet::XtionAdapterNodelet()
    : nh("~"), it(nh), queueSize(10), useExact(true), useCompressed(false),
      preparedMake3D(false)
    {
        nh.getParam("/xtion_adapter/topic_rgb_image", imageRGBTopic);
        nh.getParam("/xtion_adapter/topic_rgb_camera_info", imageRGBCameraInfoTopic);
        nh.getParam("/xtion_adapter/topic_rgb_cv_bridge_image_mat", cvImageRGBTopic);
        nh.getParam("/xtion_adapter/topic_depth_image", imageDepthTopic);
        nh.getParam("/xtion_adapter/topic_depth_camera_info", imageDepthCameraInfoTopic);
        nh.getParam("/xtion_adapter/topic_depth_cv_bridge_image_mat", cvImageDepthTopic);
        nh.getParam("/xtion_adapter/topic_pcl_point_cloud", pclPointCloudTopic);
        nh.getParam("/xtion_adapter/topic_driver_rgb_image", imageRGBDriverTopic);
        nh.getParam("/xtion_adapter/topic_driver_rgb_camera_info", imageRGBCameraInfoDriverTopic);
        nh.getParam("/xtion_adapter/topic_driver_depth_image", imageDepthDriverTopic);
        nh.getParam("/xtion_adapter/topic_driver_depth_camera_info", imageDepthCameraInfoDriverTopic);
        nh.getParam("/xtion_adapter/topic_driver_depth_camera_info", imageDepthCameraInfoDriverTopic);
        nh.getParam("/xtion_adapter/topic_driver_depth_camera_info", imageDepthCameraInfoDriverTopic);

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor.reset(new image_transport::SubscriberFilter(it, imageRGBDriverTopic, queueSize, hints));
        subImageDepth.reset(new image_transport::SubscriberFilter(it, imageDepthDriverTopic, queueSize, hints));
        subCameraInfoColor.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, imageRGBCameraInfoDriverTopic, queueSize));
        subCameraInfoDepth.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, imageDepthCameraInfoDriverTopic, queueSize));
        subImageColor.reset(new image_transport::SubscriberFilter(it, imageRGBDriverTopic, queueSize, hints));
        syncApproximate.reset(new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth));
        syncApproximate->registerCallback(boost::bind(&XtionAdapterNodelet::driverNodeCallback, this, _1, _2, _3, _4));

        pubImageColor = it.advertise(imageRGBTopic, 10);
        pubImageDepth = it.advertise(imageDepthTopic, 10);
        //pubCvImageColor = it.advertise(cvImageRGBTopic, 10);
        //pubCvImageDepth = it.advertise(cvImageDepthTopic, 10);
        pubCameraInfoColor = nh.advertise<sensor_msgs::CameraInfo>(imageRGBCameraInfoTopic, 10);
        pubCameraInfoDepth = nh.advertise<sensor_msgs::CameraInfo>(imageDepthCameraInfoTopic, 10);
        pubPointCloud.reset(new pcl_ros::Publisher< pcl::PointXYZRGB >(nh, pclPointCloudTopic, 10));



        imageScaledDepth.reset(new sensor_msgs::Image());
        imageScaledDepthCameraInfo.reset(new sensor_msgs::CameraInfo());
    }

    XtionAdapterNodelet::~XtionAdapterNodelet() {}

    void XtionAdapterNodelet::onInit() {
        NODELET_INFO("XtionAdapterNodelet initialized");
    }

    void XtionAdapterNodelet::driverNodeletCallback(const sensor_msgs::Image::ConstPtr imageColor,
                                                    const sensor_msgs::Image::ConstPtr imageDepth,
                                                    const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                                    const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
        this->imageRGB = imageColor;
        this->imageDepth = imageDepth;
        this->imageRGBCameraInfo = cameraInfoColor;
        this->imageDepthCameraInfo = cameraInfoDepth;

        NODELET_INFO("CALLBACK %p %p %p %p", imageColor, imageDepth, cameraInfoColor, cameraInfoDepth);
    }

    void XtionAdapterNodelet::driverNodeCallback(sensor_msgs::Image::ConstPtr imageColor,
                                                 sensor_msgs::Image::ConstPtr imageDepth,
                                                 sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                                 sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
        this->imageRGB = imageColor;
        this->imageDepth = imageDepth;
        this->imageRGBCameraInfo = cameraInfoColor;
        this->imageDepthCameraInfo = cameraInfoDepth;

        imageScaledDepthCameraInfo->width = cameraInfoColor->width;
        imageScaledDepthCameraInfo->height = cameraInfoColor->height;
        float x_ratio = float(imageColor->width) / float(imageDepth->width);
        float y_ratio = float(imageColor->height) / float(imageDepth->height);
        imageScaledDepthCameraInfo->K[0] = imageDepthCameraInfo->K[0] * x_ratio;
        imageScaledDepthCameraInfo->K[2] = imageDepthCameraInfo->K[2] * x_ratio;
        imageScaledDepthCameraInfo->K[4] = imageDepthCameraInfo->K[4] * y_ratio;
        imageScaledDepthCameraInfo->K[5] = imageDepthCameraInfo->K[5] * y_ratio;
        imageScaledDepthCameraInfo->P[0] = imageDepthCameraInfo->P[0] * x_ratio;
        imageScaledDepthCameraInfo->P[2] = imageDepthCameraInfo->P[2] * x_ratio;
        imageScaledDepthCameraInfo->P[5] = imageDepthCameraInfo->P[5] * y_ratio;
        imageScaledDepthCameraInfo->P[6] = imageDepthCameraInfo->P[6] * y_ratio;

        try {
            cvImageColor = cv_bridge::toCvCopy(imageRGB, sensor_msgs::image_encodings::BGR8);
            cvImageScaledDepth = cv_bridge::toCvCopy(
                    imageDepth, sensor_msgs::image_encodings::TYPE_32FC1);
            cvImageDepth = cv_bridge::toCvCopy(
                    imageDepth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        cv::resize(cvImageDepth->image, cvImageScaledDepth->image, cv::Size(imageRGB->width,
                                                                            imageRGB->height));
        pubImageColor.publish(cvImageColor->toImageMsg());
        pubImageDepth.publish(cvImageScaledDepth->toImageMsg());
        pubCameraInfoColor.publish(imageRGBCameraInfo);
        pubCameraInfoDepth.publish(imageScaledDepthCameraInfo);

        //pubCvImageColor.publish(cvImageColor->toImageMsg());
        //pubCvImageDepth.publish(cvImageScaledDepth->toImageMsg());

        //ROS_INFO("CALLBACK %p %p %p %p", this->imageRGB, this->imageDepth,
        //             this->imageRGBCameraInfo, this->imageDepthCameraInfo);

        if (!preparedMake3D) {
            prepareMake3D();
        }
        processPCLMessages();
    }

    void XtionAdapterNodelet::processPCL1() {
        //_registration->apply(rgb, depth, &_undistorted, &_registered, remove_points, &_big_mat, _map);

        cv::Mat tmp_depth(cvImageDepth->image.rows, cvImageDepth->image.cols, CV_32FC1, cvImageDepth->image.data);
        cv::Mat tmp_colorh(cvImageColor->image.rows, cvImageColor->image.cols, CV_32FC1, cvImageColor->image.data);
        //cv::Mat tmp_color  = cv::Mat((int) rgb->height, (int) rgb->width, CV_8UC4, rgb->data);

        //if (_mirror) {
        //    cv::flip(tmp_depth, depth_mat, 1);
        //    cv::flip(tmp_color, color_mat, 1);
        //}

        // cloud = getCloud(rgb, depth, cloud);

        uint32_t w = (uint32_t) cvImageDepth->image.cols;
        uint32_t h = (uint32_t) cvImageDepth->image.rows;
        if (pclPointCloud->size() != w * h)
            pclPointCloud->resize(w * h);

        //  _registration->apply(rgb, depth, &_undistorted, &_registered, true, &_big_mat, _map);

        //cv::Mat big_depth(cvImageDepth->image.rows, cvImageDepth->image.cols, CV_32FC1, _big_mat.data);

        cv::Mat tmp_image_color(cvImageColor->image.rows, cvImageColor->image.rows, CV_8UC4);
        cv::Mat tmp_image_depth(cvImageDepth->image.rows, cvImageDepth->image.cols, CV_32FC1);
        cv::Mat tmp_colmap(512, 1, CV_32FC1);
        cv::Mat tmp_rowmap(424, 1, CV_32FC1);


        //cv::Mat _image_depth(1920, 1082, CV_32F, _big_mat.data);



        //std::cout << _image_depth << std::endl;

        //std::vector<float> array;
        //if (big_depth.isContinuous()) {
        //    array.assign((float*)big_depth.datastart + 1, (float*)big_depth.dataend);
        //} else {
        //    for (int i = 0; i < big_depth.rows; ++i) {
        //        array.insert(array.end(), (float*)big_depth.ptr<uchar>(i),
        //                     (float*)big_depth.ptr<uchar>(i)+big_depth.cols);
        //    }
        //}
    }

    void XtionAdapterNodelet::processPCL2() {
        //cv::Mat big_mat, image_color,
        //        m_depth(big_mat.reshape(0, len_1d)),
        //        m_filler(len_1d, 1, CV_32F);
        cvImageScaledDepth.get()->image.copyTo(*big_mat);
        cvImageColor.get()->image.copyTo(*image_color);

        m_depth.reset(new cv::Mat(big_mat->reshape(0, len_1d)));

        //m_x_map = cv::Mat(len_1d , 1, CV_32F);
        //m_x_mapping = cv::Mat(len_1d, 1, CV_32F);
        //m_y_map = cv::Mat(len_1d, 1, CV_32F);
        //m_y_mapping = cv::Mat(len_1d, 1, CV_32F);

        //m_filler = cv::Mat(len_1d, 1, CV_32F);
        m_filler->setTo(1.0f);

        //m_padding = cv::Mat(len_1d, 1, CV_32F);
        m_padding->setTo(0.0f);

        float *x_m_ptr = (float*)m_x_map->data;
        float *y_m_ptr = (float*)m_y_map->data;
        for (int i = 0; i <= h; i++) {
            for (int j = 0; j <= w; j++) {
                *x_m_ptr++ = (float) ((i - imageScaledDepthCameraInfo->K[2] + 0.5) / imageScaledDepthCameraInfo->K[0]);
                *y_m_ptr++ = (float) ((j - imageScaledDepthCameraInfo->K[5] + 0.5) / imageScaledDepthCameraInfo->K[4]);
            }
        }

        std::vector<cv::Mat> matrices = {*m_x_mapping, *m_y_mapping, *m_depth, *m_filler,
            *m_channels_as_one_float, *m_padding, *m_padding, *m_padding};

        //std::cout << m_pt_cloud->type() << " " << m_pt_cloud->dims << std::endl;
        //std::cout << (void *)m_pt_cloud->data << std::endl;

        *m_x_mapping = m_x_map->mul(*m_depth);
        *m_y_mapping = m_y_map->mul(*m_depth);

        //cv::Mat _m_pt_cloud();
        cv::hconcat(matrices, *m_concat_result);
        //m_pt_cloud.reset(&_m_pt_cloud);

        pcl::PointXYZRGB *pt = (pcl::PointXYZRGB *)m_concat_result->datastart;
        pclPointCloud->clear();
        pclPointCloud->points.assign((pcl::PointXYZRGB *)m_concat_result->datastart, (pcl::PointXYZRGB *)m_concat_result->dataend);
        *m_pt_cloud = cv::Mat(len_1d, 8, CV_32F, (unsigned char *)pclPointCloud->points.data());


        int count = 0;
        for (auto it = pclPointCloud->begin(); it != pclPointCloud->end(); ++it) {
            //printf("%f %f %f %d %d %d %d\n", it->x, it->y, it->z, it->b, it->g, it->r, it->a);

            if (count > 10) {
                break;
            }
            else {
                count++;
            }
        }

        cv::Mat test_depth(len_1d, 1, CV_32F);
        test_depth.setTo(2.0f);
        /*
        std::cout << m_pt_cloud->type() << " " << m_pt_cloud->dims << std::endl;
        std::cout << (void *)m_pt_cloud->data << std::endl;
        std::cout << (void *)cloud.points.begin().base() << std::endl;

        std::cout << m_x_mapping->at<float>(0, 0) << std::endl;
        std::cout << m_y_mapping->at<float>(0, 0) << std::endl;

        *m_x_mapping = m_x_map->mul(test_depth);
        *m_y_mapping = m_y_map->mul(test_depth);
        cv::hconcat(matrices, *m_pt_cloud);

        std::cout << m_pt_cloud->type() << " " << m_pt_cloud->dims << std::endl;
        std::cout << (void *)m_pt_cloud->data << std::endl;
        std::cout << (void *)cloud.points.begin().base() << std::endl;

        std::cout << "------------" << std::endl;

        std::cout << m_x_mapping->at<float>(0, 0) << std::endl;
        std::cout << m_y_mapping->at<float>(0, 0) << std::endl;

        count = 0;
        for (auto it = cloud.begin(); it != cloud.end(); ++it) {
            printf("%f %f %f %d %d %d %d\n", it->x, it->y, it->z, it->b, it->g, it->r, it->a);

            if (count > 10) {
                break;
            }
            else {
                count++;
            }
        }


        std::cout << std::endl;
        */
        //big_mat = bigmat_f["bigMat"].mat().rowRange(1, 1081);
    }

    void XtionAdapterNodelet::processPCLMessages() {
        NODELET_INFO("PROCESSING PCL Messages");

        //processPCL1();
        processPCL2();
        pclPointCloud->header.frame_id = "openni_rgb_optical_frame";
        NODELET_INFO("PUBLISH PCL Messages");
        pubPointCloud->publish(pclPointCloud);
    }

    void XtionAdapterNodelet::prepareMake3D() {

        /*
        const int w = 1280;
        const int h = 1024;
        float *pm1 = colmap.data();
        float *pm2 = rowmap.data();
        for (int i = 0; i < w; i++) {
            *pm1++ = (float) ((i - imageScaledDepthCameraInfo->K[2] + 0.5) / imageScaledDepthCameraInfo->K[0]);
        }
        for (int i = 0; i < h; i++) {
            *pm2++ = (float) ((i - imageScaledDepthCameraInfo->K[5] + 0.5) / imageScaledDepthCameraInfo->K[4]);
        }

        cv::Mat colm(w, 1, CV_32F);
        cv::Mat rowm(h, 1, CV_32F);

        for (int i = 0; i < w; i++) {
            colm.at<float>(1, i) = colmap[i];
        }

        for (int i = 0; i < h; i++) {
            rowm.at<float>(1, i) = rowmap[i];
        }*/

        pclPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        big_mat.reset(new cv::Mat(1280, 1024, CV_32F));
        image_color.reset(new cv::Mat(1280, 1024, CV_8UC3));
        m_x_map.reset(new cv::Mat(len_1d , 1, CV_32F));
        m_x_mapping.reset(new cv::Mat(len_1d, 1, CV_32F));
        m_y_map.reset(new cv::Mat(len_1d, 1, CV_32F));
        m_y_mapping.reset(new cv::Mat(len_1d, 1, CV_32F));
        m_filler.reset(new cv::Mat(len_1d, 1, CV_32F));
        m_padding.reset(new cv::Mat(len_1d, 1, CV_32F));
        m_pt_cloud.reset(new cv::Mat(len_1d, 8, CV_32F));
        m_concat_result.reset(new cv::Mat(len_1d, 8, CV_32F));
        m_channels_as_one_float.reset(new cv::Mat(len_1d, 1, CV_32F, image_color->data));

        this->preparedMake3D = true;
    };

    int XtionAdapterNodelet::main(int argc, char **argv) {
        ROS_INFO("XtionAdapterNodelet initialized as node");
        ros::spin();
        return 0;
    }
}

