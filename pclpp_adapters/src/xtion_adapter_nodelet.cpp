#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"

/*
 *
 * http://ros-users.122217.n3.nabble.com/debugging-nodelets-td2370662.html
 *
 */

namespace pclpp_adapters {

    XtionAdapterNodelet::XtionAdapterNodelet(const char *adapter_name,
                                             boost::shared_ptr<ros::NodeHandle> &n_h,
                                             image_transport::ImageTransport &i_t)
            : nh((&n_h != NULL) ? n_h : boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(adapter_name))),
              it((&i_t != NULL) ? i_t : image_transport::ImageTransport(*nh)),
              queueSize(10), useExact(true), useCompressed(false), preparedMake3D(false), pointCloudAssigned(false),
              hints(useCompressed ? "compressed" : "raw"),
              adapterName("/" + std::string(adapter_name)),
              driverName(ros::param::param(
                      "/pcl_preprocessing" + adapterName + "/driver_namespace", std::string(""))),
              adapterPubFrameId(ros::param::param<std::string>(
                      "/pcl_preprocessing" + adapterName + "/frame_id", std::string(""))),
              calibrationPath(ros::param::param<std::string>(
                      "/pcl_preprocessing" + adapterName + "/calibration_path", std::string(""))),
              imageRGBTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/rgb/image", std::string(""))),
              imageRGBCameraInfoTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/rgb/camera_info", std::string(""))),
              imageDepthTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/depth/image", std::string(""))),
              imageDepthCameraInfoTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/depth/camera_info", std::string(""))),
              imageIrTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/ir/image", std::string(""))),
              imageIrCameraInfoTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/ir/camera_info", std::string(""))),
              pclPointCloudTopic(adapterName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/pcl/point_cloud", std::string(""))),
              imageRGBDriverTopic(driverName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/rgb/image", std::string(""))),
              imageRGBCameraInfoDriverTopic(driverName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/rgb/camera_info", std::string(""))),
              imageDepthDriverTopic(driverName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/depth/image", std::string(""))),
              imageDepthCameraInfoDriverTopic(driverName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/depth/camera_info", std::string(""))),
              imageIrDriverTopic(driverName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/ir/image", std::string(""))),
              imageIrCameraInfoDriverTopic(driverName + ros::param::param<std::string>(
                      "/pcl_preprocessing/topics/ir/camera_info", std::string(""))),
              useStaticTf(ros::param::param<bool>(
                      "/pcl_preprocessing" + adapterName + "/use_static_tf", false)),
              staticTfX(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/static_tf/x", 0.0)),
              staticTfY(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/static_tf/y", 0.0)),
              staticTfZ(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/static_tf/z", 0.0)),
              staticTfRoll(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/static_tf/roll", 0.0)),
              staticTfPitch(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/static_tf/pitch", 0.0)),
              staticTfYaw(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/static_tf/yaw", 0.0)),
              useTfTuning(ros::param::param<bool>(
                      "/pcl_preprocessing" + adapterName + "/use_tf_tuning", false)),
              tfTuningX(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/tf_tuning/x", 0.0)),
              tfTuningY(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/tf_tuning/y", 0.0)),
              tfTuningZ(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/tf_tuning/z", 0.0)),
              tfTuningRoll(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/tf_tuning/roll", 0.0)),
              tfTuningPitch(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/tf_tuning/pitch", 0.0)),
              tfTuningYaw(ros::param::param<double>(
                      "/pcl_preprocessing" + adapterName + "/tf_tuning/yaw", 0.0)),
              pubCameraInfoColor(nh->advertise<sensor_msgs::CameraInfo>(imageRGBCameraInfoTopic, 10)),
              pubCameraInfoDepth(nh->advertise<sensor_msgs::CameraInfo>(imageDepthCameraInfoTopic, 10)),
              pubCameraInfoIr(nh->advertise<sensor_msgs::CameraInfo>(imageIrCameraInfoTopic, 10)),
              pubPointCloud(*nh, pclPointCloudTopic, 10),
              subImageColor(new image_transport::SubscriberFilter(it, imageRGBDriverTopic, queueSize, hints)),
              subImageDepth(new image_transport::SubscriberFilter(it, imageDepthDriverTopic, queueSize, hints)),
              subCameraInfoColor(new message_filters::Subscriber<sensor_msgs::CameraInfo>(
                      *nh, imageRGBCameraInfoDriverTopic, queueSize)),
              subCameraInfoDepth(new message_filters::Subscriber<sensor_msgs::CameraInfo>(
                      *nh, imageDepthCameraInfoDriverTopic, queueSize)),
              syncApproximate(new message_filters::Synchronizer<ApproximateSyncPolicy>(
                      ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth,
                      *subCameraInfoColor, *subCameraInfoDepth)),
              imageRGB(new sensor_msgs::Image()),
              imageDepth(new sensor_msgs::Image()),
              imageRGBCameraInfo(new sensor_msgs::CameraInfo()),
              imageDepthCameraInfo(new sensor_msgs::CameraInfo()),
              cvImageColor(new cv_bridge::CvImage()),
              cvImageDepth(new cv_bridge::CvImage()),
              cvImageScaledDepth(new cv_bridge::CvImage()),
              pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
              big_mat(new cv::Mat(1280, 1024, CV_32F)),
              concat_matrices(new std::vector< cv::Mat >()),
              m_color(new cv::Mat(1280, 1024, CV_8UC4)),
              m_depth(new cv::Mat(len_1d, 1, CV_32F)),
              m_x_map(new cv::Mat(len_1d, 1, CV_32F)),
              m_x_mapping(new cv::Mat(len_1d, 1, CV_32F)),
              m_y_map(new cv::Mat(len_1d, 1, CV_32F)),
              m_y_mapping(new cv::Mat(len_1d, 1, CV_32F)),
              m_filler(new cv::Mat(len_1d, 1, CV_32F)),
              m_padding(new cv::Mat(len_1d, 1, CV_32F)),
              m_pt_cloud(new cv::Mat(len_1d, 8, CV_32F)),
              m_pointcloud_points(new cv::Mat(len_1d, 8, CV_32F)),
              m_channels_as_one_float(new cv::Mat(len_1d, 1, CV_32F, m_color->data)),
              tfBroadcaster(new tf2_ros::TransformBroadcaster()),
              tfStamped(new geometry_msgs::TransformStamped()),
              tfQuaternion(new tf2::Quaternion())
    {
        pubImageColor = it.advertise(imageRGBTopic, 10);
        pubImageDepth = it.advertise(imageDepthTopic, 10);
        //pubImageIr = it.advertise(imageIrTopic, 10);
        syncApproximate->registerCallback(boost::bind(&XtionAdapterNodelet::driverNodeCallback, this,
                                                      _1, _2, _3, _4));
        //pubCvImageColor = it.advertise(cvImageRGBTopic, 10);
        //pubCvImageDepth = it.advertise(cvImageDepthTopic, 10);
        //pubCameraInfoColor = nh.advertise<sensor_msgs::CameraInfo>(imageRGBCameraInfoTopic, 10);
        //pubCameraInfoDepth = nh.advertise<sensor_msgs::CameraInfo>(imageDepthCameraInfoTopic, 10);
        //pubPointCloud.reset(new sensor_msgs::Image());

        //imageScaledDepth.reset(new sensor_msgs::Image());
        //imageScaledDepthCameraInfo.reset(new sensor_msgs::CameraInfo());
    }

    XtionAdapterNodelet::~XtionAdapterNodelet() {}

    void XtionAdapterNodelet::onInit() {
        NODELET_INFO("XtionAdapterNodelet initialized");
    }

    void XtionAdapterNodelet::driverNodeletCallback(const sensor_msgs::Image::ConstPtr imageColor,
                                                    const sensor_msgs::Image::ConstPtr imageDepth,
                                                    const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                                    const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
       // this->imageRGB = imageColor;
        //this->imageDepth.reset(&(*imageDepth));
        //this->imageRGBCameraInfo.reset(&(*cameraInfoColor));
        //this->imageDepthCameraInfo.reset(&(*cameraInfoDepth));

        //NODELET_INFO("CALLBACK %p %p %p %p", imageColor, imageDepth, cameraInfoColor, cameraInfoDepth);
    }

    void XtionAdapterNodelet::driverNodeCallback(sensor_msgs::Image::ConstPtr imageColor,
                                                 sensor_msgs::Image::ConstPtr imageDepth,
                                                 sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                                 sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {

        const clock_t begin_time = clock();

        this->imageRGBCameraInfo->header = cameraInfoColor->header;
        this->imageRGBCameraInfo->height = cameraInfoColor->height;
        this->imageRGBCameraInfo->width = cameraInfoColor->width;
        this->imageRGBCameraInfo->distortion_model = cameraInfoColor->distortion_model;
        this->imageRGBCameraInfo->roi = cameraInfoColor->roi;
        this->imageRGBCameraInfo->binning_x = cameraInfoColor->binning_x;
        this->imageRGBCameraInfo->binning_y = cameraInfoColor->binning_y;
        this->imageRGBCameraInfo->K = cameraInfoColor->K;
        this->imageRGBCameraInfo->D = cameraInfoColor->D;
        this->imageRGBCameraInfo->P = cameraInfoColor->P;
        this->imageRGBCameraInfo->R = cameraInfoColor->R;

        this->imageScaledDepthCameraInfo.header = cameraInfoDepth->header;
        this->imageScaledDepthCameraInfo.width = cameraInfoColor->width;
        this->imageScaledDepthCameraInfo.height = cameraInfoColor->height;
        float x_ratio = float(imageColor->width) / float(imageDepth->width);
        float y_ratio = float(imageColor->height) / float(imageDepth->height);
        this->imageScaledDepthCameraInfo.K[0] = cameraInfoDepth->K[0] * x_ratio;
        this->imageScaledDepthCameraInfo.K[2] = cameraInfoDepth->K[2] * x_ratio;
        this->imageScaledDepthCameraInfo.K[4] = cameraInfoDepth->K[4] * y_ratio;
        this->imageScaledDepthCameraInfo.K[5] = cameraInfoDepth->K[5] * y_ratio;
        this->imageScaledDepthCameraInfo.P[0] = cameraInfoDepth->P[0] * x_ratio;
        this->imageScaledDepthCameraInfo.P[2] = cameraInfoDepth->P[2] * x_ratio;
        this->imageScaledDepthCameraInfo.P[5] = cameraInfoDepth->P[5] * y_ratio;
        this->imageScaledDepthCameraInfo.P[6] = cameraInfoDepth->P[6] * y_ratio;

        try {

            cvImageColor = cv_bridge::toCvCopy(*imageColor.get(), sensor_msgs::image_encodings::BGRA8);
            cvImageScaledDepth = cv_bridge::toCvCopy(*imageDepth.get(), sensor_msgs::image_encodings::TYPE_32FC1);
            cvImageDepth = cv_bridge::toCvCopy(*imageDepth.get(), sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        cv::resize(cvImageDepth->image, cvImageScaledDepth->image, cv::Size(imageColor->width,
                                                                            imageColor->height));

        this->imageRGB = cvImageColor->toImageMsg();
        this->imageDepth = cvImageScaledDepth->toImageMsg();

        this->imageRGB->header.frame_id = adapterPubFrameId;
        this->imageDepth->header.frame_id = adapterPubFrameId;
        this->imageRGBCameraInfo->header.frame_id = adapterPubFrameId;
        this->imageScaledDepthCameraInfo.header.frame_id = adapterPubFrameId;

        pubImageColor.publish(this->imageRGB);
        pubImageDepth.publish(this->imageDepth);
        pubCameraInfoColor.publish(this->imageRGBCameraInfo);
        pubCameraInfoDepth.publish(this->imageScaledDepthCameraInfo);

        //pubCvImageColor.publish(cvImageColor->toImageMsg());
        //pubCvImageDepth.publish(cvImageScaledDepth->toImageMsg());

        //ROS_INFO("CALLBACK %p %p %p %p", this->imageRGB, this->imageDepth,
        //             this->imageRGBCameraInfo, this->imageDepthCameraInfo);

        //ROS_INFO("driverNodeCallback: %f Hz", 1 / (float(clock() - begin_time) /  CLOCKS_PER_SEC));

        if (!preparedMake3D) {
            prepareMake3D();
        }
        processPCLMessages();
        publishTf();
    }

    void XtionAdapterNodelet::publishTf() {

        tfStamped->header.frame_id = "/world";
        tfStamped->child_frame_id = adapterPubFrameId;
        tfStamped->transform.translation.x = staticTfX + tfTuningX;
        tfStamped->transform.translation.y = staticTfY + tfTuningY;
        tfStamped->transform.translation.z = staticTfZ + tfTuningZ;

        tfQuaternion->setRPY(staticTfRoll + tfTuningRoll, staticTfPitch + tfTuningPitch, staticTfYaw + tfTuningYaw);
        tfStamped->transform.rotation.x = tfQuaternion->x();
        tfStamped->transform.rotation.y = tfQuaternion->y();
        tfStamped->transform.rotation.z = tfQuaternion->z();
        tfStamped->transform.rotation.w = tfQuaternion->w();

        tfStamped->header.stamp = ros::Time::now();
        tfBroadcaster->sendTransform(*tfStamped);
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
        //cvImageScaledDepth.get()->image.copyTo(*big_mat);
        //cvImageColor.get()->image.copyTo(*m_color);

        //clock_t begin_time = clock();

        m_color.reset(new cv::Mat(1280, 1024, CV_8UC4, cvImageColor->image.data));
        m_depth.reset(new cv::Mat(len_1d, 1, CV_32F, cvImageScaledDepth->image.data));
        m_channels_as_one_float.reset(new cv::Mat(len_1d, 1, CV_32F, m_color->data));

        if (!pointCloudAssigned) {
            m_pointcloud_points.reset(new cv::Mat(len_1d, 8, CV_32F, pclPointCloud->points.data()));
            pointCloudAssigned = true;
        }

        *m_x_mapping = m_x_map->mul(*m_depth);
        *m_y_mapping = m_y_map->mul(*m_depth);
        m_x_mapping->col(0).copyTo(m_pointcloud_points->col(0));
        m_y_mapping->col(0).copyTo(m_pointcloud_points->col(1));
        m_depth->col(0).copyTo(m_pointcloud_points->col(2));
        m_channels_as_one_float->col(0).copyTo(m_pointcloud_points->col(4));

        //std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> vec;
        //m_pointcloud_points->col(0).copyTo(vec);
//
//
        //pclPointCloud->points = *(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >*) NULL;


        //ROS_INFO("processPCL1: %f Hz", 1.0 / (float(clock() - begin_time) /  CLOCKS_PER_SEC));

        //m_x_map = cv::Mat(len_1d , 1, CV_32F);
        //m_x_mapping = cv::Mat(len_1d, 1, CV_32F);
        //m_y_map = cv::Mat(len_1d, 1, CV_32F);
        //m_y_mapping = cv::Mat(len_1d, 1, CV_32F);

        //m_filler = cv::Mat(len_1d, 1, CV_32F);


        //m_padding = cv::Mat(len_1d, 1, CV_32F);


        //ROS_INFO("%p %p %p %p %p %p", &(*m_pointcloud_points), m_pointcloud_points->data, m_pointcloud_points->datastart,
        //         m_pointcloud_points->dataend, &(*pclPointCloud), &pclPointCloud->points);
        //ROS_INFO("%p %p %p %p %p", (void *)pclPointCloud->points.data(), (void *)&pclPointCloud->points.back(),
        //         (void *)&pclPointCloud->points.front(), &(*pclPointCloud->points.begin()),
        //         &(*pclPointCloud->points.end()));





        //std::cout << m_pt_cloud->type() << " " << m_pt_cloud->dims << std::endl;
        //std::cout << (void *)m_pt_cloud->data << std::endl;

        //begin_time = clock();
        //*concat_matrices = {*m_x_mapping, *m_y_mapping, *m_depth, *m_filler,
        //                    *m_channels_as_one_float, *m_padding, *m_padding, *m_padding};
        //ROS_INFO("processPCL2: %f Hz", 1.0 / (float(clock() - begin_time) /  CLOCKS_PER_SEC));



        //begin_time = clock();
        //cv::hconcat(*concat_matrices, *m_pointcloud_points);
        //ROS_INFO("processPCL4: %f Hz", 1.0 / (float(clock() - begin_time) /  CLOCKS_PER_SEC));....................................s

        // ajoute les index des pixel qui non pas un depth NaN
        std::vector<int> indx;
        //int count_a = 0;
        //for (int i = 0; i < m_depth->rows; i++) {
        //    if (!std::isnan(m_depth->at<float>(i, 0))) {
        //        indx.push_back(i);
        //        count_a++;
        //        if (count_a > 5) {
        //            break;
        //        }
        //    }
        //}



        //ROS_INFO("%p %p %p %p", &(*m_pointcloud_points), m_pointcloud_points->data, m_pointcloud_points->datastart,
        //         m_pointcloud_points->dataend);
        //begin_time = clock();
        //pcl::PointXYZRGB *pt = (pcl::PointXYZRGB *) m_pointcloud_points->datastart;
        //pclPointCloud->clear();
        //ROS_INFO("processPCL5: %f Hz", 1.0 / (float(clock() - begin_time) /  CLOCKS_PER_SEC));

        //begin_time = clock();
        //pclPointCloud->points.assign((pcl::PointXYZRGB *) m_pointcloud_points->datastart,
        //                             (pcl::PointXYZRGB *) m_pointcloud_points->dataend);
        //ROS_INFO("processPCL6: %f Hz", 1.0 / (float(clock() - begin_time) /  CLOCKS_PER_SEC));

        //*m_pt_cloud = cv::Mat(len_1d, 8, CV_32F, (unsigned char *) pclPointCloud->points.data());

        //p_f = (float_t *)pclPointCloud->points.data();
        //p_c = (uint8_t *)pclPointCloud->points.data();


        std::stringstream ss;
        //ss << std::endl << m;
        //ROS_INFO("%s", ss.str().c_str());




        //int offset = len_1d / 2;
        //int count = 0;
        //for (auto it = pclPointCloud->begin(); it != pclPointCloud->end(); ++it) {
        //    if (count > offset) {
        //        ROS_INFO("%f %f %f %d %d %d %d", it->x, it->y, it->z, it->b, it->g, it->r, it->a);
        //    }
        //    if (count - offset > 10) {
        //        break;
        //    } else {
        //        count++;
        //    }
        //}

        // interpreter m_pointcloud_points comme une mat de bits
        // et le comparer avec les valeurs de son équivalent  en point cloud


        //for (int i = 0; i < m_pointcloud_points->rows; i++) {
        //    ss.str(std::string());
        //    ss << m_pointcloud_points->row(i);
        //    ROS_INFO("{%d} %s", i, ss.str().c_str());
        //}


        //print les pointXYZRGBA qui ne sont pas de depth NaN
        float x_map, x_mapping, depth;
        pcl::PointXYZRGB point;
        //for (auto it = indx.begin(); it != indx.end(); ++it) {
        //    ss.str(std::string());
        //    ss << m_pointcloud_points->row(*it);
        //    depth = m_depth->at<float>(*it);
        //    x_map = m_x_map->at<float>(*it);
        //    x_mapping = m_x_mapping->at<float>(*it);
        //    point = pclPointCloud->points.at((unsigned long)*it);
        //    ROS_INFO("{%d}\n\t %s\n\t [%f %f %f] [%f %f %f %d %d %d %d]", *it, ss.str().c_str(),
        //             depth, x_map, x_mapping, point.x, point.y, point.z,
        //             point.r, point.g, point.b, point.a);
        //}

        // voir si les pixel de l'image
        //ss.str(std::string());
        //for (int i = 0; i < m_color->rows; i++) {
        //    for (int j = 0; j < m_color->cols; j++) {
        //        ss << m_color->at<cv::Vec4b>(i, j) << std::endl;
        //    }
        //}
        //ROS_INFO("%s", ss.str().c_str());

        //cv::Mat test_depth(len_1d, 1, CV_32F);
        //test_depth.setTo(2.0f);
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
        //NODELET_INFO("PROCESSING PCL Messages");

        //processPCL1();
        processPCL2();
        pclPointCloud->header.frame_id = adapterPubFrameId;
        //NODELET_INFO("PUBLISH PCL Messages");
        pubPointCloud.publish(pclPointCloud);
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

        //pclPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        //big_mat.reset(new cv::Mat(1280, 1024, CV_32F));
        //image_color.reset(new cv::Mat(1280, 1024, CV_8UC3));
        //m_x_map.reset(new cv::Mat(len_1d, 1, CV_32F));
        //m_x_mapping.reset(new cv::Mat(len_1d, 1, CV_32F));
        //m_y_map.reset(new cv::Mat(len_1d, 1, CV_32F));
        //m_y_mapping.reset(new cv::Mat(len_1d, 1, CV_32F));
        //m_filler.reset(new cv::Mat(len_1d, 1, CV_32F));
        //m_padding.reset(new cv::Mat(len_1d, 1, CV_32F));
        //m_pt_cloud.reset(new cv::Mat(len_1d, 8, CV_32F));
        //m_pointcloud_points.reset(new cv::Mat(len_1d, 8, CV_32F));
        //m_channels_as_one_float.reset(new cv::Mat(len_1d, 1, CV_32F, image_color->data));

        float *x_m_ptr = (float *) m_x_map->data;
        float *y_m_ptr = (float *) m_y_map->data;
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                *x_m_ptr++ = (float) ((i - imageScaledDepthCameraInfo.K[2] + 0.5) / imageScaledDepthCameraInfo.K[0]);
                *y_m_ptr++ = (float) ((j - imageScaledDepthCameraInfo.K[5] + 0.5) / imageScaledDepthCameraInfo.K[4]);
            }
        }

        m_filler->setTo(1.0f);
        m_padding->setTo(0.0f);
        m_filler->col(0).copyTo(m_pointcloud_points->col(3));
        m_padding->col(0).copyTo(m_pointcloud_points->col(5));
        m_padding->col(0).copyTo(m_pointcloud_points->col(6));
        m_padding->col(0).copyTo(m_pointcloud_points->col(7));

        pclPointCloud->points.assign((pcl::PointXYZRGB *) m_pointcloud_points->datastart,
                                     (pcl::PointXYZRGB *) m_pointcloud_points->dataend);

        //ROS_INFO("%p %p %p %p %p %p", &(*m_pointcloud_points), m_pointcloud_points->data, m_pointcloud_points->datastart,
        //         m_pointcloud_points->dataend, &(*pclPointCloud), &pclPointCloud->points);
        //ROS_INFO("%p %p %p %p %p", (void *)pclPointCloud->points.data(), (void *)&pclPointCloud->points.back(),
        //         (void *)&pclPointCloud->points.front(), &(*pclPointCloud->points.begin()),
        //         &(*pclPointCloud->points.end()));

        this->preparedMake3D = true;
    };

    int XtionAdapterNodelet::main(int argc, char **argv) {
        ROS_INFO("XtionAdapterNodelet initialized as node");
        ros::spin();
        return 0;
    }
}

