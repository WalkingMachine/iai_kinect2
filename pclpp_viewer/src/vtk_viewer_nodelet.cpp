
#include "../include/pclpp_viewer/vtk_viewer_nodelet.h"

namespace pclpp_viewer {

    VTKViewerNodelet::VTKViewerNodelet(const bool useExact, const bool useCompressed) :
    useExact(useExact), useCompressed(useCompressed), topicColor(),
    updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
    nh("~"), spinner(0), it(nh), mode(VTKViewerNodelet::CLOUD) {
        nh.getParam("/pclpp_viewer/topic_to_listen_color", topicColor);
        nh.getParam("/pclpp_viewer/topic_to_listen_color", topicDepth);

        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        params.push_back((int)cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);
        params.push_back((int)cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(1);
        params.push_back((int)cv::IMWRITE_PNG_STRATEGY);
        params.push_back((int)cv::IMWRITE_PNG_STRATEGY_RLE);
        params.push_back(0);
    }

    VTKViewerNodelet::~VTKViewerNodelet() {}

    int VTKViewerNodelet::main(int argc, char **argv) {
        #if EXTENDED_OUTPUT
                ROSCONSOLE_AUTOINIT;
          if(!getenv("ROSCONSOLE_FORMAT"))
          {
            ros::console::g_formatter.tokens_.clear();
            ros::console::g_formatter.init("[${severity}] ${message}");
          }
        #endif

        ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

        if(!ros::ok())
        {
            return 0;
        }

        //std::string ns = K2_DEFAULT_NS;
        //std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
        //std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        bool useExact = true;
        bool useCompressed = false;
        VTKViewerNodelet::Mode mode = VTKViewerNodelet::CLOUD;

        for(size_t i = 1; i < (size_t)argc; ++i)
        {
            std::string param(argv[i]);

            if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
            {
                ROS_INFO("Help not implemented...");
                //help(argv[0]);
                ros::shutdown();
                return 0;
            }
            else if(param == "qhd")
            {
                //topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
                //topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
            }
            else if(param == "hd")
            {
                //topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
                //topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
            }
            else if(param == "ir")
            {
                //topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
                //topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
            }
            else if(param == "sd")
            {
                //topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
                //topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
            }
            else if(param == "approx")
            {
                useExact = false;
            }

            else if(param == "compressed")
            {
                useCompressed = true;
            }
            else if(param == "image")
            {
                mode = VTKViewerNodelet::IMAGE;
            }
            else if(param == "cloud")
            {
                mode = VTKViewerNodelet::CLOUD;
            }
            else if(param == "both")
            {
                mode = VTKViewerNodelet::BOTH;
            }
        }

        ROS_INFO("topic color: %s", topicColor.c_str());
        ROS_INFO("topic depth: %s", topicDepth.c_str());

        ROS_INFO("starting receiver...");
        run(mode);
        ros::shutdown();
        return 0;
    }

    void VTKViewerNodelet::onInit() {

    }

    void VTKViewerNodelet::run(const Mode mode) {
        start(mode);
        stop();
    }

    void VTKViewerNodelet::start(const Mode mode) {
        this->mode = mode;
        running = true;

        std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
        std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

        if(useExact)
        {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncExact->registerCallback(boost::bind(&VTKViewerNodelet::callback, this, _1, _2, _3, _4));
        }
        else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&VTKViewerNodelet::callback, this, _1, _2, _3, _4));
        }

        spinner.start();

        std::chrono::milliseconds duration(1);
        while(!updateImage || !updateCloud)
        {
            if(!ros::ok())
            {
                return;
            }
            std::this_thread::sleep_for(duration);
        }
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->height = (uint32_t) color.rows;
        cloud->width = (uint32_t) color.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height * cloud->width);
        createLookup((size_t)this->color.cols, (size_t)this->color.rows);

        switch(mode)
        {
            case CLOUD:
                cloudViewer();
                break;
            case IMAGE:
                imageViewer();
                break;
            case BOTH:
                imageViewerThread = std::thread(&VTKViewerNodelet::imageViewer, this);
                cloudViewer();
                break;
        }
    }

    void VTKViewerNodelet::stop() {
        spinner.stop();

        if(useExact)
        {
            delete syncExact;
        }
        else
        {
            delete syncApproximate;
        }

        delete subImageColor;
        delete subImageDepth;
        delete subCameraInfoColor;
        delete subCameraInfoDepth;

        running = false;
        if(mode == BOTH)
        {
            imageViewerThread.join();
        }
    }

    void VTKViewerNodelet::callback(const sensor_msgs::Image::ConstPtr imageColor,
                                    const sensor_msgs::Image::ConstPtr imageDepth,
                                    const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor,
                                    const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
        cv::Mat color, depth;

        readCameraInfo(cameraInfoColor, cameraMatrixColor);
        readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
        readImage(imageColor, color);
        readImage(imageDepth, depth);

        // IR image input
        if(color.type() == CV_16U)
        {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }

        lock.lock();
        this->color = color;
        this->depth = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();
    }


    void VTKViewerNodelet::imageViewer() {
        cv::Mat color, depth, depthDisp, combined;
        std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
        double fps = 0;
        size_t frameCount = 0;
        std::ostringstream oss;
        const cv::Point pos(5, 15);
        const cv::Scalar colorText(255, 255, 255, 0);
        const double sizeText = 0.5;
        const int lineText = 1;
        const int font = cv::FONT_HERSHEY_SIMPLEX;

        cv::namedWindow("Image Viewer");
        oss << "starting...";

        start = std::chrono::high_resolution_clock::now();
        for(; running && ros::ok();)
        {
            if(updateImage)
            {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateImage = false;
                lock.unlock();

                ++frameCount;
                now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
                if(elapsed >= 1.0)
                {
                    fps = frameCount / elapsed;
                    oss.str("");
                    oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                    start = now;
                    frameCount = 0;
                }

                dispDepth(depth, depthDisp, 12000.0f);
                combine(color, depthDisp, combined);
                //combined = color;

                cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
                cv::imshow("Image Viewer", combined);
            }

            int key = cv::waitKey(1);
            switch(key & 0xFF)
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    if(mode == IMAGE)
                    {
                        createCloud(depth, color, cloud);
                        saveCloudAndImages(cloud, color, depth, depthDisp);
                    }
                    else
                    {
                        save = true;
                    }
                    break;
                default: {};
            }
        }
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void VTKViewerNodelet::cloudViewer() {
        cv::Mat color, depth;
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        const std::string cloudName = "rendered";

        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->addPointCloud(cloud, cloudName);
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
        visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&VTKViewerNodelet::keyboardEvent, *this);

        for(; running && ros::ok();)
        {
            if(updateCloud)
            {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateCloud = false;
                lock.unlock();

                createCloud(depth, color, cloud);

                visualizer->updatePointCloud(cloud, cloudName);
            }
            if(save)
            {
                save = false;
                cv::Mat depthDisp;
                dispDepth(depth, depthDisp, 12000.0f);
                saveCloudAndImages(cloud, color, depth, depthDisp);
            }
            visualizer->spinOnce(10);
        }
        visualizer->close();
    }

    void VTKViewerNodelet::keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *) {
        if(event.keyUp())
        {
            switch(event.getKeyCode())
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    save = true;
                    break;
                default: {};
            }
        }
    }

    void VTKViewerNodelet::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void VTKViewerNodelet::readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo,
                                          cv::Mat &cameraMatrix) const {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for(size_t i = 0; i < 9; ++i, ++itC)
        {
            *itC = cameraInfo->K[i];
        }
    }

    void VTKViewerNodelet::dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue) {
        cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
        const uint32_t maxInt = 255;

#pragma omp parallel for
        for(int r = 0; r < in.rows; ++r)
        {
            const uint16_t *itI = in.ptr<uint16_t>(r);
            uint8_t *itO = tmp.ptr<uint8_t>(r);

            for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
            {
                *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
            }
        }

        cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
    }

    void VTKViewerNodelet::combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out) {
        out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

#pragma omp parallel for
        for(int r = 0; r < inC.rows; ++r)
        {
            const cv::Vec3b
                    *itC = inC.ptr<cv::Vec3b>(r),
                    *itD = inD.ptr<cv::Vec3b>(r);
            cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

            for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
            {
                itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
                itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
                itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
            }
        }
    }

    void VTKViewerNodelet::createCloud(const cv::Mat &depth, const cv::Mat &color,
                                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
        for(int r = 0; r < depth.rows; ++r)
        {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();

            for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
            {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if(*itD == 0)
                {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
                itP->a = 255;
            }
        }
    }

    void VTKViewerNodelet::saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
                                              const cv::Mat &color, const cv::Mat &depth,
                                              const cv::Mat &depthColored) {
        oss.str("");
        oss << "./" << std::setfill('0') << std::setw(4) << frame;
        const std::string baseName = oss.str();
        const std::string cloudName = baseName + "_cloud.pcd";
        const std::string colorName = baseName + "_color.jpg";
        const std::string depthName = baseName + "_depth.png";
        const std::string depthColoredName = baseName + "_depth_colored.png";

        ROS_INFO("saving cloud: %s", cloudName.c_str());
        writer.writeBinary(cloudName, *cloud);
        ROS_INFO("saving color: %s", colorName.c_str());
        cv::imwrite(colorName, color, params);
        ROS_INFO("saving depth: %s", depthName.c_str());
        cv::imwrite(depthName, depth, params);
        ROS_INFO("saving depth: %s", depthColoredName.c_str());
        cv::imwrite(depthColoredName, depthColored, params);
        ROS_INFO("saving complete!");
        ++frame;
    }

    void VTKViewerNodelet::createLookup(size_t width, size_t height) {
        const float fx = static_cast<const float>(1.0 / cameraMatrixColor.at<double>(0, 0));
        const float fy = static_cast<const float>(1.0f / cameraMatrixColor.at<double>(1, 1));
        const float cx = static_cast<const float>(cameraMatrixColor.at<double>(0, 2));
        const float cy = static_cast<const float>(cameraMatrixColor.at<double>(1, 2));
        float *it;

        lookupY = cv::Mat(1, (int)height, CV_32F);
        it = lookupY.ptr<float>();
        for(size_t r = 0; r < height; ++r, ++it)
        {
            *it = (r - cy) * fy;
        }

        lookupX = cv::Mat(1, (int)width, CV_32F);
        it = lookupX.ptr<float>();
        for(size_t c = 0; c < width; ++c, ++it)
        {
            *it = (c - cx) * fx;
        }
    }


}