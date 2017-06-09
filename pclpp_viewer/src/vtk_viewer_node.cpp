#include "../include/pclpp_viewer/vtk_viewer_nodelet.h"

int main(int argc, char **argv)
{
    int i_arg = 0;
    std::string viewer_name = "";
    while (i_arg < argc) {
        if (boost::starts_with(argv[i_arg], "__name:")) {
            size_t header_len = strlen("__name:=");
            viewer_name = std::string(argv[i_arg]);
            viewer_name = viewer_name.substr(header_len, viewer_name.length() - header_len);
        }
        i_arg++;
    }
    
    ros::init(argc, argv, viewer_name.c_str());
    boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle(viewer_name));
    image_transport::ImageTransport it(*nh);
    
    pclpp_viewer::VTKViewerNodelet vtkViewer(viewer_name, nh, it);

    ROS_INFO("starting viewer node -- %s", viewer_name.c_str());

    return vtkViewer.main(argc, argv);
}

