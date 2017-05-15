#include "../include/pclpp_viewer/vtk_viewer_nodelet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pclpp_viewer");
    pclpp_viewer::VTKViewerNodelet vtkViewer;

    ROS_INFO("HELLO WORLD -- vtk_viewer");

    return vtkViewer.main(argc, argv);
}

