#include "../include/pclpp_adapters/xtion_adapter_nodelet.h"

int main(int argc, char **argv)
{
    int i_arg = 0;
    std::string adapter_name = "";
    while (i_arg < argc) {
        if (boost::starts_with(argv[i_arg], "__name:")) {
            size_t header_len = strlen("__name:=");
            adapter_name = std::string(argv[i_arg]);
            adapter_name = adapter_name.substr(header_len, adapter_name.length() - header_len);
        }
        i_arg++;
    }

    ros::init(argc, argv, adapter_name.c_str());
    boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle(adapter_name));
    image_transport::ImageTransport it(*nh);

    pclpp_adapters::XtionAdapterNodelet xtionAdapter(adapter_name.c_str(), nh, it);

    ROS_INFO("starting node -- %s", adapter_name.c_str());

    return xtionAdapter.main(argc, argv);
}
