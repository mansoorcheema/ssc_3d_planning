
#include <ros/ros.h>
#include <glog/logging.h>
#include "ssc_mapping/ros/ssc_server.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ssc_mapping");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, false);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    //tsdf_server.reset(new voxblox::TsdfServer(nh, nh_private));
    std::unique_ptr<voxblox::SSCServer> ssc_server;
    voxblox::SSCMap::Config config;
    config.ssc_voxel_size = 0.02 * 4; //from the SSC Network which uses 0.02 m as voxel size
    ssc_server.reset(new voxblox::SSCServer(nh, nh_private, config));
    //ros::Subscriber sub = nh.subscribe("ssc", 1, SSCCallback);
    ros::spin();
    return 0;
}
