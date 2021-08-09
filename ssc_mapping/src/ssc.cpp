#include <iostream>
#include <voxblox_ros/tsdf_server.h>
#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_listener.h>

#include <ssc_msgs/SSCGrid.h>

std::unique_ptr<voxblox::TsdfServer> tsdf_server;

void SSCCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("I heard:Message Received");
}

namespace voxblox {

struct SSCOccupancyVoxel {
    float probability_log = 0.0f;
    bool observed = false;
    int label;
    float class_confidence;
};

class SSCMap {
   public:
    struct Config {
        FloatingPoint ssc_voxel_size = 0.3;
        size_t ssc_voxels_per_side = 16u;

        std::string print() const;
    };

    explicit SSCMap(const Config& config)
        : ssc_layer_(new Layer<SSCOccupancyVoxel>(config.ssc_voxel_size, config.ssc_voxels_per_side)) {
        block_size_ = config.ssc_voxel_size * config.ssc_voxels_per_side;
    }

    FloatingPoint block_size_;
    Layer<SSCOccupancyVoxel>::Ptr ssc_layer_;
};

class SSCServer {
   public:
    SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
        : SSCServer(nh, nh_private, SSCMap::Config()) {}
    SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const SSCMap::Config& config)
        : nh_(nh), nh_private_(nh_private) {
        ssc_map_.reset(new SSCMap(config));
        ssc_map_sub_ = nh_.subscribe("ssc2", 1, &SSCServer::sscCallback, this);
    }

    void sscCallback(const ssc_msgs::SSCGrid::ConstPtr& msg) {
        ROS_INFO("I heard:Message Received");
        // tf::StampedTransform transform;
        // tf_listener_.lookupTransform("/turtle2", "/turtle1", msg->header.stamp, transform);
        std::cout<<msg->frame<<std::endl;
    }

    std::shared_ptr<SSCMap> ssc_map_;
    ros::Subscriber ssc_map_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformListener tf_listener_;
};

// function definitions
std::string SSCMap::Config::print() const {
    std::stringstream ss;
    // clang-format off
  ss << "====================== SSCMap Map Config ========================\n";
  ss << " - ssc_voxel_size:               " << ssc_voxel_size << "\n";
  ss << " - ssc_voxels_per_side:          " << ssc_voxels_per_side << "\n";
  ss << "==============================================================\n";
    // clang-format on
    return ss.str();
}
}  // namespace voxblox

int main(int argc, char **argv) {
    std::cout<<"Hello World!"<<std::endl;
    ros::init(argc, argv, "ssc_mapping");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    //tsdf_server.reset(new voxblox::TsdfServer(nh, nh_private));
    std::unique_ptr<voxblox::SSCServer> ssc_server;
    ssc_server.reset(new voxblox::SSCServer(nh, nh_private));
    //ros::Subscriber sub = nh.subscribe("ssc", 1, SSCCallback);
    ros::spin();
    return 0;
}
