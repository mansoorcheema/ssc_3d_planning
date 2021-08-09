#include <iostream>
#include <voxblox_ros/tsdf_server.h>
#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_listener.h>

#include <ssc_msgs/SSCGrid.h>


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

    Layer<SSCOccupancyVoxel>* getSSCLayerPtr() { return ssc_layer_.get(); }
    const Layer<SSCOccupancyVoxel>* getSSCLayerConstPtr() const { return ssc_layer_.get(); }
    const Layer<SSCOccupancyVoxel>& getSSCLayer() const { return *ssc_layer_; }

    FloatingPoint block_size() const { return block_size_; }
    FloatingPoint voxel_size() const { return ssc_layer_->voxel_size(); }

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
        ssc_map_sub_ = nh_.subscribe("ssc", 1, &SSCServer::sscCallback, this);
    }

    void sscCallback(const ssc_msgs::SSCGrid::ConstPtr& msg) {

        std::cout<<"frame:"<<msg->frame<<std::endl;
        std::cout<<"Origin_x:"<<msg->origin_x<<std::endl;
        std::cout<<"Origin_y:"<<msg->origin_y<<std::endl;
        std::cout<<"Origin_z:"<<msg->origin_z<<std::endl;

        // std::cout<<"depth:"<<msg->depth<<std::endl;
        // std::cout<<"width:"<<msg->width<<std::endl;
        // std::cout<<"height:"<<msg->height<<std::endl;

        // todo - resize voxels to full size? Resize voxel grid from 64x36x64 to 240x144x240

        // completions are in odometry frame
        for (size_t x = 0; x < msg->depth; x++) {
            for (size_t y = 0; y < msg->height; y++) {
                for (size_t z = 0; z < msg->width; z++) {
                    size_t idx = x * msg->width * msg->height + y * msg->width + z;
                    uint cls = msg->data[idx];
                    voxblox::GlobalIndex voxelIdx(x + msg->origin_x, y + msg->origin_y, z + msg->origin_z);
                    //voxblox::SSCOccupancyVoxel* voxel = ssc_map_->getSSCLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);
                    voxblox::SSCOccupancyVoxel * voxel = ssc_map_->getSSCLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);
                    
                    // check if the block containing the voxel exists.
                    if (voxel == nullptr) {
                        //ssc_map_->getSSCLayerPtr()->a
                        voxblox::BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(voxelIdx, 1.0/ ssc_map_->getSSCLayerPtr()->voxels_per_side());
                        auto block = ssc_map_->getSSCLayerPtr()->allocateBlockPtrByIndex(block_idx);
                         const VoxelIndex local_voxel_idx =  getLocalFromGlobalVoxelIndex(voxelIdx, ssc_map_->getSSCLayerPtr()->voxels_per_side());
                        voxel = &block->getVoxelByVoxelIndex(local_voxel_idx);
                    }

                    if (!voxel->observed) {
                        voxel->label = cls;
                        voxel->class_confidence = 1.0;// todo - use confidence weight fusion like in SCFusion Paper
                        voxel->observed = true;
                    }
                }
            }
        }
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
