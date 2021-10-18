#include "ssc_mapping/ros/ssc_server.h"

#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

#include "ssc_mapping/core/voxel.h"
#include "ssc_mapping/fusion/naive_fusion.h"
#include "ssc_mapping/fusion/occupancy_fusion.h"
#include "ssc_mapping/utils/voxel_utils.h"
#include "ssc_mapping/visualization/visualization.h"

namespace voxblox {

SSCServer::SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const ssc_fusion::BaseFusion::Config& fusion_config,  const SSCMap::Config& config)
    : nh_(nh), nh_private_(nh_private), publish_pointclouds_on_update_(true), ssc_topic_("ssc"), world_frame_("odom") {
    ssc_map_.reset(new SSCMap(config));

    if (fusion_config.fusion_strategy == ssc_fusion::strategy::log_odds) {
        base_fusion_.reset(new ssc_fusion::OccupancyFusion(fusion_config));
    } else if (fusion_config.fusion_strategy == ssc_fusion::strategy::naive) {
        base_fusion_.reset(new ssc_fusion::NaiveFusion());
    } else {
        LOG(WARNING) << "Wrong Fusion strategy provided. Using default fusion strategy";
        base_fusion_.reset(new ssc_fusion::OccupancyFusion(fusion_config));
    }

    // subscribe to SSC from node with 3D CNN 
    nh.param("ssc_topic", ssc_topic_, ssc_topic_);
    ssc_map_sub_ = nh_.subscribe(ssc_topic_, 1, &SSCServer::sscCallback, this);

    //publish fused maps as occupancy pointcloud
    ssc_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("occupancy_pointcloud", 1, true);

    //publish fused maps as occupancy nodes - marker array
    occupancy_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ssc_occupied_nodes", 1, true);

    save_map_srv_ = nh_private_.advertiseService(
      "save_map", &SSCServer::saveMapCallback, this);
}

ssc_fusion::BaseFusion::Config SSCServer::loadFusionConfigROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) {
    ssc_fusion::BaseFusion::Config fusion_config;

    nh.param("fusion_pred_conf", fusion_config.pred_conf,  fusion_config.pred_conf);
    nh.param("fusion_max_weight", fusion_config.max_weight, fusion_config.max_weight);
    nh.param("fusion_prob_occupied", fusion_config.prob_occupied, fusion_config.prob_occupied);
    nh.param("fusion_prob_free", fusion_config.prob_free, fusion_config.prob_free);
    nh.param("fusion_min_prob", fusion_config.min_prob, fusion_config.min_prob);
    nh.param("fusion_max_prob", fusion_config.max_prob, fusion_config.max_prob);
    nh.param("fusion_strategy", fusion_config.fusion_strategy, fusion_config.fusion_strategy);

    return fusion_config;
}

bool SSCServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(ssc_map_->getSSCLayer(), file_path);
}

bool SSCServer::saveMapCallback(voxblox_msgs::FilePath::Request& request,
                                 voxblox_msgs::FilePath::Response& ) { 
  return saveMap(request.file_path);
}

void SSCServer::sscCallback(const ssc_msgs::SSCGrid::ConstPtr& msg) {
    if (msg->origin_z < -1.5f) {  // a check to print if there is a wrong pose/outlier received
        LOG(WARNING) << "Outlier pose detected with origin at " << msg->origin_z << ". Skipping..";
        return;
    }

    // todo - resize voxels to full size? Resize voxel grid from 64x36x64 to 240x144x240
    // todo - update the  ssc_msgs::SSCGrid to contain the scale instead of hard coding
    // update - done - removed upsampling as its to slow and not needed
    // Layer<SSCOccupancyVoxel> temp_layer(ssc_map_->getSSCLayerPtr()->voxel_size() * 4,
    //                                    (ssc_map_->getSSCLayerPtr()->block_size()/ssc_map_->getSSCLayerPtr()->voxel_size())
    //                                    / 4);
    // Note: Not needed anymore

    // completions are in odometry frame
    // Note: numpy flattens an array (x,y,z) such that
    // X
    //  \
    //   \
    //    + ------------------> Z
    //    |
    //    |
    //    v
    //    Y axis
    // Input from simulation are X,Y,Z. X forward, Z up and Y left.
    // Its converted to grid coordinate system for
    // input to network as Y,Z,X and similarly
    // Network does predictions in Y,Z,X coordinates.
    // predictions in Y,Z,X are flattened using numpy (for sending as array)
    // These flattened predictions have are formated as:
    // first 240 would have z from 0 to 239, y=0, z=0
    // and similarly next 240 would have y=1,x=0
    // and so on. if we read here in numpy way way then output would
    // be in same Y,Z,X form.
    // Solution:
    // Either convert here from Y,Z,X to X,Y,Z and load in the same format
    // as numpy saved
    // Or  convert Y,Z,X -> X,Y,Z before sending and send transpose
    // of X,Y,Z from Numpy and load here with the x being fastest axis.
    for (size_t x = 0; x < msg->depth; x++) {
        for (size_t y = 0; y < msg->height; y++) {
            for (size_t z = 0; z < msg->width; z++) {
                size_t idx = x * msg->width * msg->height + y * msg->width + z;
                uint predicted_label = msg->data[idx];

                // transform from grid coordinate system to world coordinate system
                uint32_t world_orient_x = z;  // still uses scale of grid though rotation is in world coords
                uint32_t world_orient_y = x;
                uint32_t world_orient_z = y;

                auto grid_origin_index = getGridIndexFromOriginPoint<GlobalIndex>(
                    Point(msg->origin_x, msg->origin_y, msg->origin_z), ssc_map_->getSSCLayerPtr()->voxel_size_inv());

                GlobalIndex voxelIdx(world_orient_x, world_orient_y, world_orient_z);

                // add origin so that new voxels are integrated wrt origin
                voxelIdx += grid_origin_index;

                SSCOccupancyVoxel* voxel = ssc_map_->getSSCLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);

                // check if the block containing the voxel exists.
                if (voxel == nullptr) {
                    // ssc_map_->getSSCLayerPtr()->a
                    BlockIndex block_idx =
                        getBlockIndexFromGlobalVoxelIndex(voxelIdx, ssc_map_->getSSCLayerPtr()->voxels_per_side_inv());
                    auto block = ssc_map_->getSSCLayerPtr()->allocateBlockPtrByIndex(block_idx);
                    const VoxelIndex local_voxel_idx =
                        getLocalFromGlobalVoxelIndex(voxelIdx, ssc_map_->getSSCLayerPtr()->voxels_per_side());
                    voxel = &block->getVoxelByVoxelIndex(local_voxel_idx);
                }

                base_fusion_->fuse(voxel, predicted_label);
            }
        }
    }

    // merge the layer into the map. Used to upsample the predictions
    // note - upsampling slow so using larger voxel size than to upsample
    // to match orignal voxel size
    // mergeLayerAintoLayerB(temp_layer, ssc_map_->getSSCLayerPtr());

    if (publish_pointclouds_on_update_) {
        publishSSCOccupancyPoints();
        publishSSCOccupiedNodes();
    }
}

void SSCServer::publishSSCOccupancyPoints() {
    // Create a pointcloud with distance = intensity.
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    createPointcloudFromSSCLayer(ssc_map_->getSSCLayer(), &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    ssc_pointcloud_pub_.publish(pointcloud);
}

void SSCServer::publishSSCOccupiedNodes() {
    // Create a pointcloud with distance = intensity.
    visualization_msgs::MarkerArray marker_array;
    createOccupancyBlocksFromSSCLayer(ssc_map_->getSSCLayer(), world_frame_, &marker_array);
    occupancy_marker_pub_.publish(marker_array);
}

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
