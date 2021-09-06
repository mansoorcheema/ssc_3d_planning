#include "ssc_mapping/ros/ssc_server.h"

#include <voxblox/core/common.h>
#include <voxblox/core/voxel.h>

#include "ssc_mapping/core/voxel.h"
#include "ssc_mapping/utils/voxel_utils.h"
#include "ssc_mapping/visualization/visualization.h"

namespace voxblox {

SSCServer::SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const SSCMap::Config& config)
    : nh_(nh), nh_private_(nh_private), publish_pointclouds_on_update_(true), world_frame_("odom") {
    ssc_map_.reset(new SSCMap(config));
    ssc_map_sub_ = nh_.subscribe("ssc", 1, &SSCServer::sscCallback, this);
    ssc_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("occupancy_pointcloud", 1, true);

    occupancy_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ssc_occupied_nodes", 1, true);
}

void SSCServer::sscCallback(const ssc_msgs::SSCGrid::ConstPtr& msg) {
    // std::cout << "frame:" << msg->frame << std::endl;
    // std::cout << "origin_x:" << msg->origin_x << std::endl;
    // std::cout << "origin_y:" << msg->origin_y << std::endl;
    // std::cout << "origin_z:" << msg->origin_z << std::endl;

    if(msg->origin_z < -1.5f) {
        std::cout << "origin_z:" << msg->origin_z << std::endl;
    }

    // todo - resize voxels to full size? Resize voxel grid from 64x36x64 to 240x144x240
    // todo - update the  ssc_msgs::SSCGrid to contain the scale instead of hard coding
    // update - done - removed upsampling as its to slow and not needed
    // Layer<SSCOccupancyVoxel> temp_layer(ssc_map_->getSSCLayerPtr()->voxel_size() * 4,
    //                                    (ssc_map_->getSSCLayerPtr()->block_size()/ssc_map_->getSSCLayerPtr()->voxel_size())
    //                                    / 4);

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

                // Fuse new measurements only if voxel is not obsrverd or is observed but empty
                // if (!voxel->observed || (voxel->observed && voxel->label == 0)) {
                //     voxel->label = predicted_label;
                //     voxel->label_weight = 1.0;  // todo - use confidence weight fusion like in SCFusion Paper
                //     voxel->observed = true;
                // }


                float pred_conf = 0.7f;
                float max_weight = 50;
                float prob_occupied = 0.65f;
                float prob_free = 0.4f;
                
                voxel->observed = true;
                if (predicted_label > 0) {
                    if (predicted_label == voxel->label) {
                        voxel->label_weight = std::min(voxel->label_weight + pred_conf, max_weight);
                    } else if (voxel->label_weight < pred_conf) {
                        voxel->label_weight = pred_conf - voxel->label_weight;
                        voxel->label = predicted_label;
                    } else {
                        voxel->label_weight = voxel->label_weight - pred_conf;
                    }
                }

                // if voxel is predicted as occupied
                float log_odds_update = 0;
                if (predicted_label > 0) {
                    //occupied voxel
                    log_odds_update = logOddsFromProbability(prob_occupied);
                } else {
                    //free voxel
                    log_odds_update = logOddsFromProbability(prob_free);
                }
                
                voxel->probability_log = std::min( std::max(voxel->probability_log + log_odds_update, logOddsFromProbability(0.12f)), logOddsFromProbability(0.97f));
                
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
