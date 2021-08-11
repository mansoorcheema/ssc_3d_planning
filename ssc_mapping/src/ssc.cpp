#include <iostream>
#include <voxblox_ros/tsdf_server.h>
#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/merge_integration.h>

#include <ssc_msgs/SSCGrid.h>

#include <glog/logging.h>

#include "voxel.h"
#include "ssc_map.h"
#include "visualization.h"
#include "voxel_utils.h"

namespace voxblox {

class SSCServer {
   public:
    SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
        : SSCServer(nh, nh_private, SSCMap::Config()) {}
    SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const SSCMap::Config& config)
        : nh_(nh), nh_private_(nh_private) {
        ssc_map_.reset(new SSCMap(config));
        ssc_map_sub_ = nh_.subscribe("ssc", 1, &SSCServer::sscCallback, this);
        ssc_pointcloud_pub_ =  nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "occupancy_pointcloud", 1, true);

          occupancy_marker_pub_ = 
      nh_.advertise<visualization_msgs::MarkerArray>("ssc_occupied_nodes", 1, true);
    }

    void sscCallback(const ssc_msgs::SSCGrid::ConstPtr& msg) {
        std::cout << "frame:" << msg->frame << std::endl;
        std::cout << "Origin_x:" << msg->origin_x << std::endl;
        std::cout << "Origin_y:" << msg->origin_y << std::endl;
        std::cout << "Origin_z:" << msg->origin_z << std::endl;

        // todo - resize voxels to full size? Resize voxel grid from 64x36x64 to 240x144x240
        // todo - update the  ssc_msgs::SSCGrid to contain the scale instead of hard coding
        //Layer<SSCOccupancyVoxel> temp_layer(ssc_map_->getSSCLayerPtr()->voxel_size() * 4,
        //                                    (ssc_map_->getSSCLayerPtr()->block_size()/ssc_map_->getSSCLayerPtr()->voxel_size()) / 4);

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
                    uint cls = msg->data[idx];

                    // transform from grid coordinate system to world coordinate system
                    uint32_t world_orient_x = z; // still uses scale of grid though rotation is in world coords
                    uint32_t world_orient_y = x;
                    uint32_t world_orient_z = y;

                    auto grid_origin_index = getGridIndexFromOriginPoint<GlobalIndex>(Point(msg->origin_x, msg->origin_y, msg->origin_z), ssc_map_->getSSCLayerPtr()->voxel_size_inv());

                    GlobalIndex voxelIdx(world_orient_x , world_orient_y , world_orient_z);
                    voxelIdx += grid_origin_index;
                    
                    // voxblox::SSCOccupancyVoxel* voxel =
                    // ssc_map_->getSSCLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);
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
                    if (!voxel->observed || (voxel->observed && voxel->label == 0)) {
                        voxel->label = cls;
                        voxel->class_confidence = 1.0;  // todo - use confidence weight fusion like in SCFusion Paper
                        voxel->observed = true;
                    }
                }
            }
        }

        // merge the layer into the map. Used to upsample the predictions
        //mergeLayerAintoLayerB(temp_layer, ssc_map_->getSSCLayerPtr());

        publishSSCOccupancyPoints(); 
        publishSSCOccupiedNodes();
    }

    void publishSSCOccupancyPoints() {
        // Create a pointcloud with distance = intensity.
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
        createPointcloudFromSSCLayer(ssc_map_->getSSCLayer(), &pointcloud);

        pointcloud.header.frame_id = "odom";
        ssc_pointcloud_pub_.publish(pointcloud);
    }

    void publishSSCOccupiedNodes() {
        // Create a pointcloud with distance = intensity.
        visualization_msgs::MarkerArray marker_array;
        createOccupancyBlocksFromSSCLayer(ssc_map_->getSSCLayer(), "odom", &marker_array);
        occupancy_marker_pub_.publish(marker_array);
    }

private:
    std::shared_ptr<SSCMap> ssc_map_;
    ros::Subscriber ssc_map_sub_;
    ros::Publisher ssc_pointcloud_pub_;
    ros::Publisher occupancy_marker_pub_;
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
