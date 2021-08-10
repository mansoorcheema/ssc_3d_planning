#include <iostream>
#include <voxblox_ros/tsdf_server.h>
#include <ros/ros.h>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/color_maps.h>

#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_listener.h>

#include <ssc_msgs/SSCGrid.h>

namespace voxblox {

struct SSCOccupancyVoxel {
    float probability_log = 0.0f;
    bool observed = false;
    int label = -1;
    float class_confidence = 0.0f;
};

class SSCMap {
   public:
    struct Config {
        FloatingPoint ssc_voxel_size = 0.2;
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

private:
    FloatingPoint block_size_;
    Layer<SSCOccupancyVoxel>::Ptr ssc_layer_;
};

void createPointcloudFromSSCLayer(const Layer<SSCOccupancyVoxel>& layer, pcl::PointCloud<pcl::PointXYZRGB>* pointcloud);

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
        std::cout << "frame:" << msg->frame << std::endl;
        std::cout << "Origin_x:" << msg->origin_x << std::endl;
        std::cout << "Origin_y:" << msg->origin_y << std::endl;
        std::cout << "Origin_z:" << msg->origin_z << std::endl;

        // todo - resize voxels to full size? Resize voxel grid from 64x36x64 to 240x144x240
        // todo - update the  ssc_msgs::SSCGrid to contain the scale instead of hard coding
        Layer<SSCOccupancyVoxel> temp_layer(ssc_map_->getSSCLayerPtr()->voxel_size() * 4,
                                            ssc_map_->getSSCLayerPtr()->block_size() / 4);

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
                    GlobalIndex voxelIdx(x + msg->origin_x, y + msg->origin_y, z + msg->origin_z);
                    // voxblox::SSCOccupancyVoxel* voxel =
                    // ssc_map_->getSSCLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);
                    SSCOccupancyVoxel* voxel = temp_layer.getVoxelPtrByGlobalIndex(voxelIdx);

                    // check if the block containing the voxel exists.
                    if (voxel == nullptr) {
                        // ssc_map_->getSSCLayerPtr()->a
                        BlockIndex block_idx =
                            getBlockIndexFromGlobalVoxelIndex(voxelIdx, 1.0 / temp_layer.voxels_per_side());
                        auto block = temp_layer.allocateBlockPtrByIndex(block_idx);
                        const VoxelIndex local_voxel_idx =
                            getLocalFromGlobalVoxelIndex(voxelIdx, temp_layer.voxels_per_side());
                        voxel = &block->getVoxelByVoxelIndex(local_voxel_idx);
                    }

                    if (!voxel->observed) {
                        voxel->label = cls;
                        voxel->class_confidence = 1.0;  // todo - use confidence weight fusion like in SCFusion Paper
                        voxel->observed = true;
                    }
                }
            }
        }

        // merge the layer into the map. Used to upsample the predictions
        mergeLayerAintoLayerB(temp_layer, ssc_map_->getSSCLayerPtr());
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
        //   createOccupancyBlocksFromTsdfLayer(tsdf_map_->getTsdfLayer(), world_frame_,
        //                                      &marker_array);
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

// part1 - used during upsampling. just pick largest class
// labels among voxels. Use default class prediction.
// like scfusion uses 0.51 for new predictions irespective
// of probs predicted by the network.
template <>
inline SSCOccupancyVoxel Interpolator<SSCOccupancyVoxel>::interpVoxel(const InterpVector& q_vector,
                                                                      const SSCOccupancyVoxel** voxels) {
    const int MAX_CLASSES = 12;
    SSCOccupancyVoxel voxel;

    uint32_t count_observed = 0;
    std::vector<uint16_t> preds(MAX_CLASSES, 0);
    for (int i = 0; i < q_vector.size(); ++i) {
        if (voxels[i]->observed) {
            count_observed++;
            preds[voxels[i]->label]++;
        }
    }

    // if at least of half of voxels are observed,
    // assign the maximum class among the voxels
    if (count_observed > q_vector.size() / 2) {
        auto max = std::max_element(preds.begin(), preds.end());
        auto idx = std::distance(preds.begin(), max);
        voxel.label = idx;
        voxel.class_confidence = 1.0f;
        voxel.observed = true;
    }

    return voxel;
}

// part 2 merging upsampled temp layer into voxel of map layer.
// Note; updated to use log probs and label fusion like in
// scfusion
template <>
void mergeVoxelAIntoVoxelB(const SSCOccupancyVoxel& voxel_A, SSCOccupancyVoxel* voxel_B) {
    voxel_B->label = voxel_A.label;
    voxel_B->class_confidence = voxel_A.class_confidence;
    voxel_B->observed = voxel_A.observed;
    voxel_B->probability_log = voxel_A.probability_log;
}

namespace utils {
template <>
bool isObservedVoxel(const SSCOccupancyVoxel& voxel) {
    return voxel.observed;
}
}  // namespace utils

class SSCColorMap : public IdColorMap {
   public:
    SSCColorMap() : IdColorMap() {
        colors_.push_back(Color(22, 191, 206));   // 0 empty, free space
        colors_.push_back(Color(214, 38, 40));    // 1 ceiling
        colors_.push_back(Color(43, 160, 4));     // 2 floor
        colors_.push_back(Color(158, 216, 229));  // 3 wall
        colors_.push_back(Color(114, 158, 206));  // 4 window
        colors_.push_back(Color(204, 204, 91));   // 5 chair  new: 180, 220, 90
        colors_.push_back(Color(255, 186, 119));  // 6 bed
        colors_.push_back(Color(147, 102, 188));  // 7 sofa
        colors_.push_back(Color(30, 119, 181));   // 8 table
        colors_.push_back(Color(188, 188, 33));   // 9 tvs
        colors_.push_back(Color(255, 127, 12));   // 10 furn
        colors_.push_back(Color(196, 175, 214));  // 11 objects
        colors_.push_back(Color(153, 153, 153));  // 12 Accessible area, or label==255, ignore
    }

    virtual Color colorLookup(const size_t value) const { return colors_[value]; }

   protected:
    std::vector<Color> colors_;
};

bool visualizeSSCOccupancyVoxels(const SSCOccupancyVoxel& voxel, const Point& /*coord*/, Color* color) {
    CHECK_NOTNULL(color);
    static SSCColorMap map;
    if (voxel.observed) {
        *color = map.colorLookup(voxel.label);
        return true;
    }
    return false;
}

void createPointcloudFromSSCLayer(const Layer<SSCOccupancyVoxel>& layer,
                                  pcl::PointCloud<pcl::PointXYZRGB>* pointcloud) {
    CHECK_NOTNULL(pointcloud);
    createColorPointcloudFromLayer<SSCOccupancyVoxel>(layer, &visualizeSSCOccupancyVoxels, pointcloud);
}

}  // namespace voxblox

int main(int argc, char **argv) {
    ros::init(argc, argv, "ssc_mapping");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    //tsdf_server.reset(new voxblox::TsdfServer(nh, nh_private));
    std::unique_ptr<voxblox::SSCServer> ssc_server;
    voxblox::SSCMap::Config config;
    config.ssc_voxel_size = 0.02; //from the SSC Network which uses 0.02 m as voxel size
    ssc_server.reset(new voxblox::SSCServer(nh, nh_private, config));
    //ros::Subscriber sub = nh.subscribe("ssc", 1, SSCCallback);
    ros::spin();
    return 0;
}
