#ifndef SSC_SERVER_VOXBLOX_H_
#define SSC_SERVER_VOXBLOX_H_

#include <ros/ros.h>
#include <ssc_msgs/SSCGrid.h>
#include <voxblox/core/layer.h>
// #include <voxblox/integrator/merge_integration.h>
// #include <voxblox_ros/tsdf_server.h>
//#include <tf/transform_listener.h>

#include "ssc_mapping/core/ssc_map.h"

namespace voxblox {

class SSCServer {
   public:
    SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
        : SSCServer(nh, nh_private, SSCMap::Config()) {}
    SSCServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const SSCMap::Config& config);

    void sscCallback(const ssc_msgs::SSCGrid::ConstPtr& msg);

    void setWorldFrame(const std::string& world_frame) { world_frame_ = world_frame; }

    std::string getWorldFrame() const { return world_frame_; }

    virtual void clear() { ssc_map_->getSSCLayerPtr()->removeAllBlocks(); }

    inline std::shared_ptr<SSCMap> getSSCMapPtr() { return ssc_map_; }
    inline std::shared_ptr<const SSCMap> getSSCMapPtr() const { return ssc_map_; }

    void publishSSCOccupancyPoints();

    void publishSSCOccupiedNodes();

   private:
    bool publish_pointclouds_on_update_;
    std::string world_frame_;
    std::shared_ptr<SSCMap> ssc_map_;
    ros::Subscriber ssc_map_sub_;
    ros::Publisher ssc_pointcloud_pub_;
    ros::Publisher occupancy_marker_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    //tf::TransformListener tf_listener_;
};

}  // namespace voxblox

#endif  // SSC_SERVER_VOXBLOX_H_
