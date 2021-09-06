#include "ssc_planning/map/ssc_occ_map.h"

#include <voxblox/core/common.h>
#include <voxblox_ros/ros_params.h>
#include <active_3d_planning_core/data/system_constraints.h>

namespace active_3d_planning {
namespace map {

ModuleFactoryRegistry::Registration<SSCOccupancyMap> SSCOccupancyMap::registration(
    "SSCOccupancyMap");

SSCOccupancyMap::SSCOccupancyMap(PlannerI& planner) : OccupancyMap(planner) {}

voxblox::SSCServer& SSCOccupancyMap::getSSCServer() { return *ssc_server_; }

void SSCOccupancyMap::setupFromParamMap(Module::ParamMap* param_map) {
  // create an esdf server
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  voxblox::SSCMap::Config config;
  config.ssc_voxel_size = 0.02 * 4;
  ssc_server_.reset(new voxblox::SSCServer(nh, nh_private, config));

  // cache constants
  c_voxel_size_ = ssc_server_->getSSCMapPtr()->voxel_size();
  c_block_size_ = ssc_server_->getSSCMapPtr()->block_size();
}

bool SSCOccupancyMap::isTraversable(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    // double distance = 0.0;
    // if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
    //                                                          &distance)) {
    //   // This means the voxel is observed
    //   return (distance > planner_.getSystemConstraints().collision_radius);
    // }
    double collision_radius = planner_.getSystemConstraints().collision_radius;

    Eigen::Vector3d front(position.x() + collision_radius, 0, 0);
    Eigen::Vector3d back(position.x() - collision_radius, 0, 0);

    Eigen::Vector3d left(0, position.y() + collision_radius, 0);
    Eigen::Vector3d right(0, position.y() - collision_radius, 0);

    Eigen::Vector3d top(0, 0, position.z() + collision_radius);
    Eigen::Vector3d bottom(0, 0, position.z() + collision_radius);

    if (getVoxelState(top) == OccupancyMap::FREE && getVoxelState(bottom) == OccupancyMap::FREE &&
        getVoxelState(left) == OccupancyMap::FREE && getVoxelState(right) == OccupancyMap::FREE &&
        getVoxelState(front) == OccupancyMap::FREE && getVoxelState(back) == OccupancyMap::FREE) {
        return true;
    }

    return false;
}

bool SSCOccupancyMap::isObserved(const Eigen::Vector3d& point) {
  return ssc_server_->getSSCMapPtr()->isObserved(point);
}

// get occupancy
unsigned char SSCOccupancyMap::getVoxelState(const Eigen::Vector3d& point) {
    auto voxel =
        ssc_server_->getSSCMapPtr()->getSSCLayerPtr()->getVoxelPtrByCoordinates(point.cast<voxblox::FloatingPoint>());

    if (voxel == nullptr) 
      return OccupancyMap::UNKNOWN;
      
    if (voxel->observed) {
        if (voxel->probability_log > voxblox::logOddsFromProbability(0.5f)) {  // log(0.7/0.3) = 0.3679f
            return OccupancyMap::OCCUPIED;
        } else {
            return OccupancyMap::FREE;
        }
    } else {
        return OccupancyMap::UNKNOWN;
    }
}

// get voxel size
double SSCOccupancyMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool SSCOccupancyMap::getVoxelCenter(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = ssc_server_->getSSCMapPtr()
                                     ->getSSCLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

}  // namespace map
}  // namespace active_3d_planning
