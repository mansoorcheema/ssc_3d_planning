#ifndef SSC_VOXBLOX_3D_PLANNING_MAP_H_
#define SSC_VOXBLOX_3D_PLANNING_MAP_H_

#include <memory>

#include <active_3d_planning_core/module/module_factory_registry.h>
#include <ssc_mapping/ros/ssc_server.h>
#include <voxblox_ros/esdf_server.h>
#include <active_3d_planning_core/map/occupancy_map.h>

namespace active_3d_planning {
namespace map {

// Voxblox as a map representation
class SSCVoxbloxOccupancyMap : public OccupancyMap {
 public:
  explicit SSCVoxbloxOccupancyMap(PlannerI& planner);  // NOLINT

  // implement virtual methods
  void setupFromParamMap(Module::ParamMap* param_map) override;

  // check collision for a single pose
  bool isTraversable(const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation) override;

  // check whether point is part of the map
  bool isObserved(const Eigen::Vector3d& point) override;

  // get occupancy
  unsigned char getVoxelState(const Eigen::Vector3d& point) override;

  // get voxel size
  double getVoxelSize() override;

  // get the center of a voxel from input point
  bool getVoxelCenter(Eigen::Vector3d* center,
                      const Eigen::Vector3d& point) override;

  // accessor to the server for specialized planners
  voxblox::SSCServer& getSSCServer();

  voxblox::EsdfServer& getESDFServer();

 protected:
  static ModuleFactoryRegistry::Registration<SSCVoxbloxOccupancyMap> registration;

  // esdf server that contains the map, subscribe to external ESDF/TSDF updates
  std::unique_ptr<voxblox::SSCServer> ssc_server_;

  std::unique_ptr<voxblox::EsdfServer> esdf_server_;

  // cache constants
  double c_voxel_size_;
  double c_block_size_;
};

}  // namespace map
}  // namespace active_3d_planning

#endif  // SSC_VOXBLOX_3D_PLANNING_MAP_H_
