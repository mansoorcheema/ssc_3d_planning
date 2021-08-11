#ifndef SSC_VISUALIZATION_H_
#define SSC_VISUALIZATION_H_

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox/utils/color_maps.h>

#include "voxel.h"
#include "color_map.h"

namespace voxblox {

bool visualizeSSCOccupancyVoxels(const SSCOccupancyVoxel& voxel, const Point& /*coord*/, Color* color) {
    CHECK_NOTNULL(color);
    static SSCColorMap map;
    if (voxel.observed && voxel.label > 0.f) {
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

template <typename VoxelType>
void createOccupancyBlocksFromLayer(const Layer<VoxelType>& layer,
                                    const ShouldVisualizeVoxelColorFunctionType<VoxelType>& vis_function,
                                    const std::string& frame_id, visualization_msgs::MarkerArray* marker_array) {
    CHECK_NOTNULL(marker_array);
    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;
    FloatingPoint voxel_size = layer.voxel_size();

    visualization_msgs::Marker block_marker;
    block_marker.header.frame_id = frame_id;
    block_marker.ns = "occupied_voxels";
    block_marker.id = 0;
    block_marker.type = visualization_msgs::Marker::CUBE_LIST;
    block_marker.scale.x = block_marker.scale.y = block_marker.scale.z = voxel_size;
    block_marker.action = visualization_msgs::Marker::ADD;

    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            Color color;
            if (vis_function(block.getVoxelByLinearIndex(linear_index), coord, &color)) {
                geometry_msgs::Point cube_center;
                cube_center.x = coord.x();
                cube_center.y = coord.y();
                cube_center.z = coord.z();
                block_marker.points.push_back(cube_center);
                std_msgs::ColorRGBA color_msg;
                colorVoxbloxToMsg(color, &color_msg);
                block_marker.colors.push_back(color_msg);
            }
        }
    }
    marker_array->markers.push_back(block_marker);
}

void createOccupancyBlocksFromSSCLayer(const Layer<SSCOccupancyVoxel>& layer, const std::string& frame_id,
                                       visualization_msgs::MarkerArray* marker_array) {
    CHECK_NOTNULL(marker_array);
    createOccupancyBlocksFromLayer<SSCOccupancyVoxel>(layer, &visualizeSSCOccupancyVoxels, frame_id, marker_array);
}
}  // namespace voxblox

#endif  // SSC_VISUALIZATION_H_