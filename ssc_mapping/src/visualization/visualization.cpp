

#include "ssc_mapping/visualization/visualization.h"

namespace voxblox {

bool visualizeSSCOccupancyVoxels(const SSCOccupancyVoxel& voxel, const Point& /*coord*/, Color* color) {
    CHECK_NOTNULL(color);
    static SSCColorMap map;
    if (voxel.observed &&  voxel.probability_log > logOddsFromProbability(0.5f) && voxel.label > 0) { //log(0.7/0.3) = 0.3679f
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



void createOccupancyBlocksFromSSCLayer(const Layer<SSCOccupancyVoxel>& layer, const std::string& frame_id,
                                       visualization_msgs::MarkerArray* marker_array) {
    CHECK_NOTNULL(marker_array);
    createOccupancyBlocksFromLayer<SSCOccupancyVoxel>(layer, &visualizeSSCOccupancyVoxels, frame_id, marker_array);
}
}  // namespace voxblox
