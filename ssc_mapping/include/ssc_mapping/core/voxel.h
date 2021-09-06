
#ifndef SSC_VOXEL_H_
#define SSC_VOXEL_H_

namespace voxblox {

struct SSCOccupancyVoxel {
    float probability_log = 0.0f;
    bool observed = false;
    int label = -1;
    float label_weight = 0.0f;
};
}  // namespace voxblox
#endif //SSC_VOXEL_H_