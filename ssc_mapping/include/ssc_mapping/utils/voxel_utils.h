#ifndef SSC_VOXEL_UTILS_H_
#define SSC_VOXEL_UTILS_H_

#include <voxblox/utils/evaluation_utils.h>
#include <voxblox/utils/voxel_utils.h>
#include <voxblox/interpolator/interpolator.h>

#include "ssc_mapping/core/voxel.h"

namespace voxblox {

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
        voxel.label_weight = 1.0f;
        voxel.observed = true;
    }

    return voxel;
}


// part 2 merging upsampled temp layer into voxel of map layer.
// Note; updated to use log probs and label fusion like in
// scfusion
 // namespace utils
template <>
void mergeVoxelAIntoVoxelB(const SSCOccupancyVoxel& voxel_A, SSCOccupancyVoxel* voxel_B) {
    voxel_B->label = voxel_A.label;
    voxel_B->label_weight = voxel_A.label_weight;
    voxel_B->observed = voxel_A.observed;
    voxel_B->probability_log = voxel_A.probability_log;
}

namespace utils {
template <>
bool isObservedVoxel(const SSCOccupancyVoxel& voxel) {
    return voxel.observed;
}
}  // namespace utils
}  // namespace voxblox

#endif //SSC_VOXEL_UTILS_H_