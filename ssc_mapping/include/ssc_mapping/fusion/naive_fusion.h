#ifndef SSC_NAIVE_FUSION_H_
#define SSC_NAIVE_FUSION_H_

#include "ssc_mapping/fusion/base_fusion.h"
#include <voxblox/core/common.h>

namespace ssc_fusion {
class NaiveFusion : public BaseFusion {
    public:
    virtual void fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence = 0.9f) override {
        // Fuse new measurements only if voxel is not obsrverd or is observed but empty
        if (!voxel->observed || (voxel->observed && voxel->label == 0)) {
            voxel->label = predicted_label;
            voxel->label_weight = 1.0;
            if (predicted_label > 0) {
                voxel->probability_log = voxblox::logOddsFromProbability(confidence);
            } 
            voxel->observed = true;
        }
    }
};
}  // namespace ssc_fusion

#endif  // SSC_NAIVE_FUSION_H_