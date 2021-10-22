#ifndef SSC_COUNTING_FUSION_H_
#define SSC_COUNTING_FUSION_H_

#include <voxblox/core/common.h>

#include "ssc_mapping/fusion/base_fusion.h"

namespace ssc_fusion {
/**
 * Counting Fusion
 */
class CountingFusion : public BaseFusion {
   public:
    CountingFusion(const BaseFusion::Config& config);

    CountingFusion(float pred_conf, float max_weight, float prob_occupied, float prob_free, float prob_min, float prob_max);

    virtual void fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence = 0.f) override;

   private:
    float pred_conf_; // constant count for a single semantic prediction (=> 1)
    float max_weight_; // max counts
    float log_prob_occupied_; // constant log probability to fuse occupied voxels
    float log_prob_free_;// constant log_probability to fuse free voxels
};
}  // namespace ssc_fusion

#endif  // SSC_COUNTING_FUSION_H_