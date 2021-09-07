#ifndef SSC_OCCUPANCY_FUSION_H_
#define SSC_OCCUPANCY_FUSION_H_

#include <voxblox/core/common.h>

#include "ssc_mapping/fusion/base_fusion.h"

namespace ssc_fusion {
/**
 * Log odds based occupancy fusion using SCFusion like fixed probabilities
 * for free and occupied space. Semantics are fused naively, also
 * like SCFusion.
 */
class OccupancyFusion : public BaseFusion {
   public:
    OccupancyFusion(const BaseFusion::Config& config);

    OccupancyFusion(float pred_conf, float max_weight, float prob_occupied, float prob_free, float prob_min, float prob_max);

    virtual void fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence = 0.f) override;

   private:
    float min_log_prob_;
    float max__log_prob_;
    float pred_conf_;
    float max_weight_;
    float prob_occupied_;
    float prob_free_;
};
}  // namespace ssc_fusion

#endif  // SSC_OCCUPANCY_FUSION_H_