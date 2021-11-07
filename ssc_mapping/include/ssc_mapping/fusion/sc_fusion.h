#ifndef SSC_SC_FUSION_H_
#define SSC_SC_FUSION_H_

#include <voxblox/core/common.h>

#include "ssc_mapping/fusion/base_fusion.h"

namespace ssc_fusion {
/**
 * Log odds based occupancy fusion using SCFusion like fixed probabilities
 * foroccupied space. Semantics are fused like SCFusion.
 */
class SCFusion : public BaseFusion {
   public:
    SCFusion(const BaseFusion::Config& config);

    SCFusion(float pred_conf, float max_weight, float prob_occupied, float prob_free, float prob_min, float prob_max);

    virtual void fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence = 0.f, float weight = 0.0f) override;

   private:
    float min_log_prob_; // minumim log probability
    float max__log_prob_; // maximum threshold of log probability
    float pred_conf_; // weight for a single semantic prediction - ref: scfusion - uses confidence as weight for a semantic weight
    float max_weight_; // max aggregated label semantic weight
    float prob_occupied_; // constant probability to fuse occupied voxels
};
}  // namespace ssc_fusion

#endif  // SSC_SC_FUSION_H_