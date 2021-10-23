#ifndef SSC_BASE_FUSION_H_
#define SSC_BASE_FUSION_H_

#include "ssc_mapping/core/voxel.h"

namespace ssc_fusion {
enum strategy {
    naive = 0,
    occupancy_fusion = 1,
    log_odds = 2,
    counting = 3,
    sc_fusion = 4
};

class BaseFusion {
   public:
    struct Config {
        // default confidence of a semantic label
        float pred_conf = 0.75f; 

        // max weight for semantic label. Each measurement adds default prediction score. 
        // The max weight is clipped to this value.
        float max_weight = 50.0f; 

        // default probability value for occupied voxelss
        float prob_occupied = 0.675f;

        // default probability value for free voxels
        float prob_free = 0.45f;

        // fusion strategy to fuse new measurements into a voxel
        int fusion_strategy = strategy::log_odds;

        float min_prob = 0.12f;

        float max_prob = 0.97f;
    };

    virtual void fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence = 0.51f) = 0;
};
}  // namespace ssc_fusion

#endif  // SSC_BASE_FUSION_H_