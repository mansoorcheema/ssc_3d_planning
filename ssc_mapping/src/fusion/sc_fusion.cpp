#include "ssc_mapping/fusion/sc_fusion.h"

namespace ssc_fusion {

SCFusion::SCFusion(float pred_conf, float max_weight, float prob_occupied, float prob_free, float min_prob, float max_prob)
    : pred_conf_(pred_conf),
      max_weight_(max_weight),
      prob_occupied_(prob_occupied),
      min_log_prob_(voxblox::logOddsFromProbability(min_prob)),
      max__log_prob_(voxblox::logOddsFromProbability(max_prob)){};

SCFusion::SCFusion(const BaseFusion::Config& config)
    : SCFusion(config.pred_conf, config.max_weight, config.prob_occupied, config.prob_free, config.min_prob, config.max_prob) {}

void SCFusion::fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence) {
    // only fuse label if its an object
    if (predicted_label > 0) {
        if (!voxel->observed || voxel->label > 0) {  // fuse only if the voxel is unknown or occupied
            if (predicted_label == voxel->label) {
                voxel->label_weight = std::min(voxel->label_weight + pred_conf_, max_weight_);
            } else if (voxel->label_weight < pred_conf_) {
                voxel->label_weight = pred_conf_ - voxel->label_weight;
                voxel->label = predicted_label;
            } else {
                voxel->label_weight = voxel->label_weight - pred_conf_;
            }
        }

        // fuse occupancy only if the voxel is unknown in global map
        if (!voxel->observed) {
            voxel->probability_log = voxblox::logOddsFromProbability(prob_occupied_);
            voxel->observed = true;
        }
    }

    //question:
    // how to fuse free space or consective occupancy as we dont have a global map here
}

}  // namespace ssc_fusion
