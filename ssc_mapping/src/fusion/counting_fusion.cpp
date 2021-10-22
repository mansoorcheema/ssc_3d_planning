#include "ssc_mapping/fusion/counting_fusion.h"

namespace ssc_fusion {

CountingFusion::CountingFusion(float pred_conf, float max_weight, float prob_occupied, float prob_free, float min_prob, float max_prob)
    : pred_conf_(pred_conf),
      max_weight_(max_weight),
      log_prob_occupied_(voxblox::logOddsFromProbability(prob_occupied)),
      log_prob_free_(voxblox::logOddsFromProbability(prob_free)){};

CountingFusion::CountingFusion(const BaseFusion::Config& config)
    : CountingFusion(config.pred_conf, config.max_weight, config.prob_occupied, config.prob_free, config.min_prob, config.max_prob) {}

void CountingFusion::fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence) {
    voxel->observed = true;
    if (predicted_label == voxel->label) {
        voxel->label_weight = std::min(voxel->label_weight + pred_conf_, max_weight_);
    } else if (voxel->label_weight < pred_conf_) {
        voxel->label_weight = pred_conf_ - voxel->label_weight;
        voxel->label = predicted_label;
    } else {
        voxel->label_weight = voxel->label_weight - pred_conf_;
    }

    voxel->probability_log = voxel->label > 0 ? log_prob_occupied_ : log_prob_free_;
}

}  // namespace ssc_fusion
