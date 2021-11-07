#include "ssc_mapping/fusion/occupancy_fusion.h"

namespace ssc_fusion {

OccupancyFusion::OccupancyFusion(float pred_conf, float max_weight, float prob_occupied, float prob_free, float min_prob, float max_prob)
    : pred_conf_(pred_conf),
      max_weight_(max_weight),
      prob_occupied_(prob_occupied),
      prob_free_(prob_free),
      min_log_prob_(voxblox::logOddsFromProbability(min_prob)),
      max__log_prob_(voxblox::logOddsFromProbability(max_prob)){};

OccupancyFusion::OccupancyFusion(const BaseFusion::Config& config)
    : OccupancyFusion(config.pred_conf, config.max_weight, config.prob_occupied, config.prob_free, config.min_prob, config.max_prob) {}

void OccupancyFusion::fuse(voxblox::SSCOccupancyVoxel* voxel, uint predicted_label, float confidence, float weight) {
    voxel->observed = true;
    if (predicted_label > 0) {
        if (predicted_label == voxel->label) {
            voxel->label_weight = std::min(voxel->label_weight + pred_conf_, max_weight_);
        } else if (voxel->label_weight < pred_conf_) {
            voxel->label_weight = pred_conf_ - voxel->label_weight;
            voxel->label = predicted_label;
        } else {
            voxel->label_weight = voxel->label_weight - pred_conf_;
        }
    }

    // if voxel is predicted as occupied
    float log_odds_update = 0;
    if (predicted_label > 0) {
        // occupied voxel
        float prob_new = ((prob_occupied_ - 0.5f) * weight )+ 0.5f;
        log_odds_update = voxblox::logOddsFromProbability(prob_new);
    } else {
        // free voxel

        float prob_new = ((prob_free_ - 0.5f) * weight) + 0.5f;
        log_odds_update = voxblox::logOddsFromProbability(prob_new);
    }

    voxel->probability_log =
        std::min(std::max(voxel->probability_log + log_odds_update, min_log_prob_), max__log_prob_);
}

}  // namespace ssc_fusion
