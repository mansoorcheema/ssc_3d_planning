/**
 * 
 * Note: This is specifically added for evaluating quality metrics of 
 * predicted map and needs both ground truth map and measured map. The
 * voxels observed in measured map are discarded from predicted voxels 
 * and the quality metrics are calculated on the remaining ones. The 
 * specified voxels are also discarded from ground truth observed voxels 
 * as we are not accounting those voxels in  evaluation.
 * For general quality metrics for layer agnostic evaluation use
 * ssc_map_eval_node and provide only the groundtruth layer and the target
 * layer for which the metrics have to be calculated i.e measured tsdf layer
 * or predicted ssc layer.
 * Mansoor. November 12. Week 25 Thesis.
 */ 

#include <tuple>
#include "ssc_mapping/eval/map_eval.h"
#include "ssc_mapping/utils/evaluation_utils.h"



std::string get_base_file_name(std::string path) {
    return path.substr(path.find_last_of("/\\") + 1, path.find_last_of(".") - path.find_last_of("/\\") - 1);
}

int main(int argc, char** argv) {
    
    if (argc < 6) {
        std::cout << "Usage: rosrun ssc_map_eval_quality_node <gt_layer> <measured_layer>  <observed_ssc_layer> "
                     "<quality_metrics_output> <refine_observed_layer>\n";
        return -1;
    }
    // init ros and google logging
    ros::init(argc, argv, "ssc_mapping_eval");
    google::InitGoogleLogging(argv[0]);
    google::SetCommandLineOption("GLOG_minloglevel", "0");
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, false);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    //parse command line arguments
    std::string gt_layer_path(argv[1]);
    std::string measured_layer_path(argv[2]);
    std::string observed_ssc_layer_path(argv[3]);
    std::string quality_metrics_path(argv[4]);
    bool refine_ob_layer;
    std::stringstream(argv[5]) >> refine_ob_layer;
    
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr ground_truth_layer;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr measured_layer;
    voxblox::Layer<voxblox::SSCOccupancyVoxel>::Ptr ssc_observed_layer;

    // load ground truth
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(gt_layer_path, &ground_truth_layer);
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(measured_layer_path, &measured_layer);
    voxblox::io::LoadLayer<voxblox::SSCOccupancyVoxel>(observed_ssc_layer_path, &ssc_observed_layer);

    //##########################################
    // Evaluation Metrics
    //##########################################
    
    VoxelEvalData voxel_observed_eval_data;
    VoxelEvalData voxel_measured_eval_data;
    
    voxel_measured_eval_data = get_voxel_data_from_layer(ground_truth_layer, measured_layer, refine_ob_layer);
    voxel_observed_eval_data = get_voxel_data_from_layer(ground_truth_layer, ssc_observed_layer, refine_ob_layer);
    
    IndexSet gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels, gt_obs_occ,
        gt_obs_free, gt_unobs_occ, gt_unobs_free;

    std::tie(gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels, gt_obs_occ, gt_obs_free,
                 gt_unobs_occ, gt_unobs_free) = voxel_observed_eval_data;
    
    //discard observed occupied voxels that are already measured
    map_obs_occ_voxels = ssc_mapping::evaluation::set_difference(map_obs_occ_voxels, std::get<2>(voxel_measured_eval_data));
    gt_obs_occ = ssc_mapping::evaluation::set_difference(gt_obs_occ, std::get<2>(voxel_measured_eval_data));
    
    //discard observed free voxels that are already measured
    map_obs_free_voxels = ssc_mapping::evaluation::set_difference(map_obs_free_voxels, std::get<3>(voxel_measured_eval_data));
    gt_obs_free = ssc_mapping::evaluation::set_difference(gt_obs_free, std::get<3>(voxel_measured_eval_data));
    
    auto quality_metrics = ssc_mapping::evaluation::calculate_quality_metrics(
        gt_occ_voxels, gt_obs_occ, gt_free_voxels, gt_obs_free, map_obs_occ_voxels, map_obs_free_voxels);

    // print metrics
    quality_metrics.print();

    //################################
    // Save Metrics to file
    //################################
    if (!quality_metrics_path.empty()) {
        LOG(INFO) << "Adding quality metrics to " << quality_metrics_path <<std::endl;
        // write quality metrics to file
        std::ofstream file(quality_metrics_path, std::ios::app);

        if (file.is_open()) {
            file << quality_metrics.precision_occ << "," << quality_metrics.precision_free << ","
                 << quality_metrics.precision_overall << "," << quality_metrics.recall_occ << ","
                 << quality_metrics.recall_free << "," << quality_metrics.IoU_occ << ","
                 << quality_metrics.IoU_free  << "," 
                 << get_base_file_name(measured_layer_path) << std::endl;
            file.close();
        }

        else
            LOG(ERROR) << "Unable to open " << quality_metrics_path << " file";
    }

    return 0;
}
