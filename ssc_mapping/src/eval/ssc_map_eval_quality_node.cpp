#include <tuple>
#include "ssc_mapping/eval/map_eval.h"
#include "ssc_mapping/utils/evaluation_utils.h"

using IndexSet = voxblox::LongIndexSet;
typedef std::tuple<IndexSet, IndexSet, IndexSet, IndexSet, IndexSet, IndexSet, IndexSet, IndexSet> VoxelEvalData;

template <typename VoxelTypeA, typename VoxelTypeB>
VoxelEvalData get_voxel_data_from_layer(std::shared_ptr<voxblox::Layer<VoxelTypeA>> ground_truth_layer,
                                        std::shared_ptr<voxblox::Layer<VoxelTypeB>> observed_layer,
                                        bool refine_ob_layer) {
    CHECK(ground_truth_layer->voxel_size() == observed_layer->voxel_size())
        << "Error! Observed Layer and groundtruth layers should have same voxel size!";

    if (refine_ob_layer) {
        LOG(INFO) << "Updating observed layer by discarding voxels not observed in ground truth.";
        refine_observed_layer(*ground_truth_layer, observed_layer);
    }

    IndexSet gt_occ_voxels, gt_free_voxels;
    get_free_and_occupied_voxels_from_layer(*ground_truth_layer, &gt_occ_voxels, &gt_free_voxels);

    IndexSet map_obs_occ_voxels, map_obs_free_voxels;
    get_free_and_occupied_voxels_from_layer(*observed_layer, &map_obs_occ_voxels, &map_obs_free_voxels);

    IndexSet gt_obs_occ, gt_unobs_occ;
    split_observed_unobserved_voxels(*observed_layer, gt_occ_voxels, &gt_obs_occ, &gt_unobs_occ);

    IndexSet gt_obs_free, gt_unobs_free;
    split_observed_unobserved_voxels(*observed_layer, gt_free_voxels, &gt_obs_free, &gt_unobs_free);

    return {gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels,
            gt_obs_occ,    gt_obs_free,    gt_unobs_occ,       gt_unobs_free};
}

std::string get_base_file_name(std::string path) {
    return path.substr(path.find_last_of("/\\") + 1, path.find_last_of(".") - path.find_last_of("/\\") - 1);
}

int main(int argc, char** argv) {
    
    if (argc < 7) {
        std::cout << "Usage: rosrun ssc_map_eval_node <gt_layer> <observed_layer>  <observed_ssc_layer> <quality_metrics_output> "
                     "<publish_visualization> <refine_observed_layer>";
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
    std::string observed_layer_path(argv[2]);
    std::string observed_ssc_layer_path(argv[3]);
    std::string quality_metrics_path(argv[4]);
    bool publish_visualization;
    std::stringstream(argv[5]) >> publish_visualization;
    bool refine_ob_layer;
    std::stringstream(argv[6]) >> refine_ob_layer;
    
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr ground_truth_layer;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr observed_layer;
    voxblox::Layer<voxblox::SSCOccupancyVoxel>::Ptr ssc_observed_layer;

    // load ground truth
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(gt_layer_path, &ground_truth_layer);
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(observed_layer_path, &observed_layer);
    voxblox::io::LoadLayer<voxblox::SSCOccupancyVoxel>(observed_layer_path, &ssc_observed_layer);

    //##########################################
    // Evaluation Metrics
    //##########################################
    
    VoxelEvalData voxel_observed_eval_data;
    VoxelEvalData voxel_eval_data;
    
    voxel_eval_data = get_voxel_data_from_layer(ground_truth_layer, observed_layer, refine_ob_layer);
    voxel_observed_eval_data = get_voxel_data_from_layer(ground_truth_layer, ssc_observed_layer, refine_ob_layer);
    
    IndexSet gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels, gt_obs_occ,
        gt_obs_free, gt_unobs_occ, gt_unobs_free;

    std::tie(gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels, gt_obs_occ, gt_obs_free,
                 gt_unobs_occ, gt_unobs_free) = voxel_observed_eval_data;
    
    //discard observed occupied voxels that are already measured
    map_obs_occ_voxels = ssc_mapping::evaluation::set_difference(map_obs_occ_voxels, std::get<2>(voxel_eval_data));
    gt_obs_occ = ssc_mapping::evaluation::set_difference(gt_obs_occ, std::get<2>(voxel_eval_data));
    
    //discard observed free voxels that are already measured
    map_obs_free_voxels = ssc_mapping::evaluation::set_difference(map_obs_free_voxels, std::get<3>(voxel_eval_data));
    gt_obs_free = ssc_mapping::evaluation::set_difference(gt_obs_free, std::get<3>(voxel_eval_data));
    
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
                 << get_base_file_name(observed_layer_path) << std::endl;
            file.close();
        }

        else
            LOG(ERROR) << "Unable to open " << quality_metrics_path << " file";
    }

    return 0;
}
