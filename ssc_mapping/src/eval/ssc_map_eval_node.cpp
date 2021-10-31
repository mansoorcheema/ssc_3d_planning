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
        std::cout << "Usage: rosrun ssc_map_eval_node <gt_layer> <observed_layer> <quality_metrics_output> "
                     "<coverage_metrics_output> <publish_visualization> <refine_observed_layer>";
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
    std::string quality_metrics_path(argv[3]);
    std::string coverage_metrics_path(argv[4]);
    bool publish_visualization;
    std::stringstream(argv[5]) >> publish_visualization;
    bool refine_ob_layer;
    std::stringstream(argv[6]) >> refine_ob_layer;
    
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr ground_truth_layer;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr observed_layer;
    voxblox::Layer<voxblox::SSCOccupancyVoxel>::Ptr ssc_observed_layer;

    // load ground truth
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(gt_layer_path, &ground_truth_layer);

    //##########################################
    // Evaluation Metrics
    //##########################################
    
    VoxelEvalData voxel_eval_data;
    
    if (observed_layer_path.find(voxblox::voxel_types::kSSCOccupancy) == std::string::npos) {
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(observed_layer_path, &observed_layer);
        voxel_eval_data = get_voxel_data_from_layer(ground_truth_layer, observed_layer, refine_ob_layer);
    } else {
        voxblox::io::LoadLayer<voxblox::SSCOccupancyVoxel>(observed_layer_path, &ssc_observed_layer);
        voxel_eval_data = get_voxel_data_from_layer(ground_truth_layer, ssc_observed_layer, refine_ob_layer);
    }

    IndexSet gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels, gt_obs_occ,
        gt_obs_free, gt_unobs_occ, gt_unobs_free;

    std::tie(gt_occ_voxels, gt_free_voxels, map_obs_occ_voxels, map_obs_free_voxels, gt_obs_occ, gt_obs_free,
                 gt_unobs_occ, gt_unobs_free) = voxel_eval_data;

    auto quality_metrics = ssc_mapping::evaluation::calculate_quality_metrics(
        gt_occ_voxels, gt_obs_occ, gt_free_voxels, gt_obs_free, map_obs_occ_voxels, map_obs_free_voxels);
    auto coverage_metrics = ssc_mapping::evaluation::calculate_coverage_metrics(
        gt_occ_voxels, gt_obs_occ, gt_free_voxels, gt_obs_free, map_obs_occ_voxels, map_obs_free_voxels);

    // print metrics
    quality_metrics.print();
    coverage_metrics.print();

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

    if (!coverage_metrics_path.empty()) {
        LOG(INFO) << "Adding coverage metrics to " << coverage_metrics_path <<std::endl;
        // write coverage metrics to file
        std::ofstream file(coverage_metrics_path, std::ios::app);

        if (file.is_open()) {
            file << coverage_metrics.explored_occ << "," << coverage_metrics.explored_free << ","
                 << coverage_metrics.explored_overall << "," << coverage_metrics.coverage_occ << ","
                 << coverage_metrics.coverage_free << "," << coverage_metrics.coverage_overall << ","
                 << get_base_file_name(observed_layer_path) << std::endl;
            file.close();
        }

        else
            LOG(ERROR) << "Unable to open " << coverage_metrics_path << " file";
    }

    //#####################################
    // ROS Layer Visualizations
    //#####################################
    if (publish_visualization) {  // publish

        LOG(INFO) << "Publishing voxels visualization!" <<std::endl;

        auto missed_occupancy_observed_voxels_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("occupancy_pointcloud_observed_diff", 1, true);
        auto missed_occupancy_un_observed_voxels_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("occupancy_pointcloud_un_observed_diff", 1, true);
        auto correct_occupied_voxels_observed_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("occupancy_pointcloud_inter", 1, true);
        auto false_positive_observations_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("false_positive_observations", 1, true);
        auto free_gt_voxels_pub = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("gt_free_voxels", 1, true);
        auto occ_gt_voxels_pub = nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("gt_occ_voxels", 1, true);
        auto unobserved_free_voxels_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("unobserved_free_voxels", 1, true);

        // publish voxels that are occupied in ground truth but are unoccupied and "observed" in observed map
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_observed_diff;
        ssc_mapping::createPointCloudFromVoxelIndices(ssc_mapping::evaluation::set_intersection(map_obs_free_voxels, gt_occ_voxels), &pointcloud_observed_diff, voxblox::Color::Red());
        missed_occupancy_observed_voxels_pub.publish(pointcloud_observed_diff);

        // publish voxels that are occupied in ground truth but are unoccupied and "not observed" in observed map
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_un_observed_occupied;
        ssc_mapping::createPointCloudFromVoxelIndices(gt_unobs_occ, &pointcloud_un_observed_occupied, voxblox::Color::Orange());
        missed_occupancy_un_observed_voxels_pub.publish(pointcloud_un_observed_occupied);

        //correctly observed occupied voxels
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_correct_occ;
        ssc_mapping::createPointCloudFromVoxelIndices(ssc_mapping::evaluation::set_intersection(map_obs_occ_voxels, gt_occ_voxels), &pointcloud_correct_occ, voxblox::Color::Green());
        correct_occupied_voxels_observed_pub.publish(pointcloud_correct_occ);

        // false positive occupancy observations
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_observed_occupancy_fp;
        ssc_mapping::createPointCloudFromVoxelIndices(ssc_mapping::evaluation::set_difference(map_obs_occ_voxels,gt_occ_voxels), &pointcloud_observed_occupancy_fp, voxblox::Color::Yellow());
        false_positive_observations_pub.publish(pointcloud_observed_occupancy_fp);

        // free voxels in ground truth map
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_free_gt_voxels;
        ssc_mapping::createPointCloudFromVoxelIndices(gt_free_voxels, &pointcloud_free_gt_voxels,
                                                      voxblox::Color::White());
        free_gt_voxels_pub.publish(pointcloud_free_gt_voxels);

        // occupied voxels in ground truth map
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_occ_gt_voxels;
        ssc_mapping::createPointCloudFromVoxelIndices(gt_occ_voxels, &pointcloud_occ_gt_voxels, voxblox::Color::Gray());
        occ_gt_voxels_pub.publish(pointcloud_occ_gt_voxels);
        
        ros::spin();
    }

    return 0;
}
