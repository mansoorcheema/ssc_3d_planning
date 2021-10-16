
#include "ssc_mapping/eval/map_eval.h"
#include "ssc_mapping/utils/evaluation_utils.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ssc_mapping_eval");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, false);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    if (argc < 3) {
        std::cout << "Usage: rosrun ssc_map_eval_node <gt_layer> <observed_layer> <optional_output> "
                     "<optional_publish_stats> <optional_refine_observed_layer>";
        return -1;
    }

    voxblox::Layer<voxblox::TsdfVoxel>::Ptr ground_truth_layer;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr observed_layer;

    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(argv[1], &ground_truth_layer);
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(argv[2], &observed_layer);

    CHECK(ground_truth_layer->voxel_size() == observed_layer->voxel_size())
        << "Error! Observed Layer and groundtruth layers should have same voxel size!";

    if (argc > 5 && std::string("true").compare(argv[5]) == 0) {
        refine_observed_layer(*ground_truth_layer, observed_layer);
    }

    // start calculating metrics from here

    voxblox::LongIndexSet gt_occ_voxels, gt_free_voxels;
    get_free_and_occupied_voxels_from_layer(*ground_truth_layer, &gt_occ_voxels, &gt_free_voxels);

    voxblox::LongIndexSet map_obs_occ_voxels, map_obs_free_voxels;
    get_free_and_occupied_voxels_from_layer(*observed_layer, &map_obs_occ_voxels, &map_obs_free_voxels);

    voxblox::LongIndexSet gt_obs_occ, temp;
    split_observed_unobserved_voxels(*observed_layer, gt_occ_voxels, &gt_obs_occ, &temp);

    voxblox::LongIndexSet gt_obs_free;
    split_observed_unobserved_voxels(*observed_layer, gt_free_voxels, &gt_obs_free, &temp);

    auto quality_metrics = ssc_mapping::evaluation::calculate_quality_metrics(
        gt_occ_voxels, gt_obs_occ, gt_free_voxels, gt_obs_free, map_obs_occ_voxels, map_obs_free_voxels);
    auto coverage_metrics = ssc_mapping::evaluation::calculate_coverage_metrics(
        gt_occ_voxels, gt_obs_occ, gt_free_voxels, gt_obs_free, map_obs_occ_voxels, map_obs_free_voxels);

    // print metrics
    quality_metrics.print();
    coverage_metrics.print();

    // voxblox::GlobalIndexVector difference_observed_gt, different_unobserved_gt;// Note:here
    // // observed and un observed mean in the voxblox generated observed map,  not in gt
    // split_observed_unobserved_voxels(*observed_layer, difference_gt, &difference_observed_gt,
    // &different_unobserved_gt);

    // voxblox::GlobalIndexVector intersection_gt, difference_gt;
    // calculate_Intersection_difference(*ground_truth_layer, *observed_layer, &intersection_gt,
    //                                   &difference_gt);

    // voxblox::GlobalIndexVector intersection_observed, difference_observed;
    // calculate_Intersection_difference(*observed_layer, *ground_truth_layer,
    //                                   &intersection_observed, &difference_observed);
    // //prune irrelevant voxels
    // // pruneInsideVoxels(*observed_layer, &difference_observed);//remove voxels inside walls
    // // pruneOutlierVoxels(*ground_truth_layer,&difference_observed); // remvoe voxels that lie outside gt bound

    // voxblox::GlobalIndexVector difference_observed_gt, different_unobserved_gt;// Note:here
    // // observed and un observed mean in the voxblox generated observed map,  not in gt
    // split_observed_unobserved_voxels(*observed_layer, difference_gt, &difference_observed_gt,
    // &different_unobserved_gt);

    // voxblox::GlobalIndexVector gt_free_voxels;
    // calculateFreeObservedVoxels(ground_truth_map->getTsdfLayer(), &gt_free_voxels);

    // voxels that are not observed in observed map but are free space in
    // ground truth map. In ground truth a free space is either empty voxel or has weight < 1e-6. Thats not the case for
    // observed layer. Its because voxblox_ground_truth does not fill free space with voxels. There is option to fill
    // but that produces false occupied voxels. voxblox::GlobalIndexVector unobserved_unoccupied_voxels;
    // find_unobserved_free_voxels(*ground_truth_layer, *ground_truth_layer, &unobserved_unoccupied_voxels);

    // voxblox::GlobalIndexVector gt_occ_voxels, gt_free_voxels;
    // computeFreeOccupiedSpaceFrontier(ground_truth_map->getTsdfLayer(), Point(0.0,0.0,0.0) ,&gt_free_voxels, &gt_occ_voxels
    // );

    if (argc > 3) {  // output to file
        // std::ofstream file(argv[3], std::ios::app);

        // if (file.is_open()) {
        //     file << observed_region << "," << iou << "," << precision << "," << recall << std::endl;
        //     file.close();
        // }

        // else
        //     std::cerr << "Unable to open " << argv[3] << " file";
    }

    if (argc > 4 && std::string("publish").compare(argv[4]) == 0) {  // publish

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

        std::cout << "Publishing voxels comparison!" << std::endl;

        // publish voxels that are occupied in ground truth but are unoccupied and "observed" in observed map
        // pcl::PointCloud<pcl::PointXYZRGB> pointcloud_observed_diff;
        // createPointCloudFromVoxelIndices(difference_observed_gt, &pointcloud_observed_diff, Color::Red());
        // missed_occupancy_observed_voxels_pub.publish(pointcloud_observed_diff);

        // // publish voxels that are occupied in ground truth but are unoccupied and "not observed" in observed map
        // pcl::PointCloud<pcl::PointXYZRGB> pointcloud_un_observed_diff;
        // createPointCloudFromVoxelIndices(different_unobserved_gt, &pointcloud_un_observed_diff, Color::Orange());
        // missed_occupancy_un_observed_voxels_pub.publish(pointcloud_un_observed_diff);

        // pcl::PointCloud<pcl::PointXYZRGB> pointcloud_inter;
        // createPointCloudFromVoxelIndices(intersection_gt, &pointcloud_inter, Color::Green());
        // correct_occupied_voxels_observed_pub.publish(pointcloud_inter);

        // pcl::PointCloud<pcl::PointXYZRGB> pointcloud_observed_occupancy_fp;
        // createPointCloudFromVoxelIndices(difference_observed, &pointcloud_observed_occupancy_fp, Color::Yellow());
        // false_positive_observations_pub.publish(pointcloud_observed_occupancy_fp);

        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_free_gt_voxels;
        ssc_mapping::createPointCloudFromVoxelIndices(gt_free_voxels, &pointcloud_free_gt_voxels,
                                                      voxblox::Color::White());
        free_gt_voxels_pub.publish(pointcloud_free_gt_voxels);

        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_occ_gt_voxels;
        ssc_mapping::createPointCloudFromVoxelIndices(gt_occ_voxels, &pointcloud_occ_gt_voxels, voxblox::Color::Gray());
        occ_gt_voxels_pub.publish(pointcloud_occ_gt_voxels);
        // pcl::PointCloud<pcl::PointXYZRGBA> pointcloud_unobserved_free_voxels;
        // auto color = Color::Gray();
        // color.a = 80;
        // createPointCloudFromVoxelIndices(unobserved_unoccupied_voxels, &pointcloud_unobserved_free_voxels, color);
        // unobserved_free_voxels_pub.publish(pointcloud_unobserved_free_voxels);

        ros::spin();
    }

    return 0;
}
