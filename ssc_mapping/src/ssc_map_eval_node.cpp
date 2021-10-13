
#include "ssc_mapping/eval/map_eval.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ssc_mapping_eval");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, false);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    test_eval_metrics();

    voxblox::Layer<voxblox::TsdfVoxel>::Ptr ground_truth_layer;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr observed_layer;

    voxblox::TsdfMap::Ptr ground_truth_map;
    voxblox::TsdfMap::Ptr observed_map;

    // load ground truth and observed map from file

    if (argc < 3) {
        std::cout << "Usage: rosrun ssc_map_eval_node <gt_layer> <observed_layer> <optional_output> "
                     "<optional_publish_stats>";
        return -1;
    }

    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(argv[1], &ground_truth_layer);
    voxblox::io::LoadLayer<voxblox::TsdfVoxel>(argv[2], &observed_layer);

    ground_truth_map = std::make_shared<voxblox::TsdfMap>(ground_truth_layer);
    observed_map = std::make_shared<voxblox::TsdfMap>(observed_layer);

    CHECK(ground_truth_map->getTsdfLayerPtr()->voxel_size() == observed_map->getTsdfLayerPtr()->voxel_size())
        << "Error! Observed Layer and groundtruth layers should have same voxel size!";

    voxblox::GlobalIndexVector intersection_gt, difference_gt;
    calculate_Intersection_difference(ground_truth_map->getTsdfLayer(), observed_map->getTsdfLayer(), &intersection_gt,
                                      &difference_gt);

    voxblox::GlobalIndexVector intersection_observed, difference_observed;
    calculate_Intersection_difference(observed_map->getTsdfLayer(), ground_truth_map->getTsdfLayer(),
                                      &intersection_observed, &difference_observed);

    voxblox::GlobalIndexVector difference_observed_gt, different_unobserved_gt;// Note:here
    // observed and un observed mean in the voxblox generated observed map,  not in gt
    splitObservedAndUnObservedVoxels(observed_map->getTsdfLayer(), difference_gt, &difference_observed_gt, &different_unobserved_gt);
                                      
    voxblox::GlobalIndexVector free_gt_voxels;
    calculateFreeObservedVoxels(ground_truth_map->getTsdfLayer(), &free_gt_voxels);

    //voxels that are not observed in observed map but are free space in
    // ground truth map. In ground truth a free space is either empty voxel or has weight < 1e-6. Thats not the case for observed
    // layer. Its because voxblox_ground_truth does not fill free space with voxels. There is option to fill but that produces 
    // false occupied voxels.
    voxblox::GlobalIndexVector unobserved_unoccupied_voxels;
    find_unobserved_free_voxels(observed_map->getTsdfLayer(), ground_truth_map->getTsdfLayer(), &unobserved_unoccupied_voxels);
    

    // compute evaluation metrics
    size_t gt_occupied_voxels = 0;
    size_t observed_voxels = 0;
    float precision = 0.0f;
    float recall = 0.0f;
    float iou = 0;
    float observed_region = 0;

    observed_voxels = intersection_observed.size() + difference_observed.size();
    gt_occupied_voxels = difference_gt.size() + intersection_gt.size();
    iou = intersection_gt.size() / float(intersection_gt.size() + difference_gt.size() + difference_observed.size());
    recall = intersection_observed.size() / float(intersection_gt.size() + difference_gt.size());
    precision = intersection_observed.size() / float(intersection_observed.size() + difference_observed.size());
    observed_region = (observed_voxels / float(gt_occupied_voxels));

    // print eval statis
    printf("---------- Evaluation -----------\n");
    printf("iou: %0.2lf \n", iou);
    printf("precision: %0.2lf \n", precision);
    printf("recall: %0.2lf \n", recall);
    printf("observed: %0.2lf\n", observed_region);
    // printf("gt_occupied_voxels: %d\n", gt_occupied_voxels);
    // printf("observed_voxels: %d\n",observed_voxels);
    printf("---------------------------------\n");

    if (argc > 3) {  // output to file
        std::ofstream file(argv[3], std::ios::app);

        if (file.is_open()) {
            file << observed_region << "," << iou << "," << precision << "," << recall << std::endl;
            file.close();
        }

        else
            std::cerr << "Unable to open " << argv[3] << " file";
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
        auto free_gt_voxels_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("free_gt_voxels", 1, true);
        auto unobserved_free_voxels_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("unobserved_free_voxels", 1, true);

        std::cout << "Publishing voxels comparison!" << std::endl;

        // publish voxels that are occupied in ground truth but are unoccupied and "observed" in observed map
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_observed_diff;
        createPointCloudFromVoxelIndices(difference_observed_gt, &pointcloud_observed_diff, Color::Red());
        missed_occupancy_observed_voxels_pub.publish(pointcloud_observed_diff);

        // publish voxels that are occupied in ground truth but are unoccupied and "not observed" in observed map
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_un_observed_diff;
        createPointCloudFromVoxelIndices(different_unobserved_gt, &pointcloud_un_observed_diff, Color::Orange());
        missed_occupancy_un_observed_voxels_pub.publish(pointcloud_un_observed_diff);

        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_inter;
        createPointCloudFromVoxelIndices(intersection_gt, &pointcloud_inter, Color::Green());
        correct_occupied_voxels_observed_pub.publish(pointcloud_inter);

        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_observed_occupancy_fp;
        createPointCloudFromVoxelIndices(difference_observed, &pointcloud_observed_occupancy_fp, Color::Yellow());
        false_positive_observations_pub.publish(pointcloud_observed_occupancy_fp);
        
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_free_gt_voxels;
        createPointCloudFromVoxelIndices(free_gt_voxels, &pointcloud_free_gt_voxels, Color::Gray());
        free_gt_voxels_pub.publish(pointcloud_free_gt_voxels);
        // pcl::PointCloud<pcl::PointXYZRGBA> pointcloud_unobserved_free_voxels;
        // auto color = Color::Gray();
        // color.a = 80;
        // createPointCloudFromVoxelIndices(unobserved_unoccupied_voxels, &pointcloud_unobserved_free_voxels, color);
        // unobserved_free_voxels_pub.publish(pointcloud_unobserved_free_voxels);

        ros::spin();
    }

    return 0;
}
