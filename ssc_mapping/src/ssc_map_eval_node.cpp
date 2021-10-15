
#include "ssc_mapping/eval/map_eval.h"



void refine_observed_layer(const voxblox::Layer<voxblox::TsdfVoxel>& gt_layer,
                           voxblox::Layer<voxblox::TsdfVoxel>::Ptr observed_layer) {

    

    CHECK(gt_layer.voxel_size() == observed_layer->voxel_size()) << "Layers should have same voxel size.";
    CHECK(gt_layer.voxels_per_side() == observed_layer->voxels_per_side()) << "Layers should have same block size.";
    
    size_t vps = gt_layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    observed_layer->getAllAllocatedBlocks(&blocks);
    for (const voxblox::BlockIndex& block_idx : blocks) {
        auto block = observed_layer->getBlockPtrByIndex(block_idx);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            // voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            voxblox::TsdfVoxel* voxel = &block->getVoxelByLinearIndex(linear_index);
            voxblox::VoxelIndex voxel_idx = block->computeVoxelIndexFromLinearIndex(linear_index);
                voxblox::GlobalIndex global_voxel_idx =
                    voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_idx, voxel_idx, gt_layer.voxels_per_side());

                // see if this voxel is observed in other layer
                auto gt_voxel = gt_layer.getVoxelPtrByGlobalIndex(global_voxel_idx);

                if(!gt_voxel) {
                    voxel->weight = 0.0f;//voxel not observed in gt_map so ignore voxels that are not observed in gt map
                }
        }
    }
}

void get_free_and_occupied_voxels_from_layer(const voxblox::Layer<voxblox::TsdfVoxel>& layer,
                                             GlobalIndexVector* occ_voxels, GlobalIndexVector* free_voxels) {

    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);
    for (const voxblox::BlockIndex& index : blocks) {
        const voxblox::Block<voxblox::TsdfVoxel>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            // voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            auto voxel = block.getVoxelByLinearIndex(linear_index);
            VoxelIndex voxel_idx = block.computeVoxelIndexFromLinearIndex(linear_index);
            BlockIndex block_idx = index;
            GlobalIndex global_voxel_idx =
                voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_idx, voxel_idx, layer.voxels_per_side());

            if (voxel.weight > 1e-3) {
                bool is_occupied = voxel.distance <= layer.voxel_size();

                if (is_occupied) {
                    occ_voxels->emplace_back(global_voxel_idx);
                } else {
                    free_voxels->emplace_back(global_voxel_idx);
                }
            }
        }
    }
}

void refine_gt_layer(const voxblox::Layer<voxblox::TsdfVoxel>& gt_layer,
                     voxblox::Layer<voxblox::TsdfVoxel>::Ptr refined_layer) {
    voxblox::GlobalIndexVector occ_gt_voxels, free_gt_voxels;
    computeFrontierCandidates(gt_layer, Point(0.0, 0.0, 0.0), &free_gt_voxels, &occ_gt_voxels);

    // add these voxels to refined layer
    for (auto global_voxel_idx : occ_gt_voxels) {
        // BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
        // refined_layer->voxels_per_side_inv()); VoxelIndex local_voxel_idx =
        // getLocalFromGlobalVoxelIndex(global_voxel_idx, refined_layer->voxels_per_side_inv());

        // auto block = refined_layer->allocateBlockPtrByIndex(block_idx);

        // auto occ_voxel = block->getVoxelByVoxelIndex(local_voxel_idx);

        voxblox::TsdfVoxel* occ_voxel = refined_layer->getVoxelPtrByGlobalIndex(global_voxel_idx);

        // check if the block containing the voxel exists.
        if (occ_voxel == nullptr) {
            // ssc_map_->getSSCLayerPtr()->a
            BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, refined_layer->voxels_per_side_inv());
            auto block = refined_layer->allocateBlockPtrByIndex(block_idx);
            const VoxelIndex local_voxel_idx =
                getLocalFromGlobalVoxelIndex(global_voxel_idx, refined_layer->voxels_per_side());
            occ_voxel = &block->getVoxelByVoxelIndex(local_voxel_idx);
        }

        occ_voxel->weight = 1.0;
        occ_voxel->distance = 0.0f;
    }

    for (auto global_voxel_idx : free_gt_voxels) {
       voxblox::TsdfVoxel* free_voxel = refined_layer->getVoxelPtrByGlobalIndex(global_voxel_idx);

        // check if the block containing the voxel exists.
        if (free_voxel == nullptr) {
            // ssc_map_->getSSCLayerPtr()->a
            BlockIndex block_idx = getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, refined_layer->voxels_per_side_inv());
            auto block = refined_layer->allocateBlockPtrByIndex(block_idx);
            const VoxelIndex local_voxel_idx =
                getLocalFromGlobalVoxelIndex(global_voxel_idx, refined_layer->voxels_per_side());
            free_voxel = &block->getVoxelByVoxelIndex(local_voxel_idx);
        }
        free_voxel->weight = 1.0f;
        free_voxel->distance = 100.0f;//refined_layer->voxel_size();
    }
}

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

    // create a refined gt layer that only includes traversable regions
    
    
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr ground_truth_layer_refined;
    ground_truth_layer_refined = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(ground_truth_layer->voxel_size(), ground_truth_layer->voxels_per_side());

    refine_gt_layer(*ground_truth_layer, ground_truth_layer_refined);
    //refine_observed_layer(*ground_truth_layer, observed_layer);

    voxblox::GlobalIndexVector occ_gt_voxels, free_gt_voxels;
    get_free_and_occupied_voxels_from_layer(*ground_truth_layer, &occ_gt_voxels, &free_gt_voxels );
    
    voxblox::GlobalIndexVector intersection_gt, difference_gt;
    calculate_Intersection_difference(*ground_truth_layer, *observed_layer, &intersection_gt,
                                      &difference_gt);

    voxblox::GlobalIndexVector intersection_observed, difference_observed;
    calculate_Intersection_difference(*observed_layer, *ground_truth_layer,
                                      &intersection_observed, &difference_observed);
    //prune irrelevant voxels
    // pruneInsideVoxels(*observed_layer, &difference_observed);//remove voxels inside walls
    // pruneOutlierVoxels(*ground_truth_layer,&difference_observed); // remvoe voxels that lie outside gt bound

    voxblox::GlobalIndexVector difference_observed_gt, different_unobserved_gt;// Note:here
    // observed and un observed mean in the voxblox generated observed map,  not in gt
    splitObservedAndUnObservedVoxels(*observed_layer, difference_gt, &difference_observed_gt, &different_unobserved_gt);
                                      
    //voxblox::GlobalIndexVector free_gt_voxels;
    //calculateFreeObservedVoxels(ground_truth_map->getTsdfLayer(), &free_gt_voxels);

    //voxels that are not observed in observed map but are free space in
    // ground truth map. In ground truth a free space is either empty voxel or has weight < 1e-6. Thats not the case for observed
    // layer. Its because voxblox_ground_truth does not fill free space with voxels. There is option to fill but that produces 
    // false occupied voxels.
    // voxblox::GlobalIndexVector unobserved_unoccupied_voxels;
    // find_unobserved_free_voxels(*ground_truth_layer, *ground_truth_layer, &unobserved_unoccupied_voxels);
    
    // voxblox::GlobalIndexVector occ_gt_voxels, free_gt_voxels;
    // computeFrontierCandidates(ground_truth_map->getTsdfLayer(), Point(0.0,0.0,0.0) ,&free_gt_voxels, &occ_gt_voxels );

    // compute evaluation metrics
    size_t gt_occupied_voxels = 0;
    size_t observed_voxels = 0;
    float precision = 0.0f;
    float recall = 0.0f;
    float iou = 0;
    float observed_region = 0;

    observed_voxels = intersection_observed.size() + difference_observed.size();
    gt_occupied_voxels = difference_gt.size() + intersection_gt.size();
    iou = intersection_gt.size() / float(intersection_gt.size() + difference_observed_gt.size() + difference_observed.size());
    recall = intersection_observed.size() / float(intersection_gt.size() + difference_observed_gt.size());
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
        auto occ_gt_voxels_pub =
            nh_private.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("occ_gt_voxels", 1, true);
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
        
        // pcl::PointCloud<pcl::PointXYZRGB> pointcloud_free_gt_voxels;
        // // auto color = Color::White();
        // // color.a = 50;
        // createPointCloudFromVoxelIndices(free_gt_voxels, &pointcloud_free_gt_voxels, Color::White());
        // free_gt_voxels_pub.publish(pointcloud_free_gt_voxels);

        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_occ_gt_voxels;
        createPointCloudFromVoxelIndices(occ_gt_voxels, &pointcloud_occ_gt_voxels, Color::Gray());
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
