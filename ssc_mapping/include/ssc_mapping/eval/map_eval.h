
#ifndef SSC_MAPPING_EVAL_H_
#define SSC_MAPPING_EVAL_H_

// general includes
#include <assert.h>
#include <glog/logging.h>

#include <fstream>
#include <iostream>

// ros/pcl includes
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

// voxblox includes
#include <ssc_mapping/visualization/visualization.h>
#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/voxel.h>
#include <voxblox/io/layer_io.h>

using namespace voxblox;

bool isObserved(const voxblox::TsdfVoxel& voxel, float voxel_size) {
    return voxblox::visualizeOccupiedTsdfVoxels(voxel, voxblox::Point(), voxel_size);
}

bool isObserved(const voxblox::SSCOccupancyVoxel& voxel, float voxel_size) {
    voxblox::Color color;
    return voxblox::visualizeSSCOccupancyVoxels(voxel, voxblox::Point(), &color);
}

// finds which occupied voxels in layer are also occupied in other layer. The intersecting occupied 
// voxels in otherLayer are added to intersection and the unobserved and "OBSERVED" voxels > voxel_size
// are added to difference.
template <typename VoxelTypeA, typename VoxelTypeB>
void calculate_Intersection_difference(const voxblox::Layer<VoxelTypeA>& layer,
                                       const voxblox::Layer<VoxelTypeB>& otherLayer, GlobalIndexVector* intersection,
                                       GlobalIndexVector* difference) {
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);
    for (const voxblox::BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const voxblox::Block<voxblox::TsdfVoxel>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            // voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            auto gt_voxel = block.getVoxelByLinearIndex(linear_index);

            if (isObserved(gt_voxel, layer.voxel_size())) {
                // voxel is observed in first layer. check if it exists in other layer

                // get global voxel index
                voxblox::VoxelIndex voxel_idx = block.computeVoxelIndexFromLinearIndex(linear_index);
                voxblox::BlockIndex block_idx = index;
                voxblox::GlobalIndex global_voxel_idx =
                    voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_idx, voxel_idx, layer.voxels_per_side());

                // see if this voxel is observed in other layer
                auto observed_voxel = otherLayer.getVoxelPtrByGlobalIndex(global_voxel_idx);

                if (observed_voxel != nullptr && isObserved(*observed_voxel, otherLayer.voxel_size())) {
                    intersection->emplace_back(global_voxel_idx);
                } else {
                    difference->emplace_back(global_voxel_idx);
                }
            }
        }
    }
}

// Creates a pointcloud from voxel indices
void createPointCloudFromVoxelIndices(const voxblox::GlobalIndexVector& voxels,
                                      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud, const voxblox::Color& color, const float voxel_size = 0.08) {
    for (auto voxel : voxels) {
        pcl::PointXYZRGB point;
        point.x = voxel.x() * voxel_size + (voxel_size/2);
        point.y = voxel.y() * voxel_size + (voxel_size/2);
        point.z = voxel.z() * voxel_size + (voxel_size/2);
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;
        pointcloud->push_back(point);
    }
    pointcloud->header.frame_id = "world";
}

// Creates a pointcloud from voxel indices
void createPointCloudFromVoxelIndices(const voxblox::GlobalIndexVector& voxels,
                                      pcl::PointCloud<pcl::PointXYZRGBA>* pointcloud, const voxblox::Color& color, const float voxel_size = 0.08) {
    for (auto voxel : voxels) {
        pcl::PointXYZRGBA point;
        point.x = voxel.x() * voxel_size + (voxel_size/2);
        point.y = voxel.y() * voxel_size + (voxel_size/2);
        point.z = voxel.z() * voxel_size + (voxel_size/2);
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;
        point.a = color.a;
        pointcloud->push_back(point);
    }
    pointcloud->header.frame_id = "world";
}

// finds which observed unoccupied voxels in layer are unobserved in othe layer.
template <typename VoxelTypeA, typename VoxelTypeB>
void find_unobserved_free_voxels(const voxblox::Layer<VoxelTypeA>& layer,
                                                 const voxblox::Layer<VoxelTypeB>& otherLayer,
                                                 GlobalIndexVector* out_voxels) {
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);

    for (const voxblox::BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const voxblox::Block<voxblox::TsdfVoxel>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            // voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            auto voxel = block.getVoxelByLinearIndex(linear_index);

            if (voxel.weight < 1e-6) {// voxel is unobserved in layer
                // check if its free or un observed in other layer
                voxblox::VoxelIndex voxel_idx = block.computeVoxelIndexFromLinearIndex(linear_index);
                voxblox::BlockIndex block_idx = index;
                voxblox::GlobalIndex global_voxel_idx =
                    voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_idx, voxel_idx, layer.voxels_per_side());

                    auto observed_voxel = otherLayer.getVoxelPtrByGlobalIndex(global_voxel_idx);

                    if (observed_voxel == nullptr) {
                        out_voxels->emplace_back(global_voxel_idx);
                    } else if(observed_voxel->weight < 1e-6) {
                        out_voxels->emplace_back(global_voxel_idx);
                    }
            }
        }
    }
}

template <typename VoxelType>
void calculateFreeObservedVoxels(const voxblox::Layer<VoxelType>& layer, GlobalIndexVector* voxels) {
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);
    for (const voxblox::BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const voxblox::Block<voxblox::TsdfVoxel>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            // voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            auto voxel = block.getVoxelByLinearIndex(linear_index);

            if(voxel.weight > 1e-6 && voxel.distance < layer.voxel_size() && voxel.distance > -layer.voxel_size() ) {
                voxblox::VoxelIndex voxel_idx = block.computeVoxelIndexFromLinearIndex(linear_index);
                voxblox::BlockIndex block_idx = index;
                voxblox::GlobalIndex global_voxel_idx =
                    voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_idx, voxel_idx, layer.voxels_per_side());
                voxels->emplace_back(global_voxel_idx);
            }
        }
    }
}


// Creates a list of obeserved and unobserved voxels (in the input layer) from a list of voxels.
// The results are filled in out_observed_voxels and out_UnObserved_voxels.
template <typename VoxelType>
void splitObservedAndUnObservedVoxels(const voxblox::Layer<VoxelType>& layer, const GlobalIndexVector& in_voxels, GlobalIndexVector* out_observed_voxels,  GlobalIndexVector* out_un_observed_voxels ) {

    for (auto voxel_idx : in_voxels) {
        auto voxel = layer.getVoxelPtrByGlobalIndex(voxel_idx);
        
        if (voxel == nullptr) {
            // voxel does not exist, so its un observed
            out_un_observed_voxels->emplace_back(voxel_idx);
        } else if (voxel->weight > 1e-6 ) {
            // voxel exists and is observed
            out_observed_voxels->emplace_back(voxel_idx);
        } else {
            // voxel exists, but is not observed
             out_un_observed_voxels->emplace_back(voxel_idx);
        }
    }
}

// unit_test
void test_eval_metrics() {
    voxblox::TsdfMap::Ptr ground_truth_map;
    voxblox::TsdfMap::Ptr observed_map;
    voxblox::TsdfMap::Config config;
    config.tsdf_voxel_size = 1;
    config.tsdf_voxels_per_side = 8u;

    ground_truth_map = std::make_shared<voxblox::TsdfMap>(config);
    observed_map = std::make_shared<voxblox::TsdfMap>(config);

    {
        // add a block at origin
        voxblox::Point point_in_0_0_0(0, 0, 0);
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block_0_0_0 =
            ground_truth_map->getTsdfLayerPtr()->allocateNewBlockByCoordinates(point_in_0_0_0);
        observed_map->getTsdfLayerPtr()->allocateNewBlockByCoordinates(point_in_0_0_0);

        // add another block
        voxblox::Point point_in_10_0_0(ground_truth_map->block_size(), 0, 0);
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block_1_0_0 =
            ground_truth_map->getTsdfLayerPtr()->allocateNewBlockByCoordinates(point_in_10_0_0);

        {
            int gt_x_min = 4;
            int gt_x_max = 12;
            int gt_y_min = 3;
            int gt_y_max = 6;

            for (size_t x = gt_x_min; x < gt_x_max; x++) {
                for (size_t y = gt_y_min; y < gt_y_max; y++) {
                    // global voxel by global index
                    // auto voxel =
                    voxblox::GlobalIndex voxelIdx(x, y, 0);

                    auto voxel = ground_truth_map->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);
                    voxel->weight = 1;
                }
            }

            int observed_x_min = 5;
            int observed_x_max = 8;
            int observed_y_min = 2;
            int observed_y_max = 4;

            // add voxels to observed map
            for (size_t x = observed_x_min; x < observed_x_max; x++) {
                for (size_t y = observed_y_min; y < observed_y_max; y++) {
                    // global voxel by global index
                    // auto voxel =
                    voxblox::GlobalIndex voxelIdx(x, y, 0);

                    auto voxel = observed_map->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(voxelIdx);
                    voxel->weight = 1;
                }
            }
        }
    }

    voxblox::GlobalIndexVector intersection_gt, difference_gt;
    calculate_Intersection_difference(ground_truth_map->getTsdfLayer(), observed_map->getTsdfLayer(), &intersection_gt,
                                      &difference_gt);

    CHECK(intersection_gt.size() == 3) << "Error in Intersection Algorithm! Recheck Implementation.";
    CHECK(difference_gt.size() == 21) << "Error in DIfference Algorithm! Recheck Implementation.";
}

#endif //SSC_MAPPING_EVAL_H_
