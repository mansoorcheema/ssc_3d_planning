import os
import pathlib
import glob
from pathlib import Path
import sys
import subprocess

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python create_merged_maps.py /Data/simulations/Data/ExperimentConfiguration/mapping_evaluations/run2/experiment1_baseline_maps")
        exit(-1)
        
    main_dir = sys.argv[1]
    for run_dir in os.listdir(main_dir):
        if '2021' not  in run_dir:
            continue

        experiment_dir = Path(main_dir) / run_dir
        tsdf_map_files = sorted(glob.glob(os.path.join(experiment_dir,"voxblox_maps","maps","*.tsdf")), key=os.path.getctime)
        
        if not experiment_dir.exists():
            print ("Error! Path {} does not exists!".format(experiment_dir))
            exit(-1)
        
        for f in tsdf_map_files:
            tsdf_map_file = Path(f)
            relevant_ssc_file = experiment_dir / "ssc_maps"/ "maps" /tsdf_map_file.name
            relevant_ssc_file = relevant_ssc_file.with_suffix(".ssc")
            target_merged_layer_file = Path(experiment_dir) / "mssc_maps"/ "maps" /tsdf_map_file.name
            target_merged_layer_file = target_merged_layer_file.with_suffix(".mssc")
            target_merged_layer_file.parent.mkdir(parents=True, exist_ok=True) 
            
            # rosrun
            subprocess.run(['rosrun', 'ssc_mapping', 'merge_measured_predicted_layers_node', tsdf_map_file, relevant_ssc_file, target_merged_layer_file])
