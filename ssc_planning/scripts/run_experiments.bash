#!/bin/bash
#for i in {1..8}; do
for planner_config_file in $(pwd)/planner_configs/*.yaml; do
     #filename=
     experiment_name=$(basename "${planner_config_file%.*}")
     target_output_dara_dir=$(pwd)/results/$experiment_name
     mkdir -p $target_output_dara_dir
     #echo $target_output_dara_dir
     # launch experiment
     roslaunch ssc_planning launch_ssc_pipeline.launch planner_config_file:=$planner_config_file eval_directory:=$target_output_dara_dir
     sleep 30
done
#    sleep 100
#done                                                                                      
