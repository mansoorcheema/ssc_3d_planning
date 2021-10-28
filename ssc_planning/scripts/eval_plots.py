import subprocess
import os
import csv
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
import math
import argparse
import sys
from pathlib import Path

def create_figure_object(title, num_plots, width=16, height=10):
    cols = 3
    rows = math.ceil(num_plots/cols)
    fig, axs = plt.subplots(rows, cols)
    fig.set_figheight(height)
    fig.set_figwidth(width)
    num_extra_plots =  ((rows * cols) - num_plots)
    if num_extra_plots > 0:
        for ax in axs.flat[-num_extra_plots:]:
            fig.delaxes(ax)
    fig.suptitle(title, fontsize=16)
    return fig, axs
    
def read_plot_data_csv(csv_file):
    data = {}
    with open(csv_file, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        header = next(reader)
        for key in header:
            data[key] = []
            
        for row in reader:
            for i, r in enumerate(row):
                data[header[i]].append(float(r))
    return data

def plot_and_save_metrics(csv_file, plot_out_file, title):
    data = {}
    with open(csv_file, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        header = next(reader)
        for key in header:
            data[key] = []
            
        for row in reader:
            for i, r in enumerate(row):
                data[header[i]].append(float(r))
                
    # fig.suptitle(title)
    fig, axs = create_figure_object(title, len(header))
    for i, ax in enumerate(axs.flat):
        if i < len(header):
            ax.set_ylim([0, 1])
            ax.set_title(header[i])
            ax.plot((np.arange(len(data[header[i]])) * 30) / 60 , data[header[i]])
            ax.set(xlabel='Time in Minutes')
        

        
    #for ax in axs.flat:
    #    ax.set(xlabel='Time * 30 seconds', ylabel='amount')
    plt.tight_layout()
    plt.savefig(plot_out_file, bbox_inches='tight')

# plots quality and coverage graphs for maps in given directory
def evaluate(dirs, gt_path, map_type):
    
    eval_quality_csv_file = os.path.join(dirs, "metrics", "eval_quality.csv")
    eval_coverage_csv_file = os.path.join(
        dirs, "metrics", "eval_coverage.csv")  
    
    quality_plot_out_file = os.path.join(dirs, "metrics", "quality_plot.png")
    coverage_plot_out_file = os.path.join(
        dirs, "metrics", "coverage_plot.png")  

    # create log file
    with open(eval_quality_csv_file, 'w') as f:
        f.write("precision_occ,precision_free,precision_overall,recall_occ,recall_free,IoU_occ,IoU_free\n")
    
    with open(eval_coverage_csv_file, 'w') as f:
        f.write("explored_occ,explored_free,explored_overall,coverage_occ,coverage_free,coverage_overall\n")
        
    map_files = sorted(glob(os.path.join(dirs,"maps", "*."+map_type)), key=os.path.getctime)
    for voxblox_map in map_files:
       print(voxblox_map)

    # run evaluation
    for voxblox_map in map_files:
        subprocess.run(['rosrun', 'ssc_mapping', 'ssc_map_eval_node', gt_path, voxblox_map, eval_quality_csv_file, eval_coverage_csv_file, "0", "1"])

    plot_and_save_metrics(eval_quality_csv_file, quality_plot_out_file, 'Quality Metrics')
    plot_and_save_metrics(eval_coverage_csv_file, coverage_plot_out_file, 'Coverage Metrics')

# plots the emtrics to given axs subplots.
def plot_metrics(data, axs, label=""):
    headers = list(data.keys())
    for i, ax in enumerate(axs.flat):
        if i < len(headers):
            ax.set_ylim([0, 1])
            ax.set_title(headers[i])
            ax.plot((np.arange(len(data[headers[i]])) * 30) / 60 , data[headers[i]], label=label)
            ax.set(xlabel='Time in Minutes')
            ax.legend()
    

# calculates evaluation metrics and save the results to file
def save_metrics_csv(eval_quality_csv_file, eval_coverage_csv_file, map_files, gt_path):

    # create log file
    with open(eval_quality_csv_file, 'w') as f:
        f.write("precision_occ,precision_free,precision_overall,recall_occ,recall_free,IoU_occ,IoU_free\n");
    
    with open(eval_coverage_csv_file, 'w') as f:
        f.write("explored_occ,explored_free,explored_overall,coverage_occ,coverage_free,coverage_overall\n");
        
    for voxblox_map in map_files:
       print(voxblox_map)

    # run evaluation
    for voxblox_map in map_files:
        subprocess.run(['rosrun', 'ssc_mapping', 'ssc_map_eval_node', gt_path, voxblox_map, eval_quality_csv_file, eval_coverage_csv_file, "0", "1"])

def evaluate_comp(data_folder, gt_path=None, map_type=None):
    quality_metrics_fig, quality_metrics_axs = create_figure_object("Quality Metrics {}".format(map_type.upper()), 7)
    coverage_metrics_fig, coverage_metrics_axs  = create_figure_object("Coverage Metrics {}".format(map_type.upper()), 6)

    quality_plot_out_file = Path(data_folder) / "plots" /  "quality_plot_{}.png".format(map_type)
    coverage_plot_out_file = Path(data_folder) / "plots" / "coverage_plot_{}.png".format(map_type)
    quality_plot_out_file.parent.mkdir(parents=True, exist_ok=True) 
    coverage_plot_out_file.parent.mkdir(parents=True, exist_ok=True) 

    dirs = os.listdir(data_folder)
    for d in dirs:
        curr_data_dir = os.path.join(data_folder, d)
        if not os.path.isdir(curr_data_dir):
            continue

        if "maps" not in d:
            continue

        print("Calculating plots for {}".format(d))
        
        eval_quality_csv_file = Path(curr_data_dir) / "metrics_{}".format(map_type)  /  "eval_quality.csv"
        eval_coverage_csv_file = Path(curr_data_dir)/ "metrics_{}".format(map_type) / "eval_coverage.csv"
        eval_quality_csv_file.parent.mkdir(parents=True, exist_ok=True)  
        eval_coverage_csv_file.parent.mkdir(parents=True, exist_ok=True)  

        if gt_path is not None: # ground truth is provided so calculate evaluations ans save to csv
            # create empty csv files
            map_files = sorted(glob(os.path.join(curr_data_dir,"**","maps", "*."+map_type), recursive=True), key=os.path.getctime)
            save_metrics_csv(eval_quality_csv_file, eval_coverage_csv_file, map_files, gt_path)
        else:
            print("Ground truth map path is not provided, assuming csv files are already generated!")

        # now that we have data saved in csv's at respective paths , lets draw plots
        #plot_and_save_metrics(None, quality_metrics_fig[1], ""):
        label = d[:d.find("_maps")]
        plot_metrics(read_plot_data_csv(eval_quality_csv_file),quality_metrics_axs, label=label)
        plot_metrics(read_plot_data_csv(eval_coverage_csv_file),coverage_metrics_axs, label=label)
    
    #save quality metric
    quality_metrics_fig.tight_layout()
    quality_metrics_fig.savefig(quality_plot_out_file, bbox_inches='tight') 
    
    #save coverage metric
    coverage_metrics_fig.tight_layout()
    coverage_metrics_fig.savefig(coverage_plot_out_file, bbox_inches='tight')   

    
if __name__ == '__main__':
    if len(sys.argv) <4:
        print("usage: eval_plots.py <voxblox_tsdf_maps> <gt_tsdf_map> <map_type>")
        exit(-1)
        
    #evaluate(sys.argv[1], sys.argv[2], sys.argv[3])
    evaluate_comp(sys.argv[1],sys.argv[2], sys.argv[3])

