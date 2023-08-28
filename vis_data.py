#################################################################################
# File: vis_data.py
# Project: LiPeZ
# Creation Date: 20.04.2023
# Author: Nico Leuze,
#         Munich University of Applied Science (MUAS)
#         Faculty of Electrical Engineering and Information Technology
#         Institute for Applications of Machine Learning and Intelligent Systems
# -----
# Last Modified: 23.08.2023
# Modified By: Nico Leuze
# -----
# Copyright (c) 2023 MUAS
#################################################################################

import argparse
import time, os
from _ast import arg
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import open3d as o3d


# Coloring Settings:
VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses


def main(args):
    # Set the Loading Paths:
    pc_path = args.data_dir + "/sequence{:04n}".format(args.sample_seq) + "/pointcloud/{}.npy".format(args.sample)
    lbl_path = args.data_dir + "/sequence{:04n}".format(args.sample_seq) + "/labels/{}.npy".format(args.sample)
    
    # Load Point Cloud and Labels:
    point_cloud = np.load(pc_path)
    gt_labels = np.load(lbl_path)

    # Extract Labels:
    labels = gt_labels[0]
    instances = gt_labels[1]

    # Set Colors for labels:
    color_lbl = np.zeros((labels.shape[0], 3))
    for idx, value in enumerate(labels):
        color_lbl[idx, :] = LABEL_COLORS[value]

    # Set Colors for instances:
    color_ins = np.zeros((instances.shape[0], 3))
    unq = np.unique(instances)
    instances_unq = np.zeros((instances.shape[0]))
    
    # Make Instances hashable for Color Map:
    for idx, value in enumerate(unq):
        instances_unq[instances == value] = idx
    for idx, value in enumerate(instances_unq.astype(np.int)):
        color_ins[idx, :] = LABEL_COLORS[value]

    # Initialize Open3D PointCloud:
    pcd = o3d.geometry.PointCloud()
    # Set Points of Point Cloud
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    # Visualize:
    o3d.visualization.draw_geometries([pcd])

    # Set Colors of PointCloud - Labels:
    pcd.colors = o3d.utility.Vector3dVector(color_lbl)
    # Visualize:
    o3d.visualization.draw_geometries([pcd])
    
    # Set Colors of PointCloud - Instances:
    pcd.colors = o3d.utility.Vector3dVector(color_ins)
    # Visualize:
    o3d.visualization.draw_geometries([pcd])



if __name__ =='__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('--data_dir', type=str, default=None, help='specify dataset directory')
    parser.add_argument('--sample_seq', type=int, default=None, help='specify sequence from the dataset to inspect; e.g., 14')
    parser.add_argument('--sample', type=int, default=None, help='specify sample file from the dataset to inspect; e.g., 11')
    parser.add_argument('--colorize_method', type=str, default=None, help='specify wheter to colorize Label Tag or Instances')

    args = parser.parse_args()

    main(args)