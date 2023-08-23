#################################################################################
# File: lidar_utils.py
# Project: LiPeZ
# Creation Date: 08.02.2023
# Author: Nico Leuze,
#         Munich University of Applied Science (MUAS)
#         Faculty of Electrical Engineering and Information Technology
#         Institute for Applications of Machine Learning and Intelligent Systems
# -----
# Last Modified: XX.XX.202X
# Modified By: Nico Leuze
# -----
# Copyright (c) 2023 MUAS
#################################################################################

def generate_lidar_bp(arg, world, blueprint_library, delta):
    """Generates a CARLA blueprint based on the script parameters"""
    arg.semantic = True
    if arg.semantic:
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    else:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        if arg.no_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        else:
            lidar_bp.set_attribute('noise_stddev', '0.2')

    lidar_bp.set_attribute('upper_fov', str(arg.upper_fov))
    lidar_bp.set_attribute('lower_fov', str(arg.lower_fov))
    lidar_bp.set_attribute('channels', str(arg.channels))
    lidar_bp.set_attribute('range', str(arg.range))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(arg.points_per_second))
    return lidar_bp