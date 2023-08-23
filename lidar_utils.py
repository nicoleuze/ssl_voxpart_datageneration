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


"""
LiDAR Configurations
"""
lidar_configs = {
    'channels':             32,
    'range':              10.0,
    'points_per_second': 56000,
    'rotation_frequency': 10.0,
    'upper_fov':          10.0,
    'lower_fov':         -30.0,
    'horizontal_fov':    70.0,
    'sensor_tick':         0.0
}

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

    lidar_bp.set_attribute('upper_fov', str(lidar_configs['upper_fov']))
    lidar_bp.set_attribute('lower_fov', str(lidar_configs['lower_fov']))
    lidar_bp.set_attribute('channels', str(lidar_configs['channels']))
    lidar_bp.set_attribute('range', str(lidar_configs['range']))
    lidar_bp.set_attribute('rotation_frequency', str(lidar_configs['rotation_frequency']))
    lidar_bp.set_attribute('points_per_second', str(lidar_configs['points_per_second']))
    return lidar_bp