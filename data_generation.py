#################################################################################
# File: data_generation.py
# Project: LiPeZ
# Creation Date: 12.04.2023
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

import argparse
import time, os
from _ast import arg
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import open3d as o3d
import blickfeld_utils
import carla
import matplotlib.pyplot as plt

"""
GENERATION OF HALF SPHERE
"""
def gen_sphere(r, N):
    theta = 2 * np.pi * np.random.rand(N)
    # full sphere w/o conditions: 
    # phi = np.arccos(1 - 2 * np.random.rand(N))
    # hemisphere w/o conditions:
    # phi = np.arccos(1 - np.random.rand(N))

    # hemisphere with condition 22,5° < phi < 67,5° (Reminder: 22,5° = (np.pi/2 * 0.25) * 180/np.pi):
    phi = np.sin(np.pi / 2 * np.random.uniform(low=0.5, high=0.75, size=N))

    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.sin(phi) * np.sin(theta)
    # Hint: phi = 0 lies on z-axis >> 90° from ground plane - thus phi = 22,5° corresponds to 67,5° from ground plane
    # workaround could be to use z = r * np.sin(phi) which seems more obvious
    z = r * np.cos(phi)
    return x, y, z

"""
TRANSFORMATION CALCULATION FOR LIDAR AND SPECTATOR
"""
def center_sensor(sphere_translation, sphere_radius, center_spawnfield):
    transformation = carla.Transform()
    transformation.location.x = sphere_translation[0] + center_spawnfield[0]
    transformation.location.y = sphere_translation[1] + center_spawnfield[1]
    transformation.location.z = sphere_translation[2] + center_spawnfield[2]
    transformation.rotation.pitch = -np.arcsin(sphere_translation[2]/sphere_radius) * 180/np.pi
    transformation.rotation.roll = 0
    transformation.rotation.yaw = (np.arctan2(sphere_translation[1], sphere_translation[0]) * 180/np.pi) + 180
    return transformation


"""
MEMORY READER SENSOR DATA
"""
def raw_pointcloud2array(pc_data):
    """
    params in: pc_data
    params out: cloud (x, y, z, intensity), cloud_parameters (point_id, object_idx, object_tag)
    """

    # Extract PointCloud Information:
    cloudFromBuffer = np.frombuffer(pc_data, dtype=np.float32, count=(np.int(np.shape(pc_data)[0]/4)))
    x = cloudFromBuffer[0::7]
    y = cloudFromBuffer[1::7]
    z = cloudFromBuffer[2::7]
    int = cloudFromBuffer[3::7]

    # Extract PointCloud Parameters Information:
    cloudparametersFromBuffer = np.frombuffer(pc_data, dtype=np.int32, count=(np.int(np.shape(pc_data)[0] / 4)))
    point_id = cloudparametersFromBuffer[4::7]
    object_idx = cloudparametersFromBuffer[5::7]
    object_tag = cloudparametersFromBuffer[6::7]
    # invert pointcloud_about_y_axis:
    # point_cloud_array mirrored point cloud about y-axis: lhs to rhs conversion
    y = y * -1
    # Stack arrays to PointCloud and PointCloud Parameters:
    cloud = np.stack((x, y, z, int)).T.reshape((-1, 4))
    cloud_parameters = np.stack((point_id, object_idx, object_tag)).T.reshape((-1, 3))
    return cloud, cloud_parameters



"""
LIDAR CALLBACK 
"""
def lidar_callback(data):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    point_cloud_bf = data.raw_data

    # params: cloud (x, y, z, intensity), cloud_parameters (point_id, object_idx, object_tag)
    point_cloud_bf, cloud_parameters = raw_pointcloud2array(point_cloud_bf)

    # Isolate the 3D data
    points = point_cloud_bf[:, :3]
    intensity = point_cloud_bf[:, 3]

    # Isolate the Labels and Instances from the Cloud Parameters:
    labels = cloud_parameters[:, 2]
    instances = cloud_parameters[:, 1]

    # Saving the Labels and Instances to np.array(n, 2) with idx0 = labels, idx1 = instances:
    # REMINDER: gt_labels_inst[0] = labels, gt_labels_inst[1] = instances
    gt_labels_inst = np.vstack((labels, instances))

    # Save Data to Destiny Location:
    points_dir = store_dir_pc + data_index
    np.save(points_dir, points)

    intensity_dir = store_dir_pcint + data_index
    np.save(intensity_dir, intensity)

    labels_dir = store_dir_labels + data_index
    np.save(labels_dir, gt_labels_inst)



"""
MAIN PROGRAM
"""

def main():
    """
    Sensor related settings:
    """

    # Define Destiny Location for Output:
    save_dir = "XXX"

    # Basic Carla Simulation Setup:
    client = carla.Client(host='localhost', port=2000)
    client.set_timeout(15.0)

    # Get prepared Maps:
    data_gen_maps = []
    maps = client.get_available_maps()
    for element in maps:
        data_gen_maps.append(element)

    """
    STARTING DATA GENERATION: num_sequences Iterations
    """
    num_sequences = 2

    for iteration in range(num_sequences):

        # Generate Store Directory:
        global store_dir_pc, store_dir_labels, store_dir_pcint
        gen_iteration = iteration
        store_dir_pc = save_dir + "sequence{0:04}/cube_pc/".format(gen_iteration)
        store_dir_pcint = save_dir + "sequence{0:04}/cube_int/".format(gen_iteration)
        store_dir_labels = save_dir + "sequence{0:04}/labels/".format(gen_iteration)
        if not os.path.exists(store_dir_pc):
            os.makedirs(store_dir_pc)
        if not os.path.exists(store_dir_pcint):
            os.makedirs(store_dir_pcint)
        if not os.path.exists(store_dir_labels):
            os.makedirs(store_dir_labels)

        # Get random Map from List and load world:
        map = random.choice(data_gen_maps)
        print('Loading {} ... '.format(map))
        client.load_world(str(map))

        # Get World from Client and apply settings:
        world = client.get_world()
        spectator = world.get_spectator()
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        world.apply_settings(settings)

        # Get Blueprint Library:
        blueprint_library = world.get_blueprint_library()

        # Get available Spawn-Points:
        spawn_points = world.get_map().get_spawn_points()
        spawn_points_np = np.zeros((len(spawn_points), 3))
        for index, value in enumerate(spawn_points):
            spawn_points_np[index, 0] = value.location.x
            spawn_points_np[index, 1] = value.location.y
            spawn_points_np[index, 2] = value.location.z

        # Get the center of prepared spawn field:
        center_spawnfield = np.around(np.asarray(((np.max(spawn_points_np[:, 0]) + np.min(spawn_points_np[:, 0])) / 2.,
                                                  (max(spawn_points_np[:, 1]) + min(spawn_points_np[:, 1])) / 2.,
                                                  spawn_points_np[0, 2])), decimals=2)
        # Define Settings for half-sphere generation:
        radius_sphere, n_lidarspawnpoints = np.random.uniform(9, 12), 100
        # Generate Lidar Spawn Points around half-sphere:
        lidar_spawn_points_sphere = np.vstack(gen_sphere(radius_sphere, n_lidarspawnpoints)).T

        """
        Spawning of the Pedestrians
        """
        n_pedestrians = random.randint(10, 50)
        pedestrian_T_list = []
        pedestrian_list = []
        batch_ped = []

        # Spawn Pedestrians:
        for i in range(n_pedestrians):
            pedestrian_transformation = random.choice(spawn_points)
            spawn_points.remove(pedestrian_transformation)
            pedestrian_T_list.append(pedestrian_transformation)

        for spawn_point in pedestrian_T_list:
            pedestrian_bp = random.choice(blueprint_library.filter('*walker.pedestrian*'))
            batch_ped.append(carla.command.SpawnActor(pedestrian_bp, spawn_point))


        results_ped = client.apply_batch_sync(batch_ped, True)
        time.sleep(1)

        for i in range(len(results_ped)):
            if results_ped[i].error:
                print('Apply Batch Error (PedSpawn) {}: '.format(i), results_ped[i].error)
            else:
                pedestrian_list.append({"id": results_ped[i].actor_id})

        # Spawn Pedestrian Controller:
        batch_controller = []
        pedestrian_controller_bp = world.get_blueprint_library().find('controller.ai.walker')

        for i in range(len(pedestrian_list)):
            batch_controller.append(
                carla.command.SpawnActor(pedestrian_controller_bp, carla.Transform(), pedestrian_list[i]["id"]))

        results_controller = client.apply_batch_sync(batch_controller, False)
        time.sleep(1)

        for i in range(len(results_controller)):
            if results_controller[i].error:
                print('Apply Batch Error (ConSpawn): ', results_controller[i].error)
            else:
                pedestrian_list[i]["con"] = results_controller[i].actor_id

        # Get List of all active (pedestrian-related) actors:
        all_id = []
        for i in range(len(pedestrian_list)):
            all_id.append(pedestrian_list[i]["con"])
            all_id.append(pedestrian_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # Define Goal Locations with regard to the Spawn Grid:
        upper_bound_x = np.amax(spawn_points_np, axis=0)[0]
        upper_bound_y = np.amax(spawn_points_np, axis=0)[1]
        lower_bound_x = np.amin(spawn_points_np, axis=0)[0]
        lower_bound_y = np.amin(spawn_points_np, axis=0)[1]
        goal_height = spawn_points_np[0, 2]

        ux_uy_goal = carla.Vector3D(x=upper_bound_x, y=upper_bound_y, z=goal_height)
        lx_uy_goal = carla.Vector3D(x=lower_bound_x, y=upper_bound_y, z=goal_height)
        lx_ly_goal = carla.Vector3D(x=lower_bound_x, y=lower_bound_y, z=goal_height)
        ux_ly_goal = carla.Vector3D(x=upper_bound_x, y=lower_bound_y, z=goal_height)
        # constant z value:

        goal_loc_1 = carla.Location(ux_uy_goal)
        goal_loc_2 = carla.Location(lx_uy_goal)
        goal_loc_3 = carla.Location(lx_ly_goal)
        goal_loc_4 = carla.Location(ux_ly_goal)
        goal_list = [goal_loc_1, goal_loc_2, goal_loc_3, goal_loc_4]

        # Initiate Movement of Pedestrians:
        for i in range(0, len(all_actors), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            tmp_goal_loc = random.choice(goal_list)
            all_actors[i].go_to_location(tmp_goal_loc)
            # random max speed
            all_actors[i].set_max_speed(1 + random.random())

        """
        Sensor Setup
        """
        # Generate Blickfeld Sensor Blueprint - Settings can be changed within this function:
        lidar_bf_bp = blickfeld_utils.create_blickfeld_lidar_blueprint(world, blueprint_library, delta)

        # Set initial Sensor/Spectator Transformation:

        # Reminder for Rotation: The constructor method follows a specific order of declaration:
        # (pitch, yaw, roll), which corresponds to (Y-rotation,Z-rotation,X-rotation).
        # --> Further Carla uses a Z-Up-Left Handed-System
        lidar_initialtransform_bf  = carla.Transform(carla.Location(52.5, 185, 2), carla.Rotation(0, 90, 0))
        spectator_initialtransform = carla.Transform(carla.Location(52.5, 185, 8), carla.Rotation(-30, 90, 0))

        # Spawning the Lidar/Spectator:
        lidar_bf = world.spawn_actor(lidar_bf_bp, lidar_initialtransform_bf)
        spectator.set_transform(spectator_initialtransform)

        # Establish Listening Function:
        lidar_bf.listen(lidar_callback)

        dt0 = datetime.now()

        # Start Data Generation from different Lidar Positions:
        for index, sphere_translation in enumerate(lidar_spawn_points_sphere):
            global data_index
            data_index = str(index)
            print('\n -- Iteration ', index, '/', lidar_spawn_points_sphere.shape[0])

            # Update Lidar and Spectator Transformation:
            # transformation_world_sensor = center_sensor(sphere_translation, radius_sphere, center_spawnfield)
            # print('Transformation Lidar: ', transformation_world_sensor)
            lidar_bf.set_transform(center_sensor(sphere_translation, radius_sphere, center_spawnfield))
            spectator.set_transform(center_sensor(sphere_translation, radius_sphere, center_spawnfield))

            # This can fix Open3D jittering issues:
            time.sleep(0.05)
            world.tick()

            process_time = datetime.now() - dt0
            dt0 = datetime.now()

        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        # Destroy all actors in the Scene:
        lidar_bf.destroy()
        for i in range(len(all_actors)):
            if isinstance(all_actors[i], carla.WalkerAIController):
                all_actors[i].stop()
            else:
                all_actors[i].destroy()

        time.sleep(1.2)


    print('--------------------------------')
    print('Data Generation Loop Finished ! ')
    print('--------------------------------')


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


if __name__ =='__main__':
    main()