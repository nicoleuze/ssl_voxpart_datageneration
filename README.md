# SSL VoxPart Data Generation
Data Generation Pipeline corresponding to the IEEE Sensors 2023 Conference Paper "SSL-VoxPart: A Novel Solid-State LiDAR-Tailored Voxel Partition Approach for 3D Perception". Within our work we specifically conduct to the problems of crowded scene analysis and the lack of sensor diversity in LiDAR datasets. Crowds represent locally high density distributions of people with heavy occlusion effects, demanding for fine-grained predictions. For this, we introduce a Solid-State-LiDAR data generation pipeline tailored to surveillance scenarios. 


> Unfortunately, for legal reasons, the project sponsor and project partners prohibit us from publishing company secrets, such as the detailed sensor implementation. For this reason, only basic CARLA sensors are used in the present repository. Custom sensors can be added according to the [Carla Docs: Create Sensor](https://carla.readthedocs.io/en/0.9.13/tuto_D_create_sensor/).    

# Get Started
## Installation:
### 1. Clone this repository
```shell
git clone https://github.com/nicoleuze/sslvoxpart_datagen && cd sslvoxpart_datagen
```
### 2. Install CARLA
Following the install documents from the official [Carla Documentation](https://carla.readthedocs.io/en/0.9.13/start_quickstart/)
> Code is tested with CARLA Version 0.9.13 on Ubuntu 20.04

### 3. Install Dependencies
Install dependencies by 
```shell
pip install -r requirements.txt
```
## Data Generation
Launch CARLA
```shell
cd CARLA_ROOT_DIR && make launch 
```
Start data generation pipeline with specified saving directory and amount of sequences
```shell
python data_generation.py --save_dir SAVE_DIRECTORY --seq AMOUNT_SEQUENCES 
```

Dataset with the following folder structure should be generated
```

├──SAVE_DIRECTORY/
    ├── sequence0000/           
    │   ├── cos_ang/
    |         ├── 0.npy
    |         └── ... 
    |   ├── labels/
    |         ├── 0.npy
    |         └── ... 
    │   └── pointcloud/
    |         ├── 0.npy
    |         └── ... 
    └── sequenceXXXX/
        └── ...
```
## Data Visualization
We provide a simple visualization tool that can be executed via
```shell
python vis_data.py --data_dir PATH_TO_DATA --sample_seq SEQUENCE_IDX --sample SAMPLE_IDX
```

# Citation
Please cite our paper if this code benefits your research:
```
@InProceedings{leuze_2023_IEEESensors,
author = {Leuze, Nico and Schaub, Henry and Hoh, Maximilian and Schoettl, Alfred},
title = {SSL-VoxPart: A Novel Solid-State LiDAR-Tailored Voxel Partition Approach for 3D Perception},
booktitle = {Proceedings of the 2023 IEEESensors},
month = {October},
year = {2023}
}
```


