# SSL VoxPart Data Generation
Data Generation Pipeline corresponding to the IEEE Sensors 2023 Conference Paper "SSL-VoxPart: A Novel Solid-State LiDAR-Tailored Voxel Partition Approach for 3D Perception". Within our work we specifically conduct to the problems of crowded scene analysis and the lack of sensor diversity in LiDAR datasets. Crowds represent locally high density distributions of people with heavy occlusion effects, demanding for fine-grained predictions. For this, we introduce a SSL data generation pipeline tailored to surveillance scenarios. Unfortunately, for legal reasons, the project sponsor and project partners prohibit us from publishing company secrets, such as the detailed sensor implementation. For this reason, only basic CARLA sensors are used in the present repository. Custom sensors can be added according to the [Carla documentation](https://carla.readthedocs.io/en/0.9.13/tuto_D_create_sensor/).    

# Get Started
## Installation:
### 1. Clone this repository
'''shell
git clone https://github.com/nicoleuze/sslvoxpart_datagen && cd sslvoxpart_datagen
'''
### 2. Install CARLA 
Following the install documents from the official [Carla Documentation](https://carla.readthedocs.io/en/0.9.13/start_quickstart/)
> Code is tested with CARLA Version 0.9.13 on Ubuntu 20.04

Install dependencies by 
'''shell
pip install -r requirements.txt
'''
## Data Generation

```shell
python data_generation.py NUM_GPUS --cfg_file PATH_TO_CONFIG_FILE
#For example,
bash scripts/dist_train.sh 8 --cfg_file PATH_TO_CONFIG_FILE
```
