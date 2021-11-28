![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
# Mono Visual Odometry
This represents a simple implementation of mono visual odometry uisng two different approaches to feature tracking. The first one uses KLT to track feature across two frames while the latter uses solely feature matching.

# Prerequisites
- [x] Sophus
- [x] Eigen3
- [x] Opencv4
First, install `Eigen3` and `fmt` that are dependenices of Sophus
```
sudo bash scripts/install_sophus_deps.sh
```

Install `Sophus`
```
sudo bash scripts/install_sophus.sh
```

Install `OpenCV`from source: edit the link to `python3.7` and run [this script](https://github.com/tynguyen/SAIC_docker/blob/master/installation_scripts/install_opencv4_python3.7_from_source.sh)


# Install
```
git clone git@github.com:tynguyen/mono_visual_odometry.git
cd mono_visual_odometry
mkdir build && cd build
cmake ..
make -j4
```

# Data
Download Kitti dataset.

# Usage:
```
cd mono_visual_odometry
./bin/monoVO_KT <path_to_the_image_folder>
```
For example:
```
./bin/monoVO_KT images
```
The predicted trajectory (in SO3 format [R| t]) will be written to a file `results/pred_traj_<squence_index>.txt`. To evaluate this trajectory, run
```
./bin/evaluate_pred_traj <path_to_pred_traj> <path_to_gt_traj>
```
For example:
```
./bin/evaluate_pred_traj ../results/pred_poses_07.txt /media/tynguyen/docker_folder/kitti/dataset/sequences/07/pose.txt
```

# Examples
### Sequence 01
![sequence_01](/media/monoVO_KLT_video_01.gif)

### Sequence 02
![sequence_02](/media/monoVO_KLT_video_02.gif)

### Sequence 03
![sequence_03](/media/monoVO_KLT_video_03.gif)

### Sequence 04
![sequence_04](/media/monoVO_KLT_video_04.gif)

### Sequence 05
![sequence_05](/media/monoVO_KLT_video_05.gif)

### Sequence 06
![sequence_06](/media/monoVO_KLT_video_06.gif)

### Sequence 07
![sequence_07](/media/monoVO_KLT_video_07.gif)

### Sequence 08
![sequence_08](/media/monoVO_KLT_video_08.gif)