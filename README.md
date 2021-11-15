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
./bin/monoVO_KT <link_to_the_image_folder>
```
For example:
```
./bin/monoVO_KT images
```
