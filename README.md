![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
# Mono Visual Odometry
This represents a simple implementation of mono visual odometry uisng two different approaches to feature tracking. The first one uses KLT to track feature across two frames while the latter uses a feature matching approach.


# Install
```
cd build
cmake ..
make -j8
```

# Data
Download Kitti dataset.

# Usage:
```
./bin/monoVO_KT <link_to_the_image_folder>
```
For example:
```
./bin/monoVO_KT images
```
