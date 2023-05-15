# LUVI-Mapping


LUVI-Mapping is a slam system  which uses [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM.git) as lidar odometry and  take imu and uwb to construct a more robust and accurate Multi-sensor mapping and positioning system.


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04
ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Eigen**
Follow [Eigen Installation](http://eigen.tuxfamily.org/dox/GettingStarted.html)  [Release 3.3.4 is suggested].

### 1.3. **glog**
Follow [glog Installation](https://github.com/google/glog/)  [Release 0.4.0 is suggested].

### 1.4. **gflags**
Follow [gflags Installation](https://github.com/gflags/gflags)  [Release 2.2.1 is suggested].

### 1.5. **Ceres Solver** 
Follow [Ceres Installation](http://ceres-solver.org/installation.html)  [Release 1.14.0 is suggested].

### 1.6. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html) [Release 1.8.1 is suggested].

### 1.7.**OpenCV**
Follow [OpenCV Installation](https://opencv.org/releases/) [Release  3.2.0 is suggested ].

### 1.8.**GTSAM**
Follow [GTSAM Installation](https://gtsam.org/get_started/) [Release 4.1 is suggested  ].


## 2. Build LUVI-Mapping (Release mode is suggested)
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone git@github.com:WChen09/LUVI-Mapping.git
    git checkout -b xc_dev origin/xc_dev
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. test on rosbag 

```
    roslaunch luvi-mapping luvi-mapping_VLP_16.launch
    rosbag play YOUR_DATASET_FOLDER/xx.bag
```

## 4.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM.git).

## 4.Acknowledgements
PLs update  the gatsam vision to 4.1.
