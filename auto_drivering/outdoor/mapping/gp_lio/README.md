# gp_lio
Gaussian Processing based Lidar-Inertial-Odometry

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 
ROS Kinetic [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Eigen** 
Follow [Eigen Installation](http://ceres-solver.org/installation.html).　[Release 3.2.92 is suggested on ubuntu 16.04]

### 1.3. **Ceres Solver** 
Follow [Ceres Installation](http://ceres-solver.org/installation.html).　[Release 1.14.0 is suggested on ubuntu 16.04]

### 1.4. **PCL**　
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).　[Release 1.7.2 is suggested on ubuntu 16.04]

### 1.5.**OpenCV**
Follow [OpenCV Installation](https://opencv.org/releases/).　[Release 3.3.1 is suggested on ubuntu 16.04]

### 1.5.**Gtsam**
Follow [Gtsam Installation](https://github.com/borglab/gtsam).　[Release 4.0.0 is suggested on ubuntu 16.04]

### 1.7.**nmea-navsat**
Follow [nmea-navsat Installation](http://wiki.ros.org/nmea_navsat_driver).
sudo apt-get install ros-kinetic-nmea-msgs ros-kinetic-nmea-navsat-driver


## 2. Build gp_lio
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src

    tar xvf gp_lio.tar.gz 
    tar xvf nmea_navsat_driver.tar.gz

    cd ../ 

    catkin_make

    source ~/catkin_ws/devel/setup.bash
```

## 3. test on rosbag 
```
    roslaunch gp_lio estimator_node_CUHK.launch

    roslaunch nmea_driver_to_vel_fix.launch

    rosbag play YOUR_DATASET_FOLDER/mapping_data.bag 

    --- to save map : when all adta run out and map is finished, press "s " to save all map datas and pose graph.
```

