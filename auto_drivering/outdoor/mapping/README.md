# gp_lio

### ******************************************************************系统************************************************************************
Ubuntu 64-bit 16.04


### ****************************************************************安装依赖************************************************************************
## 1.1. **ROS** 
ROS Kinetic [ROS Installation](http://wiki.ros.org/ROS/Installation)


## 1.2. **Ceres Solver** 
Follow [Ceres Installation](http://ceres-solver.org/installation.html).　[Release 1.14.0 is suggested on ubuntu 16.04]
    cd CUHK/lib
    tar xvf ceres-solver-1.14.0.tar.gz
    cd ceres-solver-1.14.0
    mkdir build
    cd build
    cmake ..
    make -j8


## 1.3.**Gtsam**
Follow [Gtsam Installation](https://github.com/borglab/gtsam).　[Release 4.0.0 is suggested on ubuntu 16.04]
    cd CUHK/lib
    tar xvf gtsam.tar.gz
    cd gtsam
    mkdir build
    cd build
    cmake ..
    make -j8


## 1.4. **Eigen** 
Follow [Eigen Installation](http://ceres-solver.org/installation.html).　[Release 3.2.92 is suggested on ubuntu 16.04]
    gtsam自带,无需单独安装


## 1.5. **PCL**　
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).　[Release 1.7.2 is suggested on ubuntu 16.04]
    ROS中已默认安装该版本,无需单独安装


## 1.6.**OpenCV**
Follow [OpenCV Installation](https://opencv.org/releases/).　[Release 3.3.1 is suggested on ubuntu 16.04]
    ROS中已默认安装该版本,无需单独安装


## 1.7.**nmea-navsat**
Follow [nmea-navsat Installation](http://wiki.ros.org/nmea_navsat_driver).
    sudo apt-get install ros-kinetic-nmea-msgs ros-kinetic-nmea-navsat-driver


### *************************************************************编译工程***************************************************************************
Clone the repository and catkin_make:
```
    创建工程: catkin_ws

    cd ~/catkin_ws/src

    tar xvf gp_lio.tar.gz 
    tar xvf nmea_navsat_driver.tar.gz

    cd ../ 

    catkin_make

    source ~/catkin_ws/devel/setup.bash
```


### ************************************************************修改路径****************************************************************************

```
    将gp_lio/config/CUHK_config_no_extrinsic.yaml中第4行 output_path(/home/luo/work/catkin_ws/src/gp_lio/result) 
    修改为: result文件夹的绝对路径   eg.xxx/xxx/xxx/gp_lio/result

    将gp_lio/config/localization.yaml 中第2行 map_path("/home/luo/work/catkin_ws/src/gp_lio/result") 
    修改为: result文件夹的绝对路径

    将gp_lio/src/gp_lio/localization/localization_node.cpp文件中第11行(/home/luo/work/catkin_ws/src/gp_lio/config/localization.yaml) 
    修改为localization.yaml文件的绝对路径

    将gp_lio/src/gp_lio/localization/parameters.cpp文件中第46行(/home/luo/work/catkin_ws/src/gp_lio/config/localization.yaml)
    修改为localization.yaml文件的绝对路径
```

    修改完路径以后在编译工程

### ****************************************************************运行测试************************************************************************
```
    roslaunch gp_lio estimator_node_CUHK.launch

    roslaunch nmea_driver_to_vel_fix.launch

    rosbag play YOUR_DATASET_FOLDER/mapping_data.bag 

    保存地图:

        在终端中按 s 键保存地图(运行 roslaunch gp_lio estimator_node_CUHK.launch的终端)

```

