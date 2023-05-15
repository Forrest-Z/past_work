## 1. **代码结构**
闭环，融合IMU,地图保存在**globalOptimization.cpp**中实现，uwb标定在**uwb_calibration.cpp**中实现．


## 2. **调试**
调试中设计的具体函数和关键参数介绍．
### 2.1闭环
#### **>>>** bool GlobalOptimization::DetectLoopClosure(int& latest_id,int& closed_id): 基于位置的闭环检测;
#### 关键参数:
**matching_min_id_distance :** 关键帧大于该参数才进行闭环检测
**loop_closure_search_radius_:** kdtree搜索半径
#### **>>>** void GlobalOptimization::PerformLoopClosure(): 基于icp求解闭环位姿并加入因子图优化;
#### 关键参数:
**icp_fitness_score_ :** icp得分阈值，该值越小对icp收敛条件越苛刻
**coef_loop_closure_:** 闭环噪声系数，系数越大表示越不信任闭环结果

### 2.2 Aruco优化
#### **>>>** void GlobalOptimization::GauessArucoInitial(): 估计Aruco初始位姿，为优化做准备;
#### 关键参数：详见代码注释

### 2.3 UWB标定
#### **>>>** void UwbCalibration::GuassUwbInitial(): 估计wub基站初始位置，为优化问题提供初值;
#### 关键参数:
**mid_dis_** ,**front_dis_**, **back_dis_:** uwb初值估计中以三圆交点均值做为估计直，此三个变量分别对应中，前，后三个圆的圆心到对应关键帧位置的距离，根据uwb在具体场景的布置适当调整变量对应距离阈值．（例如：采集设备聚基站越远该数值越大）．








