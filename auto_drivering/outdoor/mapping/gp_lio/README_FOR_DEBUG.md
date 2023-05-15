## 1. **loop closure**
#### [主要实现室内环境下基于位姿的闭环]
### 1.1代码位置:　
GlobalMapper.cpp:  LoopClosureDetection()
### 1.2主要参数：
####**>detection_method**:"lpdnet"指用lpdnet网络检测闭环，"loam"指基于位置的闭环检测，前者一般用于户外环境，后者更适合室内场景．
####**>sequence_n**: 用于闭环的关键帧序列数，越多帧求解位姿更准确，计算量也更大．


## 2. **map resuse**
#### [主要实现地图保存和基于已有地图的增量建图] 
### 2.1代码位置:　
GlobalMapper.cpp:  SavePosegraph(), LoadPosegraph(), FastRelocalization(State & updated_state),  MapMerge(), 
### 2.2主要参数：
####**>reuse_map**: １表示启动地图复用功能模块．
####**>fast_relocalization**: 1表示启用快速重定位功能模块．





