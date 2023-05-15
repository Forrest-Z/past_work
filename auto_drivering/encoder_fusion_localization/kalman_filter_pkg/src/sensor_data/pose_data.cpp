/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:07:17
 * @LastEditors: luo
 * @LastEditTime: 2021-11-22 15:21:58
 */
#include <iostream>

#include "kalman_filter_pkg/sensor_data/pose_data.hpp"

namespace KalmanFilter
{
Eigen::Quaternionf PoseData::GetQuaternion()
{
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;

}



bool PoseData::SyncData(std::deque<PoseData> & UnsyncedData, std::deque<PoseData> &SyncedData, double sync_time)
{
    std::cout << "UnsyncedData size = " << UnsyncedData.size() << std::endl;
    // std::cout << "UnsyncedData.front().time = " << UnsyncedData.front().time << std::endl;
    // std::cout << "UnsyncedData.at(1).time = " << UnsyncedData.at(1).time << std::endl;
    std::cout << "sync_time  = " << sync_time << std::endl;

    while(UnsyncedData.size() >= 2)
    {
        if(UnsyncedData.front().time > sync_time)
            return false;

        if(UnsyncedData.at(1).time < sync_time)
        {
            UnsyncedData.pop_front();
            continue;
        }

        if(sync_time - UnsyncedData.front().time > 0.2)
        {
            UnsyncedData.pop_front();
            return false;
        }

        if(UnsyncedData.at(1).time - sync_time > 0.2)
        {
            UnsyncedData.pop_front();
            return false;
        }

        break;
    }

    if(UnsyncedData.size() < 2)
        return false;
    
    std::cout << "11111111111 size = " << std::endl;

    PoseData front_data = UnsyncedData.at(0);
    PoseData back_data = UnsyncedData.at(1);
    PoseData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.pose(0, 3) = front_data.pose(0, 3) * front_scale + back_data.pose(0, 3) * back_scale;
    synced_data.pose(1, 3) = front_data.pose(1, 3) * front_scale + back_data.pose(1, 3) * back_scale;
    synced_data.pose(2, 3) = front_data.pose(2, 3) * front_scale + back_data.pose(2, 3) * back_scale;

    Eigen::Quaternionf front_q = front_data.GetQuaternion();
    Eigen::Quaternionf back_q = back_data.GetQuaternion();
    Eigen::Quaternionf mid_q;

    mid_q.w() = front_q.w() * front_scale + back_q.w() * back_scale;
    mid_q.x() = front_q.x() * front_scale + back_q.x() * back_scale;
    mid_q.y() = front_q.y() * front_scale + back_q.y() * back_scale;
    mid_q.z() = front_q.z() * front_scale + back_q.z() * back_scale;


    mid_q.normalize();
    synced_data.pose.block<3, 3>(0, 0) = mid_q.toRotationMatrix().cast<float>();

    SyncedData.push_back(synced_data);
    std::cout << "SyncedData size = " << SyncedData.size() << std::endl;

    return true;



}

} // namespace localization_ns
