/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:58
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:45:15
 */

#include <cmath>
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"


namespace KalmanFilter
{

Eigen::Matrix3f IMUData::GetOrientationMatrix()
{
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);

    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;

}


bool IMUData::SyncData(std::deque<IMUData> & UnsyncedData, std::deque<IMUData> &SyncedData, double sync_time)
{
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
    
    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acc.x = front_data.linear_acc.x * front_scale + back_data.linear_acc.x * back_scale;
    synced_data.linear_acc.y = front_data.linear_acc.y * front_scale + back_data.linear_acc.y * back_scale;
    synced_data.linear_acc.z = front_data.linear_acc.z * front_scale + back_data.linear_acc.z * back_scale;

    synced_data.angular_vel.x = front_data.angular_vel.x * front_scale + back_data.angular_vel.x * back_scale;
    synced_data.angular_vel.y = front_data.angular_vel.y * front_scale + back_data.angular_vel.y * back_scale;
    synced_data.angular_vel.z = front_data.angular_vel.z * front_scale + back_data.angular_vel.z * back_scale;

    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;


    synced_data.orientation.Normlize();

    SyncedData.push_back(synced_data);

    return true;



}

}