/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:07:30
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:45:22
 */
#include "kalman_filter_pkg/sensor_data/velocity_data.hpp"

namespace KalmanFilter
{


bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time)
{
    while( UnsyncedData.size() >= 2)
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


    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    VelocityData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);

    synced_data.time = sync_time;

    synced_data.linear_vectority.x = front_data.linear_vectority.x * front_scale + back_data.linear_vectority.x * back_scale;
    synced_data.linear_vectority.y = front_data.linear_vectority.y * front_scale + back_data.linear_vectority.y * back_scale;
    synced_data.linear_vectority.z = front_data.linear_vectority.z * front_scale + back_data.linear_vectority.z * back_scale;

    synced_data.angular_vectory.x = front_data.angular_vectory.x * front_scale + back_data.angular_vectory.x * back_scale;
    synced_data.angular_vectory.y = front_data.angular_vectory.y * front_scale + back_data.angular_vectory.y * back_scale;
    synced_data.angular_vectory.z = front_data.angular_vectory.z * front_scale + back_data.angular_vectory.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix)
{
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();

    Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
    Eigen::Vector3d t = matrix.block<3, 1>(0, 3);

    Eigen::Vector3d w(angular_vectory.x, angular_vectory.y, angular_vectory.z);
    Eigen::Vector3d v(linear_vectority.x, linear_vectority.y, linear_vectority.z);

    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * t(2) - w(2) * t(1);
    delta_v(1) = w(2) * t(0) - w(0) * t(2);
    delta_v(2) = w(0) * t(1) - w(1) * t(0);

    v += delta_v;

    w = R.transpose() * w;
    v = R.transpose() * v;

    angular_vectory.x = w(0);
    angular_vectory.y = w(1);
    angular_vectory.z = w(2);

    linear_vectority.x = v(0);
    linear_vectority.y = v(1);
    linear_vectority.z = v(2);

    
}

void VelocityData::NED2ENU(void)
{
    LinearVelocity linear_velocity_enu;
    linear_velocity_enu.x = +linear_vectority.y;
    linear_velocity_enu.y = +linear_vectority.x;
    linear_velocity_enu.z = +linear_vectority.z;

    linear_vectority.x = linear_velocity_enu.x;
    linear_vectority.y = linear_velocity_enu.y;
    linear_vectority.z = linear_velocity_enu.z;

    AngularVelocity angular_enu;
    angular_enu.x = +angular_vectory.y;
    angular_enu.y = +angular_vectory.x;
    angular_enu.z = +angular_vectory.z;

    angular_vectory.x = angular_enu.x;
    angular_vectory.y = angular_enu.y;
    angular_vectory.z = angular_enu.z;

}
}