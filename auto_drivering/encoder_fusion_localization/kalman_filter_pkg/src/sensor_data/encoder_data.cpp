/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-24 20:34:26
 * @LastEditors: luo
 * @LastEditTime: 2022-01-13 15:37:21
 */
#include "kalman_filter_pkg/sensor_data/encoder_data.hpp"

namespace KalmanFilter
{
bool EncoderData::GetStatus()
{
    status_buff.push_back(v_encoder);

    if(status_buff.size() > 2)
    {
        status_buff.pop_front();
    }

}

bool EncoderData::CalculateNLeft()
{   
    double mid_FreL;
    
    if( control_mode_status == 1 )
    {
        mid_FreL = (frequency_l1 + frequency_l2 ) / 2;

    }

    if( control_mode_status == 2 )
    {
        mid_FreL = -( frequency_l1 + frequency_l2 ) / 2;

    }

    if(control_mode_status == 0)
    {
        if(vehicle_speed == 0)
        {
            mid_FreL = 0.0;
        }
        else if( status_buff.at(0) > 0)
        {
            mid_FreL = (frequency_l1 + frequency_l2 ) / 2;

        }else 
        {
            mid_FreL = ( frequency_l1 + frequency_l2 ) / 2;
        }
    }



    n_left = mid_FreL / num;

}


bool EncoderData::CalculateNRight()
{
    
    double mid_FreR;

    // std::cout << " CalculateNRight " << std::endl;
    
    if( control_mode_status == 1 )
    {
        // std::cout << " CalculateNRight 111111 " << std::endl;
        mid_FreR = ( frequency_r1 + frequency_r2 ) / 2;
    }
    
    if( control_mode_status == 2 )
    {
        // std::cout << " CalculateNRight 222222 " << std::endl;
        mid_FreR = -( frequency_r1 + frequency_r2 ) / 2;
    }

    if(control_mode_status == 0)
    {
        // std::cout << " CalculateNRight 0000000000 " << std::endl;
        if(vehicle_speed == 0)
        {
            // std::cout << " CalculateNRight vehicle_speed == 0 " << std::endl;
            mid_FreR = 0.0;
        }
        else if( status_buff.at(0) > 0)
        {
            // std::cout << " CalculateNRight status_buff > 0 " << std::endl;
            mid_FreR = ( frequency_r1 + frequency_r2 ) / 2;

        }else 
        {
            // std::cout << " CalculateNRight status_buff < 0 " << std::endl;
             mid_FreR = ( frequency_r1 + frequency_r2 ) / 2;
        }
    }


    n_right = mid_FreR / num;

}

bool EncoderData::CalculateVelo()
{
    w_left = n_left * M_PI * 2.0;
    w_right = n_right * M_PI * 2.0;

    velo_left = n_left * wheel_radius * M_PI * 2.0;
    velo_right = n_right * wheel_radius * M_PI * 2.0;

    r_encoder = wheel_base * (velo_right + velo_left) / (velo_right - velo_left);

    v_encoder = 0.5 * (velo_left + velo_right);
    w_encoder = (w_left + w_right) / wheel_base;

}




bool EncoderData::SyncData(std::deque<EncoderData> & UnsyncedData, std::deque<EncoderData> &SyncedData, double sync_time)
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
    
    EncoderData front_data = UnsyncedData.at(0);
    EncoderData back_data = UnsyncedData.at(1);
    EncoderData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);

    synced_data.time = sync_time;

    synced_data.velo_left = front_data.velo_left * front_scale + back_data.velo_left * back_scale;
    synced_data.velo_right = front_data.velo_right * front_scale + back_data.velo_right * back_scale;

    SyncedData.push_back(synced_data);

    return true;


}

}