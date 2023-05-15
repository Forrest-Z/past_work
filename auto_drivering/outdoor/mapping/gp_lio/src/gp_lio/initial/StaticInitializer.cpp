//
// Created by wchen on 2020/4/24.
//

#include "gp_lio/initial/StaticInitializer.h"

namespace gp_lio
{

    StaticInitializer::StaticInitializer()
    {

        local_gravity_.setZero();
        bias_a_.setZero();
        bias_g_.setZero();
        gravity_aligned_rot_.Identity();

        // moving_detection
        moving_threshold_ = 0.0;
        moving_size_ = 50;
    }

    StaticInitializer::~StaticInitializer()
    {
    }

    void StaticInitializer::GetIMUData(std::vector<ImuMeasurement> imu_vector)
    {

        Eigen::Vector3d sum_w, sum_a;
        sum_w.setZero();
        sum_a.setZero();
        for (int i = 0; i < imu_vector.size(); ++i)
        {

            sum_w += imu_vector.at(i).getGyr();
            sum_a += imu_vector.at(i).getAcc();
        }
        Eigen::Vector3d avg_w = sum_w / float(imu_vector.size());
        Eigen::Vector3d avg_a = sum_a / float(imu_vector.size());

        // noise std
        Eigen::Vector3d std_a_sum, std_w_sum;
        std_a_sum.setZero();
        std_w_sum.setZero();
        for (int k = 0; k < imu_vector.size(); ++k) {

            auto gyr = imu_vector.at(k).getGyr();
            auto diff_w = gyr - avg_w;
            std_w_sum.x() += diff_w.x()*diff_w.x();
            std_w_sum.y() += diff_w.y()*diff_w.y();
            std_w_sum.z() += diff_w.z()*diff_w.z();

            auto acc = imu_vector.at(k).getAcc();
            auto diff_a = acc - avg_a;
            std_a_sum.x() += diff_a.x()*diff_a.x();
            std_a_sum.y() += diff_a.y()*diff_a.y();
            std_a_sum.z() += diff_a.z()*diff_a.z();
//            std::cout << acc.transpose() << " " << diff_a.transpose() << " " << gyr.transpose() << " " << diff_w.transpose() << std::endl;

        }

        auto noise_a = Eigen::Vector3d ({sqrt(std_a_sum.x()/double(imu_vector.size()-1)),
                                         sqrt(std_a_sum.y()/double(imu_vector.size()-1)),
                                         sqrt(std_a_sum.z()/double(imu_vector.size()-1))});
        noise_std_a_ = noise_a.norm();

        auto noise_w = Eigen::Vector3d ({sqrt(std_w_sum.x()/double(imu_vector.size()-1)),
                                         sqrt(std_w_sum.y()/double(imu_vector.size()-1)),
                                         sqrt(std_w_sum.z()/double(imu_vector.size()-1))});
        noise_std_w_ = noise_w.norm();

//        std::cout << std::endl << avg_w.transpose() << "/ " << avg_a.transpose() << "/ "
//                  << std_w_sum.transpose() << "/ " << std_a_sum.transpose()
//                  << "/ " << noise_w.transpose() << "/ " << noise_a.transpose()
//                  << "/ " << noise_std_w_ << "/ " << noise_std_a_ << std::endl;

        // bias
        bias_g_ = avg_w;

        Eigen::Vector3d local_gravity = avg_a;
        local_gravity_ = local_gravity;

        // gravity alignment
        Eigen::Matrix3d R_w_b0 = Utility::g2R(local_gravity);
        gravity_aligned_rot_ = R_w_b0;

        Eigen::Vector3d sum_bias_a;
        sum_bias_a.setZero();
        Eigen::Vector3d global_gravity = GRAVITY;
        for (int j = 0; j < imu_vector.size(); ++j)
        {

            sum_bias_a += (imu_vector.at(j).getAcc() - R_w_b0.transpose() * global_gravity);
        }

        bias_a_ = sum_bias_a / float(imu_vector.size());

//        ROS_DEBUG_STREAM("init bias a: " << bias_a_.transpose());
//        ROS_DEBUG_STREAM("init bias g: " << bias_g_.transpose());
//        ROS_DEBUG_STREAM("init local gravity: " << local_gravity_.transpose());
//        ROS_DEBUG_STREAM("init R_w_b0: " << Utility::R2ypr(R_w_b0).transpose());
    }

    bool StaticInitializer::MovingDetector(std::vector<ImuMeasurement> imu_measurements, bool force_check)
    {
        std::vector<double> ax_vec, ay_vec, az_vec;
        ax_vec.resize(imu_measurements.size());
        ay_vec.resize(imu_measurements.size());
        az_vec.resize(imu_measurements.size());
        for (size_t i = 0; i < imu_measurements.size(); i++)
        {
            ax_vec[i] = imu_measurements[i].ddx_;
            ay_vec[i] = imu_measurements[i].ddy_;
            az_vec[i] = imu_measurements[i].ddz_;
        }

        double ax_stdev;
        double ay_stdev;
        double az_stdev;
        Utility::ComputeMeanStdev(ax_stdev, ax_vec);
        Utility::ComputeMeanStdev(ay_stdev, ay_vec);
        Utility::ComputeMeanStdev(az_stdev, az_vec);

        double value_squ = pow(ax_stdev, 4) + pow(ay_stdev, 4) + pow(az_stdev, 4);

        if ((moving_threshold_ == 0.0) && !force_check)
        {
            moving_threshold_ = value_squ;
            return false;
        }
        else
        {
            if (value_squ <= moving_threshold_ * 1.5)
                return false;
            else
                return true;
        }
    }

    bool StaticInitializer::MovingDetector(std::vector<sensor_msgs::ImuConstPtr> imu_measurements)
    {
        std::vector<double> ax_vec, ay_vec, az_vec;
        ax_vec.resize(imu_measurements.size());
        ay_vec.resize(imu_measurements.size());
        az_vec.resize(imu_measurements.size());
        for (size_t i = 0; i < imu_measurements.size(); i++)
        {
            ax_vec[i] = imu_measurements[i]->linear_acceleration.x;
            ay_vec[i] = imu_measurements[i]->linear_acceleration.y;
            az_vec[i] = imu_measurements[i]->linear_acceleration.z;
        }

        double ax_stdev;
        double ay_stdev;
        double az_stdev;
        Utility::ComputeMeanStdev(ax_stdev, ax_vec);
        Utility::ComputeMeanStdev(ay_stdev, ay_vec);
        Utility::ComputeMeanStdev(az_stdev, az_vec);

        double value_squ = pow(ax_stdev, 4) + pow(ay_stdev, 4) + pow(az_stdev, 4);

        ROS_DEBUG("moving check value: %f ? %f", value_squ,  0.00001);

        if (value_squ <=  0.00001)
            return false;
        else
            return true;
    }

} // namespace gp_lio