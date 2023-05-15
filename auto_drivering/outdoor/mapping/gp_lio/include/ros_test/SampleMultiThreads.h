//
// Created by wchen on 2019/12/2.
//
// ref: https://www.cnblogs.com/c4isr/p/9205164.html

#ifndef SRC_SAMPLEMULTITHREADS_H
#define SRC_SAMPLEMULTITHREADS_H

#include <iostream>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>

namespace gp_lio{

    class thread1{
    public:

        thread1(){

        };

        ~thread1(){};

        void Run(){

            ros::Rate rate(0.1);
            while(ros::ok()){

                ROS_ERROR("in Thread1");
                for (int i = 0; i < deque_.size(); ++i) {
                    std::cout << deque_.at(i) << " ";
                }
                std::cout << std::endl;

                rate.sleep();
            }


        }

        std::deque<int> deque_;

    };

    class SampleMultiThreads {

    public:

        SampleMultiThreads();

        ~SampleMultiThreads();

        void ThreadFunction1();

        void ThreadFunction2();

        void ThreadFunction3();

        void RunDemo();

    private:

        std::mutex mu_;
        std::condition_variable cond_;

        thread1 thread1_;

    };

}


#endif //SRC_SAMPLEMULTITHREADS_H
