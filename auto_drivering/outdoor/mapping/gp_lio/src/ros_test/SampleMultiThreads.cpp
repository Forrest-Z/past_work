//
// Created by wchen on 2019/12/2.
//

#include "ros_test/SampleMultiThreads.h"

namespace gp_lio{

    SampleMultiThreads::SampleMultiThreads() {

        ROS_DEBUG("Constructor working!!!");

    };

    SampleMultiThreads::~SampleMultiThreads() {

        ROS_DEBUG("Deconstructor working!!!");

    };

//    void SampleMultiThreads::ThreadFunction1() {
//
//        ROS_ERROR("in Thread1");
//
//        int count = 10;
//        while (count > 0) {
//            std::unique_lock<std::mutex> locker(mu_);
//            deque_.push_front(count);
//            std::cout << "t1 add value: " << count << std::endl;
//            locker.unlock();
//            cond_.notify_one();
//            std::this_thread::sleep_for(std::chrono::seconds(1));
//            count--;
//        }
//    };

    // 这个函数会一直判断，导致CPU占用极高
//    void SampleMultiThreads::ThreadFunction2() {
//
//        ROS_ERROR("in Thread1");
//
//        int data = 0;
//        while ( data != 1) {
//            std::unique_lock<std::mutex> locker(mu_);
//            if (!deque_.empty()) {
//                data = deque_.back();
//                deque_.pop_back();
//                locker.unlock();
//                std::cout << "t2 remove value: " << data << std::endl;
//            } else {
//                std::cout << "t2 remove nothing ***** "<< std::endl;
//                locker.unlock();
//            }
//        }
//    };

//    // 每次没查询到有数据时，就惩罚延时500ms，CPU占用有所下降
//    void SampleMultiThreads::ThreadFunction2() {
//
//        ROS_ERROR("in Thread1");
//
//        int data = 0;
//        while ( data != 1) {
//            std::unique_lock<std::mutex> locker(mu_);
//            if (!deque_.empty()) {
//                data = deque_.back();
//                deque_.pop_back();
//                locker.unlock();
//                std::cout << "t2 remove value: " << data << std::endl;
//            } else {
//                std::cout << "t2 remove nothing ***** "<< std::endl;
//                locker.unlock();
//                std::this_thread::sleep_for(std::chrono::milliseconds(500));
//            }
//        }
//    };

    // 使用条件数精准控制
    void SampleMultiThreads::ThreadFunction2() {

        ros::Rate rate(100);
        int data = 1;
        while (ros::ok()){
            ROS_ERROR("in Thread2");
            data ++ ;
//            mu_.lock();
            thread1_.deque_.push_back(data);
//            mu_.unlock();
            std::cout << "t2 add value: " << data << std::endl;

            rate.sleep();

        }

    };

    void SampleMultiThreads::ThreadFunction3() {

        ros::Rate rate(100);

        while (ros::ok()){
            ROS_ERROR("in Thread3");

//            mu_.lock();
            int data = thread1_.deque_.back() + 1;
            thread1_.deque_.push_back(data);
//            mu_.unlock();
            std::cout << "t3 add value: " << data << std::endl;

            rate.sleep();

        }

    };

    void SampleMultiThreads::RunDemo() {

        std::thread* thread1(new std::thread(&thread1::Run, &this->thread1_));
        std::thread* thread2(new std::thread(&SampleMultiThreads::ThreadFunction2, this));
        std::thread* thread3(new std::thread(&SampleMultiThreads::ThreadFunction3, this));

        thread1->join();
        thread2->join();
        thread3->join();

        return;
    };
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "SampleMultiThreads");
    ros::NodeHandle node("~");

    gp_lio::SampleMultiThreads sample_multi_threads;
    sample_multi_threads.RunDemo();

    return 0;
}