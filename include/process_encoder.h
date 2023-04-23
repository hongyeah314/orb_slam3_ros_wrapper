//
// Created by jhon on 4/20/23.
//

#ifndef ORB_SLAM3_ROS_WRAPPER_PROCESS_ENCODER_H
#define ORB_SLAM3_ROS_WRAPPER_PROCESS_ENCODER_H
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <custom_msgs/Encoder.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//多线程环境变量
std::condition_variable con;

//初始化一些变量
double current_time = -1;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<custom_msgs::EncoderConstPtr> encoder_buf;
int sum_of_wait = 0;

std::mutex m_buf; // imu_buf lock
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;
std::mutex e_buf; // encoder_buf lock

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;
double last_encoder_t = 0;


class process_encoder {
    void predict(const sensor_msgs::ImuConstPtr &imu_msg);
    void upodate();
    std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr,
            std::vector<custom_msgs::EncoderConstPtr>>>
    getMeasurements()
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void encoder_callback(const custom_msgs::EncoderConstPtr &encoder_msg);
};


#endif //ORB_SLAM3_ROS_WRAPPER_PROCESS_ENCODER_H
