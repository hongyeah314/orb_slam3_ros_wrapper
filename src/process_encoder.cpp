//
// Created by jhon on 4/20/23.
//

#include "process_encoder.h"
void process_encoder::predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if(init_imu)
    {
        latest_time =t ;
        init_imu = 0;
        return;
    }

    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx,dy,dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d  angular_velocity{rx,ry,rz};

    Eigen::Vector3d un_acc_0 = tmp_Q*(acc_0-tmp_Ba)-estimator.g;

    Eigen::Vector3d un_gyr = 0.5*(gyr_0+angular_velocity)-tmp_Bg;

    tmp_Q = tmp_Q*Utility::deltaQ(un_gyr*dt);
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;


}

void process_encoder::update(){
    TicToc t_pre
}
std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr,
        std::vector<custom_msgs::EncoderConstPtr>>>
process_encoder::getMeasurements()
{
    std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr,
            std::vector<custom_msgs::EncoderConstPtr>>> measurements;

    while (true)
    {
    // 这里是循环终止条件
    if (imu_buf.empty() || feature_buf.empty() || encoder_buf.empty())
        return measurements;

    if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
    {
        //ROS_WARN("wait for imu, only should happen at the beginning");
        sum_of_wait++;
        return measurements;
    }

    if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
    {
        ROS_WARN("throw img, only should happen at the beginning");
        feature_buf.pop();
        continue;
    }

    sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
    feature_buf.pop();

    std::vector<sensor_msgs::ImuConstPtr> IMUs;
    while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
    {
        IMUs.emplace_back(imu_buf.front());
        imu_buf.pop();
    }

    // encoder
    // 1、进行一次判断，要比早于图像帧的IMU数据还要早一帧
    std::vector<custom_msgs::EncoderConstPtr> encoders;
    if (IMUs.size() > 1)
    {
        while (encoder_buf.front()->header.stamp.toSec() < IMUs[IMUs.size()-2]->header.stamp.toSec())
        {
            encoders.emplace_back(encoder_buf.front());
            encoder_buf.pop_front();
        }
    }

    // 2、多添加一帧IMU数据
    IMUs.emplace_back(imu_buf.front());
    if (IMUs.empty())
        ROS_WARN("no imu between two image");

    // 3、和多一帧的IMU数据进行比较，因为queue不能遍历，所以换deque
    for (auto iter = encoder_buf.begin(); iter != encoder_buf.end(); iter++)
    {
        if ((*iter)->header.stamp.toSec() < IMUs.back()->header.stamp.toSec())
        {
            encoders.emplace_back(*iter);
        }
        else
        {
            encoders.emplace_back(*iter);
            break;
        }
    }
    // 4、比IMU数据要多一帧，用来时间戳差分计算
    // encoders.emplace_back(encoder_buf.front());
    if (encoders.empty())
        ROS_WARN("no encoder between two image.");
    measurements.emplace_back(IMUs, img_msg, encoders);
}
return measurements;
}
void process_encoder::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void process_encoder::encoder_callback(const custom_msgs::EncoderConstPtr &encoder_msg)
{
    if (encoder_msg->header.stamp.toSec() <= last_encoder_t)
    {
        ROS_WARN("encoder message in disorder!");
    }

    last_encoder_t = encoder_msg->header.stamp.toSec();

    e_buf.lock();
    encoder_buf.push_back(encoder_msg);
    e_buf.unlock();
    con.notify_one();

    last_encoder_t = encoder_msg->header.stamp.toSec(); // ?

}
