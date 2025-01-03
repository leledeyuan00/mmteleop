#ifndef MMTELEOP_IMU_HPP__
#define MMTELEOP_IMU_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <fstream>
#include <thread>

// ros
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "kdl/frames.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

// tf sub
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

// low pass filter
#include <control_toolbox/control_toolbox/filters.hpp>

// custom
#include "cartesian_controller_msgs/srv/joint_move.hpp"
#include "garment_motion_base/garment_motion_base.hpp"
#include "mmteleop/kalman_filter.hpp"

namespace garment_research
{

class MmteleopIMU : public GarmentMotionBase
{
public:
    MmteleopIMU();

private:
    /* function */
    virtual void custom_init();

    void tf_update();
    // tasks
    void test_service(bool on);
    virtual void tasks_init();


    /* variable */
    // sub
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_l_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_r_;

    // srvs
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_srv_;
    bool teleop_start_;

    // tf   
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    tf2::BufferCore buffer_;

    // ros time
    rclcpp::Clock ros_clock_;
    rclcpp::Time start_time_;

    // robot transform matrix
    sensor_msgs::msg::Imu imu_msg_l_;
    sensor_msgs::msg::Imu imu_msg_r_;
    bool imu_received_l_{false};
    bool imu_received_r_{false};

    // body transofrm matrix
    Eigen::Isometry3d body_neck_;
    Eigen::Isometry3d body_right_hand_;
    Eigen::Isometry3d body_left_hand_;

    // IMU
    Eigen::Vector3d imu_acc_l_;
    Eigen::Vector3d imu_acc_r_;
    Eigen::Quaterniond imu_ori_l_;
    Eigen::Quaterniond imu_ori_r_;

    // kalman filter
    std::vector<std::shared_ptr<KalmanFilter>> kalman_filters_ptrs_l_;
    std::vector<std::shared_ptr<KalmanFilter>> kalman_filters_ptrs_r_;

    // teleop states
    Eigen::Vector3d hand_pose_start_l_;
    Eigen::Vector3d hand_pose_start_r_;
    Eigen::Quaterniond hand_ori_start_l_;
    Eigen::Quaterniond hand_ori_start_r_;
};

} // namespace garment_research


#endif // MMTELEOP_IMU_HPP__