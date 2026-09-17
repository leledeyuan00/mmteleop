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
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "kdl/frames.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

// tf sub
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

// custom
#include "cartesian_controller_msgs/srv/joint_move.hpp"
#include "garment_motion_base/garment_motion_base.hpp"

// low pass filter
#include "mmteleop/low_pass_filter.hpp"

namespace garment_research
{

class MmteleopQuest : public GarmentMotionBase
{
public:
    MmteleopQuest(): GarmentMotionBase("mmteleop_quest3"){};

private:
    /* function */
    virtual void custom_init();
    void tf_update();
    void emergenccy_detection();
    // tasks
    virtual void tasks_init();


    /* variable */
    // pub for monitor
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr haptics_pub_l_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr haptics_pub_r_;

    // srvs
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_srv_;
    bool teleop_start_;

    // tf   
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    tf2::BufferCore buffer_;

    // ros time
    rclcpp::Clock ros_clock_;
    rclcpp::Time start_time_;

    // body transofrm matrix
    Eigen::Isometry3d body_right_hand_quest_;
    Eigen::Isometry3d body_left_hand_quest_;


    // lowe pass filter
    std::shared_ptr<LowPassFilter> low_pass_filter_ptr_l_;
    std::shared_ptr<LowPassFilter> low_pass_filter_ptr_r_;

    // teleop states
    Eigen::Vector3d hand_pose_start_l_;
    Eigen::Vector3d hand_pose_start_r_;
    Eigen::Quaterniond hand_ori_start_l_;
    Eigen::Quaterniond hand_ori_start_r_;
    bool axis_mirror_{false};
    bool emergency_stop_{false};

    double move_rate_;

    uint8_t task_num_;
    uint8_t tele_quest_start_task_num_;

    double force_threshold_ = 20.0; // N

    // algorithm
    Eigen::Matrix<double, 3, 2> boundary_limit_l_;
    Eigen::Matrix<double, 3, 2> boundary_limit_r_;
};

} // namespace garment_research


#endif // MMTELEOP_IMU_HPP__