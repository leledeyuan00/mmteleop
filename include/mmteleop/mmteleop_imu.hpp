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
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <sensor_msgs/msg/imu.hpp>
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
#include "mmteleop/kalman_filter.hpp"

// low pass filter
#include "mmteleop/low_pass_filter.hpp"

enum MARKER_ID: uint8_t
{
    RIGHT_HAND = 0,
    LEFT_HAND = 1,
    NECK_UPPER = 2,
    NECK_LEFT = 3,
    NECK_RIGHT = 4
};

namespace garment_research
{

struct PointData
{
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
};
class MmteleopIMU : public GarmentMotionBase
{
public:
    MmteleopIMU(): GarmentMotionBase("multi_modal_teleop_imu"){};

private:
    /* function */
    virtual void custom_init();
    void tf_update();
    void emergenccy_detection();
    void record_data_init();
    void read_data_init();
    // tasks
    void test_service(bool on);
    virtual void tasks_init();


    /* variable */
    // sub
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_l_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_r_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr body_tracking_status_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr confidence_sub_;

    // pub for monitor
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr monitor_pub_;

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
    Eigen::Isometry3d body_neck_, body_neck_start_;
    Eigen::Isometry3d body_right_hand_;
    Eigen::Isometry3d body_left_hand_;

    // IMU
    std::vector<Eigen::Vector3d> imu_acc_l_buffer_;
    std::vector<Eigen::Vector3d> imu_acc_r_buffer_;
    std::vector<Eigen::Quaterniond> imu_ori_l_buffer_;
    std::vector<Eigen::Quaterniond> imu_ori_r_buffer_;

    // kalman filter
    std::shared_ptr<KalmanFilter> kalman_filter_ptr_l_;
    std::shared_ptr<KalmanFilter> kalman_filter_ptr_r_;

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
    bool new_tracking_data_{false};
    bool new_imu_data_l_{false};
    bool new_imu_data_r_{false};
    Vector12d state_l_;
    Vector12d state_r_;

    double move_rate_;

    uint8_t task_num_;
    uint8_t tele_start_task_num_;

    std::vector<uint8_t> marker_confidences_;

    // recording data to vector buffer
    std::vector<PointData> recorded_trj_l_;
    std::vector<PointData> recorded_trj_r_;

    // read data from vector buffer
    uint32_t recorded_trj_idx_ = 0;
    double linear_int_count_ = 0;

    // recording the data to txt file
    bool start_record_data_;
    std::ofstream data_file_;
    std::string filename_;
    std::ifstream data_file_in_;    

    // algorithm
    Eigen::Matrix<double, 3, 2> boundary_limit_l_;
    Eigen::Matrix<double, 3, 2> boundary_limit_r_;
    // Eigen::Vector3d boundary_left_corner_;
    // Eigen::Vector3d boundary_right_corner_;
    
};

} // namespace garment_research


#endif // MMTELEOP_IMU_HPP__