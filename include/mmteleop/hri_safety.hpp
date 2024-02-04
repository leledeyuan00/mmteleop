#ifndef _HRI_SAFETY_HPP__
#define _HRI_SAFETY_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <fstream>
#include <thread>

// ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/int16.hpp>

// tf sub
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.h>

// low pass filter
#include <control_toolbox/control_toolbox/filters.hpp>

// eigen
#include <eigen3/Eigen/Dense>

// std srv
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

enum class STATE : uint8_t
{
    AUTO = 0,
    READY = 1,
    SLOW = 2,
    DRAGGING = 3,
    STOP = 4,
};

class hri_safety : public rclcpp::Node
{

public:
    hri_safety();

private:
    // ros
    void ros_init();
    void record_data_init(void);

    void robot_update();
    void loop();
    void emergency_check();
    void state_switch(STATE &state);


    // ros time
    rclcpp::Clock ros_clock_;
    rclcpp::Time start_time_;
    rclcpp::Time last_time_;

    // pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr monitor_; // datas monitor
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_gripper_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr state_pub_;

    // sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_current_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_current_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;

    // srv
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready2slow_srv_;


    // tf   
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    tf2::BufferCore buffer_;

    // std
    std::string robot_frame_;
    std::thread control_loop_thread_;


    // robot transform matrix
    geometry_msgs::msg::PoseStamped right_current_pose_msg_;
    geometry_msgs::msg::PoseStamped left_current_pose_msg_;

    geometry_msgs::msg::PoseStamped right_start_pose_msg_;
    geometry_msgs::msg::PoseStamped left_start_pose_msg_;

    geometry_msgs::msg::PoseStamped right_target_pose_msg_;
    geometry_msgs::msg::PoseStamped left_target_pose_msg_;

    geometry_msgs::msg::WrenchStamped right_wrench_msg_;
    geometry_msgs::msg::WrenchStamped left_wrench_msg_;

    Eigen::Isometry3d right_current_pose_;
    Eigen::Isometry3d left_current_pose_;

    Eigen::Vector3d right_start_wrench_;
    Eigen::Vector3d left_start_wrench_;

    // body transofrm matrix
    Eigen::Isometry3d body_neck_;
    Eigen::Isometry3d body_right_hand_;
    Eigen::Isometry3d body_left_hand_;

    // for system states
    double min_hand_distance_;
    double max_force_;

    // bool
    bool initialized_r_;
    bool initialized_l_;

    // state
    STATE state_; // 0: slow, 1: fast
    uint8_t slow2fast_count_;

    bool ready2slow_ = false;


    // recording the data to txt file
    std::ofstream data_file_;    
};



#endif