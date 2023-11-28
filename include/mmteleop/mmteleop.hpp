#ifndef MMTELEOP_HPP__
#define MMTELEOP_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <fstream>
#include <thread>

// ros
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/Dense>

#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

// low pass filter
#include <control_toolbox/control_toolbox/filters.hpp>

const size_t MARKER_NUM = 32;


class MMTeleop : public rclcpp::Node
{
public:
    MMTeleop();
    void start();
    void loop();

private:
    /* functions */
    // ros
    void ros_init();
    void body_arrary_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    // algorithm
    void get_hand_position(std::vector<Eigen::Vector3d> body_positions, Eigen::Vector3d& right_hand_position, Eigen::Vector3d& left_hand_position);


    /* variables */
    // ros
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr body_tracking_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;


    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;

    // tf
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // robot state
    geometry_msgs::msg::PoseStamped right_current_pose_;
    geometry_msgs::msg::PoseStamped left_current_pose_;

    geometry_msgs::msg::PoseStamped right_start_pose_;
    geometry_msgs::msg::PoseStamped left_start_pose_;
    

    // std
    std::thread control_loop_thread_;

    // time
    rclcpp::Clock my_clock_;

    // Eigen
    Eigen::Vector3d right_hand_position_;
    Eigen::Vector3d left_hand_position_;

    Eigen::Vector3d right_hand_position_start_;
    Eigen::Vector3d left_hand_position_start_;

    // system
    int recorded_id_;
    std::vector<int> ids_;

    // bool
    bool tracking_ready_;
    bool initialized_r_;
    bool initialized_l_;
};


#endif