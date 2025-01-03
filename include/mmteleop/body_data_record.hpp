#ifndef _BODY_DATA_RECORD__
#define _BODY_DATA_RECORD__

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
#include <sensor_msgs/msg/imu.hpp>

// tf sub
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// low pass filter
#include <control_toolbox/control_toolbox/filters.hpp>

// eigen
#include <eigen3/Eigen/Dense>

// std srv
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

class body_record : public rclcpp::Node
{

public:
    body_record();

private:
    // ros
    void ros_init();
    void record_data_init(void);

    void robot_update();
    void loop();

    void start_record_cb(const std_srvs::srv::SetBool::Request::SharedPtr request, 
                        std_srvs::srv::SetBool::Response::SharedPtr response);
    


    // ros time
    rclcpp::Clock ros_clock_;
    rclcpp::Time start_time_;
    rclcpp::Time last_time_;

    // sub
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // srv
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_record_srv_;


    // tf   
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    tf2::BufferCore buffer_;

    // std
    std::string robot_frame_;
    std::thread control_loop_thread_;


    // robot transform matrix
    sensor_msgs::msg::Imu imu_msg_;


    Eigen::Isometry3d right_current_pose_;
    Eigen::Isometry3d left_current_pose_;

    Eigen::Vector3d right_start_wrench_;
    Eigen::Vector3d left_start_wrench_;

    // body transofrm matrix
    Eigen::Isometry3d body_neck_;
    Eigen::Isometry3d body_right_hand_;
    Eigen::Isometry3d body_left_hand_;

    // recording the data to txt file
    bool start_record_data_;
    std::ofstream data_file_;    
};



#endif