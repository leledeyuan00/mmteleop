#include "mmteleop/hri_safety.hpp"

hri_safety::hri_safety() : Node("hri_safety")
{
    ros_init();
}

void hri_safety::ros_init()
{
    // ros
    tf_listener_.reset(new tf2_ros::TransformListener(this->buffer_));

    // parameters
    this->declare_parameter<std::string>("robot_frame", "robot_base_link");
    this->get_parameter<std::string>("robot_frame", robot_frame_);
    
    // pub
    monitor_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hri_safety/monitor", rclcpp::SystemDefaultsQoS());
    right_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/right_cartesian_compliance_controller/target_frame", rclcpp::SystemDefaultsQoS()
    );

    left_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/left_cartesian_compliance_controller/target_frame", rclcpp::SystemDefaultsQoS()
    );

    right_gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/R_gripper_forward_position_controller/commands", rclcpp::SystemDefaultsQoS()
    );

    left_gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/L_gripper_forward_position_controller/commands", rclcpp::SystemDefaultsQoS()
    );

    // sub
    right_current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right_cartesian_compliance_controller/current_pose", rclcpp::SystemDefaultsQoS(),
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            right_current_pose_msg_ = *msg;

            if (!initialized_r_)
            {
                right_target_pose_msg_ = right_current_pose_msg_;
                right_start_pose_msg_ = right_current_pose_msg_;
                initialized_r_ = true;
                start_time_ = ros_clock_.now();
            }
        }
    );

    left_current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left_cartesian_compliance_controller/current_pose", rclcpp::SystemDefaultsQoS(),
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            left_current_pose_msg_ = *msg;
            
            if (!initialized_l_)
            {
                left_target_pose_msg_ = left_current_pose_msg_;
                left_start_pose_msg_ = left_current_pose_msg_;
                initialized_l_ = true;
            }
        }
    );

    // init variables
    right_target_pose_msg_.header.frame_id = robot_frame_;
    left_target_pose_msg_.header.frame_id = robot_frame_;

    right_current_pose_.setIdentity();
    left_current_pose_.setIdentity();

    ros_clock_ = rclcpp::Clock(RCL_ROS_TIME); 

    state_ = STATE::AUTO;
    slow2fast_count_ = 0;

    // thread
    control_loop_thread_ = std::thread(&hri_safety::loop, this);
}

void hri_safety::robot_update()
{
    // get robot pose
    geometry_msgs::msg::TransformStamped right_current_pose;
    geometry_msgs::msg::TransformStamped left_current_pose;
    geometry_msgs::msg::TransformStamped right_hand_transform;
    geometry_msgs::msg::TransformStamped left_hand_transform;
    geometry_msgs::msg::TransformStamped neck_transform;

    try
    {
        // robot
        right_current_pose = buffer_.lookupTransform(robot_frame_, "R_end_link", tf2::TimePointZero);
        left_current_pose = buffer_.lookupTransform(robot_frame_, "L_end_link", tf2::TimePointZero);
        // body
        neck_transform = buffer_.lookupTransform(robot_frame_, "body_base_link", tf2::TimePointZero);
        right_hand_transform = buffer_.lookupTransform(robot_frame_, "right_hand_link", tf2::TimePointZero);
        left_hand_transform = buffer_.lookupTransform(robot_frame_, "left_hand_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    // convert to eigen
    right_current_pose_ = tf2::transformToEigen(right_current_pose);
    left_current_pose_ = tf2::transformToEigen(left_current_pose);
    body_neck_ = tf2::transformToEigen(neck_transform);
    body_right_hand_ = tf2::transformToEigen(right_hand_transform);
    body_left_hand_ = tf2::transformToEigen(left_hand_transform);
}

void hri_safety::loop()
{
    rclcpp::Rate loop_rate(100);

    double fast_duration = 2.0;
    double slow_duration = 40.0;
    double moving_distance = 0.1;
    double moving_phi = 0;
    double current_duration = slow_duration;
    double moving_phase = 0;

    while (rclcpp::ok())
    {
        robot_update();

        // calculate the distance between robot right hand and body right hand
        Eigen::Vector3d right_hand_distance = body_right_hand_.translation() - right_current_pose_.translation();
        Eigen::Vector3d left_hand_distance = body_left_hand_.translation() - right_current_pose_.translation();
        
        double hand_distance = std::min(right_hand_distance.norm(), left_hand_distance.norm());


        // show the distance
        std_msgs::msg::Float64MultiArray monitor_msg;
        monitor_msg.data.push_back(hand_distance);
        monitor_->publish(monitor_msg);

        if (initialized_l_ && initialized_r_)
        {
            switch (state_)
            {
            case STATE::SLOW:
                {
                    if (hand_distance > 0.12)
                    {
                        slow2fast_count_++;
                        if (slow2fast_count_ > 5)
                        {
                            slow2fast_count_ = 0;
                            state_ = STATE::AUTO;
                            current_duration = fast_duration;
                            moving_phi = moving_phase;
                            start_time_ = ros_clock_.now();

                            double close_gripper = 0.78;
                            std_msgs::msg::Float64MultiArray right_gripper_msg;
                            right_gripper_msg.data.push_back(close_gripper);
                            right_gripper_pub_->publish(right_gripper_msg);
                        }
                    }
                    break;
                }
            case STATE::AUTO:
                {
                    if (hand_distance < 0.1)
                    {
                        state_ = STATE::SLOW;
                        current_duration = slow_duration;
                        moving_phi = moving_phase;
                        start_time_ = ros_clock_.now();

                        double open_gripper = 0.0;
                        std_msgs::msg::Float64MultiArray right_gripper_msg;
                        right_gripper_msg.data.push_back(open_gripper);
                        right_gripper_pub_->publish(right_gripper_msg);
                    }
                    break;
                }            
            default:
                break;
            }

            double current_time = (ros_clock_.now() - start_time_).seconds();
            moving_phase = 2 * M_PI * current_time / current_duration + moving_phi;

            right_target_pose_msg_.header.stamp = ros_clock_.now();
            right_target_pose_msg_.pose.position.y = right_start_pose_msg_.pose.position.y + moving_distance * sin(moving_phase);

            right_pose_pub_->publish(right_target_pose_msg_);
        }
    }
}

int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("hri_safety"), "hri_safety Started");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<hri_safety>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}