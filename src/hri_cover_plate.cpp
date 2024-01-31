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
    
    left_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/left_cartesian_compliance_controller/current_wrench", rclcpp::SystemDefaultsQoS(),
        [&](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            left_wrench_msg_ = *msg;
        }
    );

    right_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/right_cartesian_compliance_controller/current_wrench", rclcpp::SystemDefaultsQoS(),
        [&](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            right_wrench_msg_ = *msg;
        }
    );

    // init variables
    right_target_pose_msg_.header.frame_id = robot_frame_;
    left_target_pose_msg_.header.frame_id = robot_frame_;

    right_current_pose_.setIdentity();
    left_current_pose_.setIdentity();

    right_start_wrench_ << 0, 0, 0;
    left_start_wrench_ << 0, 0, 0;

    ros_clock_ = rclcpp::Clock(RCL_ROS_TIME); 

    state_ = STATE::FAST;
    slow2fast_count_ = 0;

    // for recording
    record_data_init();

    // thread
    control_loop_thread_ = std::thread(&hri_safety::loop, this);
}

void hri_safety::record_data_init()
{
    // record data
    std::stringstream filename;
    std::time_t now = std::time(NULL);
    std::tm *lt = std::localtime(&now);
    char* home_dir = getenv("HOME");
    filename << home_dir <<"/Documents/log/hri_safety/" << lt->tm_mon +1 << "-" << lt->tm_mday << "-" << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".txt";
    data_file_.open(filename.str());
    if(!data_file_) std::cout<<"error"<<std::endl;

    data_file_ << "time STATE distance right_x right_y right_z left_x left_y left_z right_fx right_fy right_fz left_fx left_fy left_fz" << std::endl;
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

void hri_safety::emergency_check()
{
    Eigen::Vector3d right_wrench = Eigen::Vector3d(right_wrench_msg_.wrench.force.x, right_wrench_msg_.wrench.force.y, right_wrench_msg_.wrench.force.z) - right_start_wrench_;
    Eigen::Vector3d left_wrench = Eigen::Vector3d(left_wrench_msg_.wrench.force.x, left_wrench_msg_.wrench.force.y, left_wrench_msg_.wrench.force.z) - left_start_wrench_;

    double emergency_stop_force = 20.0;

    if (right_wrench.norm() > emergency_stop_force || left_wrench.norm() > emergency_stop_force)
    {
        state_ = STATE::STOP;
        RCLCPP_INFO(this->get_logger(),"Emergency stop");
    }
}

void hri_safety::loop()
{
    rclcpp::Rate loop_rate(100);

    double fast_moving_speed = 0.1; // m/s
    double slow_moving_speed = 0.001; // m/s
    double current_speed = fast_moving_speed;
    double target_x = 0.88;

    last_time_ = ros_clock_.now();

    while (rclcpp::ok())
    {
        robot_update();
        emergency_check();

        double current_duration = (ros_clock_.now() - last_time_).seconds();

        // calculate the distance between robot right hand and body right hand
        Eigen::Vector3d right_hand_distance = body_right_hand_.translation() - right_current_pose_.translation();
        Eigen::Vector3d left_hand_distance = body_left_hand_.translation() - right_current_pose_.translation();
        
        double hand_distance = std::min(right_hand_distance.norm(), left_hand_distance.norm());

        // Get current wrench
        Eigen::Vector3d right_wrench = Eigen::Vector3d(right_wrench_msg_.wrench.force.x, right_wrench_msg_.wrench.force.y, right_wrench_msg_.wrench.force.z) - right_start_wrench_;
        Eigen::Vector3d left_wrench = Eigen::Vector3d(left_wrench_msg_.wrench.force.x, left_wrench_msg_.wrench.force.y, left_wrench_msg_.wrench.force.z) - left_start_wrench_;

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
                    if (hand_distance > 0.3)
                    {
                        slow2fast_count_++;
                        if (slow2fast_count_ > 10)
                        {
                            slow2fast_count_ = 0;
                            state_ = STATE::FAST;
                            current_speed = fast_moving_speed;
                            RCLCPP_INFO(this->get_logger(),"Switch to fast mode");
                            break;
                        }
                    }

                    if (right_wrench.norm() > 5. || left_wrench.norm() > 5.)
                    {
                        state_ = STATE::DRAGGING;
                        RCLCPP_INFO(this->get_logger(),"Switch to dragging mode");
                    }
                    break;
                }
            case STATE::FAST:
                {
                    if (hand_distance < 0.2)
                    {
                        state_ = STATE::SLOW;
                        current_speed = slow_moving_speed;
                        right_start_wrench_ << 0.0, 0.0, right_wrench_msg_.wrench.force.z;
                        left_start_wrench_ << 0.0, 0.0, left_wrench_msg_.wrench.force.z;
                        RCLCPP_INFO(this->get_logger(),"Switch to slow mode");
                    }
                    break;
                }
            case STATE::DRAGGING:
                {
                    if (hand_distance >= 0.15 && ( right_wrench.norm() < 5. && left_wrench.norm() < 5.))
                    {
                        state_ = STATE::SLOW;
                        current_speed = slow_moving_speed;
                        RCLCPP_INFO(this->get_logger(),"Switch to slow mode");
                    }
                    break;
                }      
            case STATE::STOP:
                {
                    break;
                }      
            default:
                break;
            }

            if (state_ != STATE::STOP)
            {
                if (state_ != STATE::DRAGGING)
                {
                    right_target_pose_msg_.pose.position.x = right_target_pose_msg_.pose.position.x + current_speed * current_duration;
                    left_target_pose_msg_.pose.position.x = left_target_pose_msg_.pose.position.x + current_speed * current_duration;
                }
                else
                {
                    right_target_pose_msg_ = right_current_pose_msg_;
                    left_target_pose_msg_ = left_current_pose_msg_;
                }

                right_target_pose_msg_.header.stamp = ros_clock_.now();
                left_target_pose_msg_.header.stamp = ros_clock_.now();

                if (right_current_pose_msg_.pose.position.x > target_x)
                {
                    RCLCPP_INFO(this->get_logger(),"Reach the target position");
                    data_file_.close();
                    break;
                }            

                right_pose_pub_->publish(right_target_pose_msg_);
                left_pose_pub_->publish(left_target_pose_msg_);

                last_time_ = ros_clock_.now();
            }
            // record data
            data_file_ << (ros_clock_.now() - start_time_).seconds() << " " << (int)(state_) << " " << hand_distance << " " << right_current_pose_msg_.pose.position.x << " " << right_current_pose_msg_.pose.position.y << " " << right_current_pose_msg_.pose.position.z << " " << left_current_pose_msg_.pose.position.x << " " << left_current_pose_msg_.pose.position.y << " " << left_current_pose_msg_.pose.position.z << " " << right_wrench_msg_.wrench.force.x << " " << right_wrench_msg_.wrench.force.y << " " << right_wrench_msg_.wrench.force.z << " " << left_wrench_msg_.wrench.force.x << " " << left_wrench_msg_.wrench.force.y << " " << left_wrench_msg_.wrench.force.z << std::endl;
            loop_rate.sleep();
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