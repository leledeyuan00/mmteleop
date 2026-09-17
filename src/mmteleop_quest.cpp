#include "mmteleop/mmteleop_quest.hpp" 
// OC Demo 2025

using namespace std::chrono_literals;

namespace garment_research
{

// For custom initialization such as service client and parameters
void MmteleopQuest::custom_init()
{

    boundary_limit_l_   << 0.3, 0.8,
                           -0.6, 0.3,
                           -1.2, -0.4;

    boundary_limit_r_   << 0.3, 0.8,
                           -0.3, 0.6,
                           -1.2, -0.4;

    // pub for haptics
    haptics_pub_l_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/L_haptic_amplitude", rclcpp::SystemDefaultsQoS()
    );

    haptics_pub_r_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/R_haptic_amplitude", rclcpp::SystemDefaultsQoS()
    );

    // service
    teleop_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/teleop_start", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                std_srvs::srv::SetBool::Response::SharedPtr response) {
            teleop_start_ = request->data;
            response->success = true;
            response->message = "Teleop service is " + std::string(teleop_start_ ? "on" : "off");
            if (teleop_start_)
            {
                RCLCPP_INFO(this->get_logger(), "Teleop service is trying to start");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Teleop service is trying to stop");
            }
            
        });

    // Initialize tf
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);  
}


void MmteleopQuest::tf_update()
{
    // get robot pose
    geometry_msgs::msg::TransformStamped right_hand_transform, right_hand_transform_quest;
    geometry_msgs::msg::TransformStamped left_hand_transform,  left_hand_transform_quest;
    geometry_msgs::msg::TransformStamped neck_transform;
    std::string robot_frame = "robot_base_link";

    try
    {
        // quest
        right_hand_transform_quest = buffer_.lookupTransform(robot_frame, "oculus_r", tf2::TimePointZero);
        left_hand_transform_quest  = buffer_.lookupTransform(robot_frame, "oculus_l", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    // convert to eigen
    body_right_hand_quest_ = tf2::transformToEigen(right_hand_transform_quest);
    body_left_hand_quest_  = tf2::transformToEigen(left_hand_transform_quest);
}

void MmteleopQuest::emergenccy_detection()
{
    auto robot_l = get_robot_state_l();
    auto robot_r = get_robot_state_r();
    Eigen::Vector3d left_wrench_force = Eigen::Vector3d(robot_l.current_wrench.wrench.force.x, robot_l.current_wrench.wrench.force.y, robot_l.current_wrench.wrench.force.z);
    Eigen::Vector3d right_wrench_force = Eigen::Vector3d(robot_r.current_wrench.wrench.force.x, robot_r.current_wrench.wrench.force.y, robot_r.current_wrench.wrench.force.z);

    if (left_wrench_force.norm() > force_threshold_ || right_wrench_force.norm() > force_threshold_)
    {
        RCLCPP_ERROR(this->get_logger(), "Emergency detected, current force is: [%f, %f]", left_wrench_force.norm(), right_wrench_force.norm());
        emergency_stop_ = true;
    }
}

void MmteleopQuest::tasks_init()
{

    // Go Home
    task_pushback(TaskPtr("Go Home", [this](){

        std::vector<double> left_home_joints = {-0.259797, -0.180369, 2.012831, -1.642967, 1.319282, 3.416804};
        std::vector<double> right_home_joints = {0.059437, -0.132879, 1.972856, 1.583187, 1.507452, 2.896193};

        if(joint_move(left_home_joints, right_home_joints, 5.0)){
            set_task_finished();
        }
    }));

    // Waiting until the start button is pressed
    task_pushback(TaskPtr("Waiting until the start button is pressed", [this](){
        if (teleop_start_)
        {
            goto_specific_task(tele_quest_start_task_num_);
        }
    }));

    // Teleop with Quest only
    tele_quest_start_task_num_ = task_pushback(TaskPtr("Start Teleop with Quest", [this](){
        tf_update();
        // Initialize the start position
        hand_pose_start_l_ = body_left_hand_quest_.translation();
        RCLCPP_INFO(this->get_logger(), "Left hand start pose: [%f, %f, %f]", hand_pose_start_l_(0), hand_pose_start_l_(1), hand_pose_start_l_(2));
        hand_pose_start_r_ = body_right_hand_quest_.translation();
        hand_ori_start_l_ = Eigen::Quaterniond(body_left_hand_quest_.rotation());
        hand_ori_start_r_ = Eigen::Quaterniond(body_right_hand_quest_.rotation());

        // low pass filter
        Eigen::Vector3d alpha(0.5, 0.5, 0.5);
        low_pass_filter_ptr_l_.reset(new LowPassFilter(alpha));
        low_pass_filter_ptr_r_.reset(new LowPassFilter(alpha));
        
    },
    [this](){
        auto robot_l = get_robot_state_l();
        auto robot_r = get_robot_state_r();
        double alpha_ori = 0.5;
        geometry_msgs::msg::PoseStamped current_pose_l = robot_l.current_pose;
        geometry_msgs::msg::PoseStamped current_pose_r = robot_r.current_pose;
        Eigen::Vector3d zero_3d = Eigen::Vector3d::Zero();
        // Update tf
        tf_update();

        // Move Rate
        double elapsed_time = get_system_state().current_time.seconds() + 
                            get_system_state().current_time.nanoseconds() * 1e-9 -
                            get_system_state().start_time.seconds() - 
                            get_system_state().start_time.nanoseconds() * 1e-9;

        move_rate_ = (elapsed_time < 1.0) ? (elapsed_time / 1.0) : 1.0;

        // Left
        // Update kalman filter
        Eigen::Vector3d current_left_hand = body_left_hand_quest_.translation();
        Eigen::Quaterniond current_left_hand_ori = Eigen::Quaterniond(body_left_hand_quest_.rotation());

        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_l = robot_l.start_pose;
        Eigen::Quaterniond robot_ori_start_l = Eigen::Quaterniond(start_pose_l.pose.orientation.w, start_pose_l.pose.orientation.x, start_pose_l.pose.orientation.y, start_pose_l.pose.orientation.z);
        
        /// limit the hand pose for safety before low pass filter -- Dayuan 0730
        // calculate current allowed increased position range
        Eigen::Matrix<double, 3, 2> current_boundary_limit_l = boundary_limit_l_ 
                                                                - Eigen::Vector3d(start_pose_l.pose.position.x, start_pose_l.pose.position.y, start_pose_l.pose.position.z).replicate(1, 2)
                                                                + hand_pose_start_l_.replicate(1, 2);
        Eigen::Vector3d boundaried_hand_pose_filtered_l = current_left_hand.cwiseMax(current_boundary_limit_l.col(0)).cwiseMin(current_boundary_limit_l.col(1));

        // low pass filter
        Eigen::Vector3d hand_pose_filtered_l = low_pass_filter_ptr_l_->update(boundaried_hand_pose_filtered_l);

        // Calculate the target pose
        Eigen::Quaterniond ori_inc_l =  hand_ori_start_l_.inverse() * current_left_hand_ori;
        Eigen::Quaterniond ori_inc_trans_l =  axis_mirror_ ?  Eigen::Quaterniond(ori_inc_l.w(), ori_inc_l.x(), -ori_inc_l.y(), -ori_inc_l.z()) : ori_inc_l; // This is a trick to convert the orientation from the right hand to the left hand rotation
        // get current orientation for a low pass filter quaternion
        Eigen::Quaterniond current_ori_l(current_pose_l.pose.orientation.w,current_pose_l.pose.orientation.x, current_pose_l.pose.orientation.y, current_pose_l.pose.orientation.z);
        Eigen::Quaterniond robot_ori_des_l = ori_inc_trans_l * robot_ori_start_l;
        Eigen::Quaterniond robot_ori_target_l = current_ori_l.slerp(alpha_ori, robot_ori_des_l); // 0.2 is the interpolation factor, you can adjust it to make the robot orientation more smooth
        
        geometry_msgs::msg::PoseStamped target_pose_l = robot_l.start_pose;
        Eigen::Vector3d position_inc_l = hand_pose_filtered_l - hand_pose_start_l_;

        target_pose_l.header.stamp = this->now();
        target_pose_l.pose.position.x = start_pose_l.pose.position.x  + position_inc_l(0) * move_rate_;
        target_pose_l.pose.position.y = start_pose_l.pose.position.y  + position_inc_l(1) * move_rate_;
        target_pose_l.pose.position.z = start_pose_l.pose.position.z  + position_inc_l(2) * move_rate_;

        target_pose_l.pose.orientation.x = robot_ori_target_l.x();
        target_pose_l.pose.orientation.y = robot_ori_target_l.y();
        target_pose_l.pose.orientation.z = robot_ori_target_l.z();
        target_pose_l.pose.orientation.w = robot_ori_target_l.w();
        


        // Right
        // Update kalman filter
        Eigen::Vector3d current_right_hand = body_right_hand_quest_.translation();
        Eigen::Quaterniond current_right_hand_ori = Eigen::Quaterniond(body_right_hand_quest_.rotation());

        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_r = robot_r.start_pose;
        Eigen::Quaterniond robot_ori_start_r = Eigen::Quaterniond(start_pose_r.pose.orientation.w, start_pose_r.pose.orientation.x, start_pose_r.pose.orientation.y, start_pose_r.pose.orientation.z);
        
        /// limit the hand pose for safety before low pass filter -- Dayuan 0730
        // calculate current allowed increased position range
        Eigen::Matrix<double, 3, 2> current_boundary_rimit_r = boundary_limit_r_ 
                                                                - Eigen::Vector3d(start_pose_r.pose.position.x, start_pose_r.pose.position.y, start_pose_r.pose.position.z).replicate(1, 2)
                                                                + hand_pose_start_r_.replicate(1, 2);
        Eigen::Vector3d boundaried_hand_pose_filtered_r = current_right_hand.cwiseMax(current_boundary_rimit_r.col(0)).cwiseMin(current_boundary_rimit_r.col(1));

        // low pass filter
        Eigen::Vector3d hand_pose_filtered_r = low_pass_filter_ptr_r_->update(boundaried_hand_pose_filtered_r);

        
        // RCLCPP_INFO(this->get_rogger(), "hand_pose_filtered_r: [%f, %f, %f]", hand_pose_filtered_r(0), hand_pose_filtered_r(1), hand_pose_filtered_r(2));
        // Eigen::Vector3d hand_pose_filtered_r = low_pass_filter_ptr_r_->update(current_right_hand);

        // Calculate the target pose
        Eigen::Quaterniond ori_inc_r =  hand_ori_start_r_.inverse() * current_right_hand_ori;
        Eigen::Quaterniond ori_inc_trans_r =  axis_mirror_ ?  Eigen::Quaterniond(ori_inc_r.w(), ori_inc_r.x(), -ori_inc_r.y(), -ori_inc_r.z()) : ori_inc_r; // This is a trick to convert the orientation from the right hand to the right hand rotation
        // get current orientation for a low pass filter quaternion
        Eigen::Quaterniond current_ori_r(current_pose_r.pose.orientation.w,current_pose_r.pose.orientation.x, current_pose_r.pose.orientation.y, current_pose_r.pose.orientation.z);
        Eigen::Quaterniond robot_ori_des_r = ori_inc_trans_r * robot_ori_start_r;
        Eigen::Quaterniond robot_ori_target_r = current_ori_r.slerp(alpha_ori, robot_ori_des_r); // 0.2 is the interpolation factor, you can adjust it to make the robot orientation more smooth
        
        geometry_msgs::msg::PoseStamped target_pose_r = robot_r.start_pose;
        Eigen::Vector3d position_inc_r = hand_pose_filtered_r - hand_pose_start_r_;

        target_pose_r.header.stamp = this->now();
        target_pose_r.pose.position.x = start_pose_r.pose.position.x  + position_inc_r(0) * move_rate_;
        target_pose_r.pose.position.y = start_pose_r.pose.position.y  + position_inc_r(1) * move_rate_;
        target_pose_r.pose.position.z = start_pose_r.pose.position.z  + position_inc_r(2) * move_rate_;

        target_pose_r.pose.orientation.x = robot_ori_target_r.x();
        target_pose_r.pose.orientation.y = robot_ori_target_r.y();
        target_pose_r.pose.orientation.z = robot_ori_target_r.z();
        target_pose_r.pose.orientation.w = robot_ori_target_r.w();

        // publish haptics feedback
        double force_l = Eigen::Vector3d(robot_l.current_wrench.wrench.force.x, robot_l.current_wrench.wrench.force.y, robot_l.current_wrench.wrench.force.z).norm();
        double haptics_l = force_l > 2.0 ? force_l/(force_threshold_ - 2.0) : 0.0;
        if (haptics_l > 1.0) haptics_l = 1.0;
        std_msgs::msg::Float64MultiArray haptics_msg_l;
        haptics_msg_l.data.push_back(haptics_l);
        haptics_pub_l_->publish(haptics_msg_l);

        double force_r = Eigen::Vector3d(robot_r.current_wrench.wrench.force.x, robot_r.current_wrench.wrench.force.y, robot_r.current_wrench.wrench.force.z).norm();
        double haptics_r = force_r > 2.0 ? force_r/(force_threshold_ - 2.0) : 0.0;
        if (haptics_r > 1.0) haptics_r = 1.0;
        std_msgs::msg::Float64MultiArray haptics_msg_r;
        haptics_msg_r.data.push_back(haptics_r);
        haptics_pub_r_->publish(haptics_msg_r);        
        
        
        // Set target pose
        emergenccy_detection();
        if (!emergency_stop_)
        {
            set_target_pose_l(target_pose_l);
            set_target_pose_r(target_pose_r);
        }        
        
        if (!teleop_start_)
        {
            data_file_.close();
            // goto_init_task();
            set_task_finished();
            emergency_stop_ = false;
        }
    }));

    // Sleep for 1 second
    task_pushback(TaskPtr("Sleep for 1.0 seconds", [this](){
        if(sleep(1.0))
        {
            goto_init_task();
        }
    }));
        


    // Log Test
    task_pushback(TaskPtr("Log Test", [this](){
        RCLCPP_INFO(this->get_logger(), "Log Test");
        auto robot_l = get_robot_state_l();
        auto robot_r = get_robot_state_r();
        auto system_state = get_system_state();

        RCLCPP_INFO(this->get_logger(), "Left current Pose is: [%f, %f, %f]", robot_l.current_pose.pose.position.x, robot_l.current_pose.pose.position.y, robot_l.current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Right current Pose is: [%f, %f, %f]", robot_r.current_pose.pose.position.x, robot_r.current_pose.pose.position.y, robot_r.current_pose.pose.position.z);

        RCLCPP_INFO(this->get_logger(), "Current system task number is: %d", system_state.task_num);
        RCLCPP_INFO(this->get_logger(), "Current system start time is: %f", system_state.start_time.seconds());
        RCLCPP_INFO(this->get_logger(), "Current system current time is: %f", system_state.current_time.seconds());

        set_task_finished();
    }));

    // // Shut down
    task_pushback(TaskPtr("Shut down", [this](){
        //initialize the task
        rclcpp::shutdown();
    }));
}


} // namespace garment_research

int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "Garment Motion Test by FSM Node Started");
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<garment_research::MmteleopQuest>();

    node->start();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "Garment Motion Test by FSM Node Stopped");
    return 0;
}
