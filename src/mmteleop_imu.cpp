#include "mmteleop/mmteleop_imu.hpp"


using namespace std::chrono_literals;

namespace garment_research
{

MmteleopIMU::MmteleopIMU() : GarmentMotionBase("multi_modal_teleop_imu")
{
    custom_init();
    tasks_init();
}

// For custom initialization such as service client and parameters
void MmteleopIMU::custom_init()
{
    // imu sub
    imu_sub_l_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/left_cartesian_compliance_controller/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_msg_l_ = *msg;
            imu_ori_l_ = Eigen::Quaterniond(imu_msg_l_.orientation.w, imu_msg_l_.orientation.x, imu_msg_l_.orientation.y, imu_msg_l_.orientation.z);
            Eigen::Vector3d imu_acc = Eigen::Vector3d(imu_msg_l_.linear_acceleration.x, imu_msg_l_.linear_acceleration.y, imu_msg_l_.linear_acceleration.z);
            Eigen::Vector3d g(0, 0, 0.981);
            imu_acc_l_ = -10*(imu_ori_l_.inverse() * imu_acc + g);
            if (!imu_received_l_){
                RCLCPP_INFO(this->get_logger(), "IMU received");
                imu_received_l_ = true;
            }
        });
    
    imu_sub_r_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/right_cartesian_compliance_controller/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_msg_r_ = *msg;
            imu_ori_r_ = Eigen::Quaterniond(imu_msg_r_.orientation.w, imu_msg_r_.orientation.x, imu_msg_r_.orientation.y, imu_msg_r_.orientation.z);
            Eigen::Vector3d imu_acc = Eigen::Vector3d(imu_msg_r_.linear_acceleration.x, imu_msg_r_.linear_acceleration.y, imu_msg_r_.linear_acceleration.z);
            Eigen::Vector3d g(0, 0, 0.981);
            imu_acc_r_ = -10*(imu_ori_r_.inverse() * imu_acc + g);
            if (!imu_received_r_){
                RCLCPP_INFO(this->get_logger(), "IMU received");
                imu_received_r_ = true;
            }
        });

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

    // Initialize Kalman
    // Kalman filter for calculating cartesian velocities
    double dt = 0.004; // 4 ms
    // std::shared_ptr<KalmanFilter> kalman_filter_ptr;
    kalman_filters_ptrs_l_.resize(3);
    kalman_filters_ptrs_r_.resize(3);
    Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d transition_matrix = (Eigen::Matrix3d() << 1, dt, 0, 0, 1, dt, 0, 0, 1).finished();
    Eigen::Matrix3d observation_matrix = (Eigen::Matrix3d() << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished();
    Eigen::Matrix3d process_noise = (Eigen::Matrix3d() << pow(dt,4)/4, pow(dt,3)/2, pow(dt,2)/2, pow(dt,3)/2, pow(dt,2), dt, pow(dt,2)/2, dt, 1).finished();
    Eigen::Matrix3d measurement_noise = (Eigen::Matrix3d() << 0.08*0.08, 0, 0, 0, 1000, 0, 0, 0, 100).finished();
    for (size_t i = 0; i < kalman_filters_ptrs_l_.size(); ++i)
    {
        kalman_filters_ptrs_l_[i].reset(new KalmanFilter(initial_covariance, transition_matrix, observation_matrix, process_noise, measurement_noise));
        kalman_filters_ptrs_r_[i].reset(new KalmanFilter(initial_covariance, transition_matrix, observation_matrix, process_noise, measurement_noise));
    }
}

void MmteleopIMU::tf_update()
{
    // get robot pose
    geometry_msgs::msg::TransformStamped right_hand_transform;
    geometry_msgs::msg::TransformStamped left_hand_transform;
    geometry_msgs::msg::TransformStamped neck_transform;
    std::string robot_frame = "robot_base_link";

    try
    {
        // body
        neck_transform = buffer_.lookupTransform(robot_frame, "body_base_link", tf2::TimePointZero);
        right_hand_transform = buffer_.lookupTransform(robot_frame, "right_hand_link", tf2::TimePointZero);
        left_hand_transform = buffer_.lookupTransform(robot_frame, "left_hand_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    // convert to eigen
    body_neck_ = tf2::transformToEigen(neck_transform);
    body_right_hand_ = tf2::transformToEigen(right_hand_transform);
    body_left_hand_ = tf2::transformToEigen(left_hand_transform);
}



void MmteleopIMU::tasks_init()
{

    // Go Home
    task_pushback(TaskPtr("Go Home", [this](){
        std::vector<double> left_home_joints = {-0.26677, -0.453412, 2.06686, -0.2983, -0.64860, -1.184497};
        std::vector<double> right_home_joints = {0.07035, -0.4897, 2.0653009, 0.114585, -0.54722, 1.42553};

        if(joint_move(left_home_joints, right_home_joints, 5.0)){
            set_task_finished();
        }
    }));

    // Initial tf and waiting teleop service
    task_pushback(TaskPtr("Initial tf and waiting teleop service",[this](){
        tf_update();
        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f], [%f, %f, %f]", imu_acc_l_(0), imu_acc_l_(1), imu_acc_l_(2),body_left_hand_.translation().x(), body_left_hand_.translation().y(), body_left_hand_.translation().z());
        // print the left orientation
        RCLCPP_INFO(this->get_logger(), "Left orientation is: [%f, %f, %f, %f]", imu_ori_l_.x(), imu_ori_l_.y(), imu_ori_l_.z(), imu_ori_l_.w());


        if (teleop_start_ && imu_received_l_ && imu_received_r_)
        {
            set_task_finished();
        }
    }));

    // Start Teleop
    task_pushback(TaskPtr("Start Teleop", [this](){
        tf_update();
        // Initialize the start position
        hand_pose_start_l_ = Eigen::Vector3d(body_left_hand_.translation().x(), body_left_hand_.translation().y(), body_left_hand_.translation().z());
        hand_pose_start_r_ = Eigen::Vector3d(body_right_hand_.translation().x(), body_right_hand_.translation().y(), body_right_hand_.translation().z());
        hand_ori_start_l_ = imu_ori_l_;
        hand_ori_start_r_ = imu_ori_r_;

        // Initialize the kalman filter
        // Left kalman filter
        kalman_filters_ptrs_l_[0]->set_initial_state(Eigen::Vector3d(body_left_hand_.translation().x(), 0, imu_acc_l_(0)));
        kalman_filters_ptrs_l_[1]->set_initial_state(Eigen::Vector3d(body_left_hand_.translation().y(), 0, imu_acc_l_(1)));
        kalman_filters_ptrs_l_[2]->set_initial_state(Eigen::Vector3d(body_left_hand_.translation().z(), 0, imu_acc_l_(2)));
        // right kalman filter
        kalman_filters_ptrs_r_[0]->set_initial_state(Eigen::Vector3d(body_right_hand_.translation().x(), 0, imu_acc_r_(0)));
        kalman_filters_ptrs_r_[1]->set_initial_state(Eigen::Vector3d(body_right_hand_.translation().y(), 0, imu_acc_r_(1)));
        kalman_filters_ptrs_r_[2]->set_initial_state(Eigen::Vector3d(body_right_hand_.translation().z(), 0, imu_acc_r_(2)));
    },
    [this](){
        auto robot_l = get_robot_state_l();
        auto robot_r = get_robot_state_r();
        // Update tf
        tf_update();

        // Left
        // Update kalman filter
        Eigen::Vector3d hand_filtered_l_x = kalman_filters_ptrs_l_[0]->update(imu_acc_l_(0), body_left_hand_.translation().x());
        Eigen::Vector3d hand_filtered_l_y = kalman_filters_ptrs_l_[1]->update(imu_acc_l_(1), body_left_hand_.translation().y());
        Eigen::Vector3d hand_filtered_l_z = kalman_filters_ptrs_l_[2]->update(imu_acc_l_(2), body_left_hand_.translation().z());

        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f], [%f, %f, %f]", imu_acc_l_(0), imu_acc_l_(1), imu_acc_l_(2),body_left_hand_.translation().x(), body_left_hand_.translation().y(), body_left_hand_.translation().z());


        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_l = robot_l.start_pose;
        Eigen::Quaterniond robot_ori_start_l = Eigen::Quaterniond(start_pose_l.pose.orientation.w, start_pose_l.pose.orientation.x, start_pose_l.pose.orientation.y, start_pose_l.pose.orientation.z);
        Eigen::Vector3d hand_pose_filtered_l = Eigen::Vector3d(hand_filtered_l_x(0), hand_filtered_l_y(0), hand_filtered_l_z(0));

        // Calculate the target pose
        Eigen::Quaterniond ori_inc_l =  hand_ori_start_l_.inverse() * imu_ori_l_;
        Eigen::Quaterniond ori_inc_trans_l =  y_axis_mirror_ ?  Eigen::Quaterniond(ori_inc_l.w(), ori_inc_l.x(), -ori_inc_l.y(), ori_inc_l.z()) : ori_inc_l; // This is a trick to convert the orientation from the right hand to the left hand rotation
        Eigen::Quaterniond robot_ori_target_l =  ori_inc_trans_l * robot_ori_start_l;
        geometry_msgs::msg::PoseStamped target_pose_l = robot_l.start_pose;
        target_pose_l.pose.position.x = start_pose_l.pose.position.x  + (hand_pose_filtered_l(0) - hand_pose_start_l_(0)) * 0.8;
        target_pose_l.pose.position.y = start_pose_l.pose.position.y  + (hand_pose_filtered_l(1) - hand_pose_start_l_(1)) * 0.8;
        target_pose_l.pose.position.z = start_pose_l.pose.position.z  + (hand_pose_filtered_l(2) - hand_pose_start_l_(2)) * 0.8;
        // RCLCPP_INFO(this->get_logger(), "Target pose is: [%f, %f, %f]", target_pose_l.pose.position.x, target_pose_l.pose.position.y, target_pose_l.pose.position.z);
        target_pose_l.pose.orientation.x = robot_ori_target_l.x();
        target_pose_l.pose.orientation.y = robot_ori_target_l.y();
        target_pose_l.pose.orientation.z = robot_ori_target_l.z();
        target_pose_l.pose.orientation.w = robot_ori_target_l.w();

        set_target_pose_l(target_pose_l);

        // Right
        // Update kalman filter
        Eigen::Vector3d hand_filtered_r_x = kalman_filters_ptrs_r_[0]->update(imu_acc_r_(0), body_right_hand_.translation().x());
        Eigen::Vector3d hand_filtered_r_y = kalman_filters_ptrs_r_[1]->update(imu_acc_r_(1), body_right_hand_.translation().y());
        Eigen::Vector3d hand_filtered_r_z = kalman_filters_ptrs_r_[2]->update(imu_acc_r_(2), body_right_hand_.translation().z());

        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_r = robot_r.start_pose;
        Eigen::Quaterniond robot_ori_start_r = Eigen::Quaterniond(start_pose_r.pose.orientation.w, start_pose_r.pose.orientation.x, start_pose_r.pose.orientation.y, start_pose_r.pose.orientation.z);
        Eigen::Vector3d hand_pose_filtered_r = Eigen::Vector3d(hand_filtered_r_x(0), hand_filtered_r_y(0), hand_filtered_r_z(0));
        
        // Calculate the target pose
        Eigen::Quaterniond ori_inc_r =  hand_ori_start_r_.inverse() * imu_ori_r_;
        Eigen::Quaterniond ori_inc_trans_r = y_axis_mirror_ ? Eigen::Quaterniond(ori_inc_r.w(), ori_inc_r.x(), -ori_inc_r.y(), ori_inc_r.z()) : ori_inc_r; // This is a trick to convert the orientation from the right hand to the left hand rotation
        Eigen::Quaterniond robot_ori_target_r =  ori_inc_trans_r * robot_ori_start_r;
        geometry_msgs::msg::PoseStamped target_pose_r = robot_r.start_pose;
        target_pose_r.pose.position.x = start_pose_r.pose.position.x  + (hand_pose_filtered_r(0) - hand_pose_start_r_(0)) * 0.8;
        target_pose_r.pose.position.y = start_pose_r.pose.position.y  + (hand_pose_filtered_r(1) - hand_pose_start_r_(1)) * 0.8;
        target_pose_r.pose.position.z = start_pose_r.pose.position.z  + (hand_pose_filtered_r(2) - hand_pose_start_r_(2)) * 0.8;
        // RCLCPP_INFO(this->get_logger(), "Target pose is: [%f, %f, %f]", target_pose_r.pose.position.x, target_pose_r.pose.position.y, target_pose_r.pose.position.z);
        target_pose_r.pose.orientation.x = robot_ori_target_r.x();
        target_pose_r.pose.orientation.y = robot_ori_target_r.y();
        target_pose_r.pose.orientation.z = robot_ori_target_r.z();
        target_pose_r.pose.orientation.w = robot_ori_target_r.w();

        set_target_pose_r(target_pose_r);
        
        if (!teleop_start_)
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
    auto node = std::make_shared<garment_research::MmteleopIMU>();

    node->start();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "Garment Motion Test by FSM Node Stopped");
    return 0;
}
