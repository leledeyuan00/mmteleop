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
            // Eigen::Vector3d g(0, 0, 0.981);
            // imu_acc_l_ = (imu_ori_l_.inverse() * imu_acc);
            imu_acc_l_ = imu_acc;
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
            imu_acc_r_ = -1*(imu_ori_r_.inverse() * imu_acc + g);
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
    Eigen::Matrix3d measurement_noise = (Eigen::Matrix3d() << 0.08*0.08, 0, 0, 0, 1000, 0, 0, 0, 1).finished();
    for (size_t i = 0; i < kalman_filters_ptrs_l_.size(); ++i)
    {
        kalman_filters_ptrs_l_[i].reset(new KalmanFilter(initial_covariance, transition_matrix, observation_matrix, process_noise, measurement_noise));
        kalman_filters_ptrs_r_[i].reset(new KalmanFilter(initial_covariance, transition_matrix, observation_matrix, process_noise, measurement_noise));
    }
}

void MmteleopIMU::record_data_init()
{
    // record data
    std::stringstream filename;
    std::time_t now = std::time(NULL);
    std::tm *lt = std::localtime(&now);
    char* home_dir = getenv("HOME");
    filename << home_dir <<"/Documents/log/mmteleop/" << lt->tm_mon +1 << "-" << lt->tm_mday << "-" << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".txt";
    data_file_.open(filename.str());
    if(!data_file_) std::cout<<"error"<<std::endl;

    data_file_ << "time px_l py_l pz_l fx_l fy_l fz_l px_r py_r pz_r fx_r fy_r fz_r" << std::endl;
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

void MmteleopIMU::emergenccy_detection()
{
    auto robot_l = get_robot_state_l();
    auto robot_r = get_robot_state_r();
    Eigen::Vector3d left_wrench_force = Eigen::Vector3d(robot_l.current_wrench.wrench.force.x, robot_l.current_wrench.wrench.force.y, robot_l.current_wrench.wrench.force.z);
    Eigen::Vector3d right_wrench_force = Eigen::Vector3d(robot_r.current_wrench.wrench.force.x, robot_r.current_wrench.wrench.force.y, robot_r.current_wrench.wrench.force.z);

    double thereshold = 15; // N
    if (left_wrench_force.norm() > thereshold || right_wrench_force.norm() > thereshold)
    {
        RCLCPP_ERROR(this->get_logger(), "Emergency detected, current force is: [%f, %f]", left_wrench_force.norm(), right_wrench_force.norm());
        emergency_stop_ = true;
    }
}

void MmteleopIMU::tasks_init()
{

    // Go Home
    task_pushback(TaskPtr("Go Home", [this](){
        std::vector<double> left_home_joints = {-0.568016, -0.082871, 2.115638, 0.011060, -1.239965, 0.353554};
        std::vector<double> right_home_joints = {0.497529, -0.080796, 2.134854, -0.084095, -1.352510, -0.399333};

        if(joint_move(left_home_joints, right_home_joints, 5.0)){
            set_task_finished();
        }
    }));

    // Initial tf and waiting teleop service
    task_pushback(TaskPtr("Initial tf and waiting teleop service",[this](){
        tf_update();
        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f], [%f, %f, %f]", imu_acc_l_(0), imu_acc_l_(1), imu_acc_l_(2),body_left_hand_.translation().x(), body_left_hand_.translation().y(), body_left_hand_.translation().z());
        // print the left orientation
        // RCLCPP_INFO(this->get_logger(), "Left orientation is: [%f, %f, %f, %f]", imu_ori_l_.x(), imu_ori_l_.y(), imu_ori_l_.z(), imu_ori_l_.w());

        if (teleop_start_ && imu_received_l_ && imu_received_r_)
        {
            RCLCPP_INFO(this->get_logger(), "Teleop service is on");
            set_task_finished();
        }
    }));

    // Start Teleop
    task_pushback(TaskPtr("Start Teleop", [this](){
        tf_update();
        // Initialize the start position
        body_neck_start_ = body_neck_;
        hand_pose_start_l_ = (body_neck_start_.inverse() * body_left_hand_).translation(); // The hand pose is relative to the neck
        hand_pose_start_r_ = (body_neck_start_.inverse() * body_right_hand_).translation(); // The hand pose is relative to the neck
        hand_ori_start_l_ = imu_ori_l_;
        hand_ori_start_r_ = imu_ori_r_;

        // Initialize the kalman filter
        // Left kalman filter
        kalman_filters_ptrs_l_[0]->set_initial_state(Eigen::Vector3d(hand_pose_start_l_(0) , 0, 0));
        kalman_filters_ptrs_l_[1]->set_initial_state(Eigen::Vector3d(hand_pose_start_l_(1), 0, 0));
        kalman_filters_ptrs_l_[2]->set_initial_state(Eigen::Vector3d(hand_pose_start_l_(2), 0, 0));
        // right kalman filter
        kalman_filters_ptrs_r_[0]->set_initial_state(Eigen::Vector3d(hand_pose_start_r_(0) , 0, 0));
        kalman_filters_ptrs_r_[1]->set_initial_state(Eigen::Vector3d(hand_pose_start_r_(1), 0, 0));
        kalman_filters_ptrs_r_[2]->set_initial_state(Eigen::Vector3d(hand_pose_start_r_(2), 0, 0));

        // record data
        record_data_init();
    },
    [this](){
        auto robot_l = get_robot_state_l();
        auto robot_r = get_robot_state_r();
        Eigen::Vector3d zero_3d = Eigen::Vector3d::Zero();
        // Update tf
        tf_update();
        
        // Left
        // Update kalman filter
        Eigen::Vector3d current_left_hand = (body_neck_start_.inverse() * body_left_hand_).translation();

        Eigen::Vector3d hand_filtered_l_x = kalman_filters_ptrs_l_[0]->update(imu_acc_l_(0), current_left_hand(0));
        Eigen::Vector3d hand_filtered_l_y = kalman_filters_ptrs_l_[1]->update(imu_acc_l_(1), current_left_hand(1));
        Eigen::Vector3d hand_filtered_l_z = kalman_filters_ptrs_l_[2]->update(imu_acc_l_(2), current_left_hand(2));

        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_l = robot_l.start_pose;
        Eigen::Quaterniond robot_ori_start_l = Eigen::Quaterniond(start_pose_l.pose.orientation.w, start_pose_l.pose.orientation.x, start_pose_l.pose.orientation.y, start_pose_l.pose.orientation.z);
        Eigen::Vector3d hand_pose_filtered_l = Eigen::Vector3d(hand_filtered_l_x(0), hand_filtered_l_y(0), hand_filtered_l_z(0));

        // Calculate the target pose
        Eigen::Quaterniond ori_inc_l =  hand_ori_start_l_.inverse() * imu_ori_l_;
        Eigen::Quaterniond ori_inc_trans_l =  y_axis_mirror_ ?  Eigen::Quaterniond(ori_inc_l.w(), ori_inc_l.x(), -ori_inc_l.y(), ori_inc_l.z()) : ori_inc_l; // This is a trick to convert the orientation from the right hand to the left hand rotation
        Eigen::Quaterniond robot_ori_target_l =  ori_inc_trans_l * robot_ori_start_l;
        
        geometry_msgs::msg::PoseStamped target_pose_l = robot_l.start_pose;
        target_pose_l.header.stamp = this->now();
        target_pose_l.pose.position.x = start_pose_l.pose.position.x  + (hand_pose_filtered_l(0) - hand_pose_start_l_(0)) * 1.0;
        target_pose_l.pose.position.y = start_pose_l.pose.position.y  + (hand_pose_filtered_l(1) - hand_pose_start_l_(1)) * 1.0;
        target_pose_l.pose.position.z = start_pose_l.pose.position.z  + (hand_pose_filtered_l(2) - hand_pose_start_l_(2)) * 1.0;

        target_pose_l.pose.orientation.x = robot_ori_target_l.x();
        target_pose_l.pose.orientation.y = robot_ori_target_l.y();
        target_pose_l.pose.orientation.z = robot_ori_target_l.z();
        target_pose_l.pose.orientation.w = robot_ori_target_l.w();
        


        // Right
        // Update kalman filter
        Eigen::Vector3d current_right_hand = (body_neck_start_.inverse() * body_right_hand_).translation();
        
        Eigen::Vector3d hand_filtered_r_x = kalman_filters_ptrs_r_[0]->update(imu_acc_r_(0), current_right_hand(0));
        Eigen::Vector3d hand_filtered_r_y = kalman_filters_ptrs_r_[1]->update(imu_acc_r_(1), current_right_hand(1));
        Eigen::Vector3d hand_filtered_r_z = kalman_filters_ptrs_r_[2]->update(imu_acc_r_(2), current_right_hand(2));

        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_r = robot_r.start_pose;
        Eigen::Quaterniond robot_ori_start_r = Eigen::Quaterniond(start_pose_r.pose.orientation.w, start_pose_r.pose.orientation.x, start_pose_r.pose.orientation.y, start_pose_r.pose.orientation.z);
        Eigen::Vector3d hand_pose_filtered_r = Eigen::Vector3d(hand_filtered_r_x(0), hand_filtered_r_y(0), hand_filtered_r_z(0));
        
        // Calculate the target pose
        Eigen::Quaterniond ori_inc_r =  hand_ori_start_r_.inverse() * imu_ori_r_;
        Eigen::Quaterniond ori_inc_trans_r = y_axis_mirror_ ? Eigen::Quaterniond(ori_inc_r.w(), ori_inc_r.x(), -ori_inc_r.y(), ori_inc_r.z()) : ori_inc_r; // This is a trick to convert the orientation from the right hand to the left hand rotation
        Eigen::Quaterniond robot_ori_target_r =  ori_inc_trans_r * robot_ori_start_r;

       
        geometry_msgs::msg::PoseStamped target_pose_r = robot_r.start_pose;
        target_pose_r.header.stamp = this->now();
        target_pose_r.pose.position.x = start_pose_r.pose.position.x  + (hand_pose_filtered_r(0) - hand_pose_start_r_(0)) * 1.0;
        target_pose_r.pose.position.y = start_pose_r.pose.position.y  + (hand_pose_filtered_r(1) - hand_pose_start_r_(1)) * 1.0;
        target_pose_r.pose.position.z = start_pose_r.pose.position.z  + (hand_pose_filtered_r(2) - hand_pose_start_r_(2)) * 1.0;
        
        target_pose_r.pose.orientation.x = robot_ori_target_r.x();
        target_pose_r.pose.orientation.y = robot_ori_target_r.y();
        target_pose_r.pose.orientation.z = robot_ori_target_r.z();
        target_pose_r.pose.orientation.w = robot_ori_target_r.w();
        
        
        // Set target pose
        emergenccy_detection();
        if (!emergency_stop_)
        {
            set_target_pose_l(target_pose_l);
            set_target_pose_r(target_pose_r);
        }        
        
        // record data
        {
            auto system_state = get_system_state();
            data_file_ << (system_state.current_time - system_state.start_time).seconds() << " " 
            << robot_l.current_pose.pose.position.x << " "
            << robot_l.current_pose.pose.position.y << " "
            << robot_l.current_pose.pose.position.z << " "
            << robot_l.current_wrench.wrench.force.x << " "
            << robot_l.current_wrench.wrench.force.y << " "
            << robot_l.current_wrench.wrench.force.z << " "
            << robot_r.current_pose.pose.position.x << " "
            << robot_r.current_pose.pose.position.y << " "
            << robot_r.current_pose.pose.position.z << " "
            << robot_r.current_wrench.wrench.force.x << " "
            << robot_r.current_wrench.wrench.force.y << " "
            << robot_r.current_wrench.wrench.force.z << " " << std::endl;
        }


        if (!teleop_start_)
        {
            data_file_.close();
            // goto_init_task();
            set_task_finished();
        }
    }));

    // Sleep for 1 second
    task_pushback(TaskPtr("Sleep for 1.0 seconds", [this](){
        if(sleep(1.0))
        {
            set_task_finished();
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
