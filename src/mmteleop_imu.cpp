#include "mmteleop/mmteleop_imu.hpp" 
// OC Demo 2025

using namespace std::chrono_literals;

namespace garment_research
{

// For custom initialization such as service client and parameters
void MmteleopIMU::custom_init()
{

    boundary_limit_l_   << 0.3, 0.8,
                           -0.3, 0.3,
                           -0.8, -0.5;

    boundary_limit_r_   << 0.3, 0.8,
                           -0.3, 0.3,
                           -0.8, -0.5;

    // imu sub
    imu_acc_l_buffer_.resize(30); // for 240 ms
    imu_ori_l_buffer_.resize(30);
    imu_sub_l_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/left_cartesian_compliance_controller/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_msg_l_ = *msg;
            Eigen::Quaterniond imu_ori_l = Eigen::Quaterniond(imu_msg_l_.orientation.w, imu_msg_l_.orientation.x, imu_msg_l_.orientation.y, imu_msg_l_.orientation.z);
            Eigen::Vector3d imu_acc = Eigen::Vector3d(imu_msg_l_.linear_acceleration.x, imu_msg_l_.linear_acceleration.y, imu_msg_l_.linear_acceleration.z);
            Vector3d imu_acc_l = imu_acc * 10;
            imu_acc_l_buffer_.push_back(imu_acc_l);
            imu_acc_l_buffer_.erase(imu_acc_l_buffer_.begin());
            imu_ori_l_buffer_.push_back(imu_ori_l);
            imu_ori_l_buffer_.erase(imu_ori_l_buffer_.begin());
            if (!imu_received_l_){
                RCLCPP_INFO(this->get_logger(), "IMU received");
                imu_received_l_ = true;
            }
            new_imu_data_l_ = true;
        });
    
    imu_acc_r_buffer_.resize(30); // for 240 ms
    imu_ori_r_buffer_.resize(30);
    imu_sub_r_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/right_cartesian_compliance_controller/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_msg_r_ = *msg;
            imu_ori_r_buffer_[0] = Eigen::Quaterniond(imu_msg_r_.orientation.w, imu_msg_r_.orientation.x, imu_msg_r_.orientation.y, imu_msg_r_.orientation.z);
            Eigen::Vector3d imu_acc = Eigen::Vector3d(imu_msg_r_.linear_acceleration.x, imu_msg_r_.linear_acceleration.y, imu_msg_r_.linear_acceleration.z);
            Vector3d imu_acc_r = imu_acc * 10;
            imu_acc_r_buffer_.push_back(imu_acc_r);
            imu_acc_r_buffer_.erase(imu_acc_r_buffer_.begin());
            imu_ori_r_buffer_.push_back(imu_ori_r_buffer_[0]);
            imu_ori_r_buffer_.erase(imu_ori_r_buffer_.begin());
            if (!imu_received_r_){
                RCLCPP_INFO(this->get_logger(), "IMU received");
                imu_received_r_ = true;
            }
            new_imu_data_r_ = true;
        });

    body_tracking_status_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/body_tracking_status", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
            new_tracking_data_ = true;
        });

    // pub for monitor
    monitor_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/tele/monitor", rclcpp::SystemDefaultsQoS()
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

    // Initialize Kalman
    // Kalman filter for calculating cartesian velocities
    double dt = 0.008; // 4 ms
    double sigma_bias = 1.0;
    double sigma_measurement = 0.01;
    double sigma_dtheata = 1.0;
    // std::shared_ptr<KalmanFilter> kalman_filter_ptr;
    Matrix12d initial_covariance = Matrix12d::Identity();
    Matrix12d jacobian_matrix = Matrix12d::Identity(); // will be updated in the kalman filter loop
    MatrixHd observation_matrix = (MatrixHd() << Matrix3d::Identity(), Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero()).finished(); // 3x9
    Matrix12d process_noise;
    process_noise << Matrix3d::Identity() * pow(dt,4)/4, Matrix3d::Identity() * pow(dt,3)/2, Matrix3d::Zero(), Matrix3d::Zero(),
                     Matrix3d::Identity() * pow(dt,3)/2, Matrix3d::Identity() * pow(dt,2), Matrix3d::Zero(), Matrix3d::Zero(),
                     Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity() * pow(sigma_bias,2), Matrix3d::Zero(),
                     Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity() * pow(sigma_dtheata,2); 
    Matrix3d measurement_noise = Matrix3d::Identity() * pow(sigma_measurement,2); // 

    kalman_filter_ptr_l_.reset(new KalmanFilter(initial_covariance, jacobian_matrix, observation_matrix, process_noise, measurement_noise));
    kalman_filter_ptr_r_.reset(new KalmanFilter(initial_covariance, jacobian_matrix, observation_matrix, process_noise, measurement_noise));
    
}

void MmteleopIMU::record_data_init()
{
    // record data
    std::time_t now = std::time(NULL);
    std::tm *lt = std::localtime(&now);
    char* home_dir = getenv("HOME");
    filename_ = std::string(home_dir) + "/Documents/log/mmteleop/" +
            std::to_string(lt->tm_mon + 1) + "-" +
            std::to_string(lt->tm_mday) + "-" +
            std::to_string(lt->tm_hour) + "-" +
            std::to_string(lt->tm_min) + "-" +
            std::to_string(lt->tm_sec) + ".txt";
    data_file_.open(filename_);
    if(!data_file_) std::cout<<"error"<<std::endl;
    data_file_ << "time px_l py_l pz_l fx_l fy_l fz_l px_r py_r pz_r fx_r fy_r fz_r" << std::endl;
}

void MmteleopIMU::read_data_init()
{
    // read data
    data_file_in_.open(filename_);
    if(!data_file_in_) std::cout<<"error"<<std::endl;
    std::string line;
    std::getline(data_file_in_, line);  // This reads the header line
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

        std::vector<double> left_home_joints = {-0.206238, 0.011980, 2.086497, -1.680621, 1.395523, 2.115011};
        std::vector<double> right_home_joints = {0.069006, -0.018931, 2.117219, 1.604778, 1.507740, -2.009313};

        if(joint_move(left_home_joints, right_home_joints, 5.0)){
            set_task_finished();
        }
    }));

    // Initial tf, sin wave looping and waiting teleop service
    task_pushback(TaskPtr("Initial tf ,move along y-axis with a sin wave 20mm, waiting teleop service",
    // loop function
    [this](){
        tf_update();
        double distance = 0.02;
        double loop_duration = 8.0;
        auto sin_start_pose_l = get_robot_state_l().start_pose;
        auto sin_start_pose_r = get_robot_state_r().start_pose;
        double current_duration = (get_system_state().current_time - get_system_state().start_time).seconds();

        auto left_pose = get_robot_state_l().start_pose;
        auto right_pose = get_robot_state_r().start_pose;

        left_pose.pose.position.y = sin_start_pose_l.pose.position.y - (distance * sin(current_duration  / loop_duration * (2 * M_PI) + 3*M_PI/2) + distance);
        right_pose.pose.position.y = sin_start_pose_r.pose.position.y + (distance * sin(current_duration  / loop_duration * (2 * M_PI) + 3*M_PI/2) + distance);

        set_target_pose_l(left_pose);
        set_target_pose_r(right_pose);

        if (teleop_start_ && imu_received_l_ && imu_received_r_)
        {
            RCLCPP_INFO(this->get_logger(), "Teleop service is on");
            set_task_finished();
            teleop_start_ = false; // reset teleop start flag
        }
    }));

    // Going to teleop start position
    task_pushback(TaskPtr("Going to teleop start positions", [this](){

        std::vector<double> left_home_joints = {-0.554431, -0.019080, 2.253256, -3.011834, 1.082631, 3.040541};
        std::vector<double> right_home_joints = {0.486575, -0.113307, 2.366543, 3.091654, 1.115678, -2.921454};

        if(joint_move(left_home_joints, right_home_joints, 4.0)){
            set_task_finished();
        }
    }));

    // Waiting until the start button is pressed
    task_pushback(TaskPtr("Waiting until the start button is pressed", [this](){
        if (teleop_start_)
        {
            RCLCPP_INFO(this->get_logger(), "Teleop start button is pressed");
            set_task_finished();
            teleop_start_ = false; // reset teleop start flag
        }
    }));

    // Pre calibration for the kalman filter
    task_pushback(TaskPtr("Pre calibration for the kalman filter...... After around 3~5s press the start again", [this](){
        // Initialize the start position
        body_neck_start_ = body_neck_;
        hand_pose_start_l_ = (body_neck_start_.inverse() * body_left_hand_).translation(); // The hand pose is relative to the neck
        hand_pose_start_r_ = (body_neck_start_.inverse() * body_right_hand_).translation(); // The hand pose is relative to the neck

        // Initialize the kalman filter
        Vector3d initial_bias = (Vector3d() <<0.132193723718177,-0.0412141803030037,0.498650440417530).finished();
        // Left kalman filter
        Vector12d initial_state_l = (Vector12d() << hand_pose_start_l_, Vector3d::Zero(), initial_bias, Vector3d::Zero()).finished();
        kalman_filter_ptr_l_->set_initial_state(initial_state_l);

        // right kalman filter
        Vector12d initial_state_r = (Vector12d() << hand_pose_start_r_, Vector3d::Zero(), initial_bias, Vector3d::Zero()).finished();
        kalman_filter_ptr_r_->set_initial_state(initial_state_r);

        teleop_start_ = false;
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

        kalman_filter_ptr_l_->get_state();
        Eigen::Vector3d acc_fir_lf_l = std::accumulate(imu_acc_l_buffer_.begin(), imu_acc_l_buffer_.end(), zero_3d) / imu_acc_l_buffer_.size();
        kalman_filter_ptr_l_->prio_estimation(acc_fir_lf_l , imu_ori_l_buffer_[0].matrix(), 0.008);
        kalman_filter_ptr_l_->update(current_left_hand, imu_ori_l_buffer_[0].matrix(), 0.008);
 

        // Right
        // Update kalman filter
        Eigen::Vector3d current_right_hand = (body_neck_start_.inverse() * body_right_hand_).translation();
        
        kalman_filter_ptr_r_->get_state();
        Eigen::Vector3d acc_fir_lf_r = std::accumulate(imu_acc_r_buffer_.begin(), imu_acc_r_buffer_.end(), zero_3d) / imu_acc_r_buffer_.size();
        kalman_filter_ptr_r_->prio_estimation(acc_fir_lf_r, imu_ori_r_buffer_[0].matrix(), 0.008);
        kalman_filter_ptr_r_->update(current_right_hand, imu_ori_r_buffer_[0].matrix(), 0.008);

        state_l_ = kalman_filter_ptr_l_->get_state();
        geometry_msgs::msg::PointStamped monitor_msg;
        monitor_msg.header.stamp = this->now();
        monitor_msg.header.frame_id = "robot_base_link";
        monitor_msg.point.x = state_l_(6);
        monitor_msg.point.y = state_l_(7);
        monitor_msg.point.z = state_l_(8);
        monitor_pub_->publish(monitor_msg);
        if (teleop_start_ && imu_received_l_ && imu_received_r_)
        {
            RCLCPP_INFO(this->get_logger(), "Teleop service is on");
            state_l_ = kalman_filter_ptr_l_->get_state();
            state_r_ = kalman_filter_ptr_r_->get_state();
            Vector3d current_bias_l = state_l_.block<3,1>(6,0);
            Vector3d current_bias_r = state_r_.block<3,1>(6,0);

            RCLCPP_INFO(this->get_logger(), "Left bias is: [%f, %f, %f]", current_bias_l(0), current_bias_l(1), current_bias_l(2));
            RCLCPP_INFO(this->get_logger(), "Right bias is: [%f, %f, %f]", current_bias_r(0), current_bias_r(1), current_bias_r(2));
            set_task_finished();
        }
    }));

    // Start Teleop
    task_pushback(TaskPtr("Start Teleop", [this](){
        tf_update();
        // Initialize the start position
        body_neck_start_ = body_neck_;
        hand_pose_start_l_ = (body_neck_start_.inverse() * body_left_hand_).translation(); // The hand pose is relative to the neck
        RCLCPP_INFO(this->get_logger(), "Left hand start pose: [%f, %f, %f]", hand_pose_start_l_(0), hand_pose_start_l_(1), hand_pose_start_l_(2));
        hand_pose_start_r_ = (body_neck_start_.inverse() * body_right_hand_).translation(); // The hand pose is relative to the neck
        hand_ori_start_l_ = imu_ori_l_buffer_[0];
        hand_ori_start_r_ = imu_ori_r_buffer_[0];

        // Initialize the kalman filter
        double sigma_measurement = 0.01; // 1 cm
        // Left kalman filter
        Vector12d initial_state_l = (Vector12d() << hand_pose_start_l_, Vector3d::Zero(), state_l_.block<3,1>(6,0), Vector3d::Zero()).finished();
        kalman_filter_ptr_l_->set_initial_state(initial_state_l);
        kalman_filter_ptr_l_->set_measurement_noise(Matrix3d::Identity() * pow(sigma_measurement,2)); // reduce the convidence of the camera tracking position
        // right kalman filter
        Vector12d initial_state_r = (Vector12d() << hand_pose_start_r_, Vector3d::Zero(), state_r_.block<3,1>(6,0), Vector3d::Zero()).finished();
        kalman_filter_ptr_r_->set_initial_state(initial_state_r);
        kalman_filter_ptr_r_->set_measurement_noise(Matrix3d::Identity() * pow(sigma_measurement,2)); // reduce the convidence of the camera tracking position
        // low pass filter
        Eigen::Vector3d alpha(0.005, 0.005, 0.005);
        low_pass_filter_ptr_l_.reset(new LowPassFilter(alpha));
        low_pass_filter_ptr_r_.reset(new LowPassFilter(alpha));

        // refresh recorded buffers
        recorded_trj_l_.clear();
        recorded_trj_r_.clear();
        
    },
    [this](){
        auto robot_l = get_robot_state_l();
        auto robot_r = get_robot_state_r();
        double alpha_ori = 0.1;
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
        // move_rate_ = 1.0;

        // Left
        // Update kalman filter
        Eigen::Vector3d current_left_hand = (body_neck_start_.inverse() * body_left_hand_).translation();

        Eigen::Vector12d hand_filtered_l = kalman_filter_ptr_l_->get_state();
        if (new_imu_data_l_){
        Eigen::Vector3d acc_fir_lf_l = std::accumulate(imu_acc_l_buffer_.begin(), imu_acc_l_buffer_.end(), zero_3d) / imu_acc_l_buffer_.size();
            hand_filtered_l = kalman_filter_ptr_l_->prio_estimation(acc_fir_lf_l , imu_ori_l_buffer_[0].matrix(), 0.008);
            new_imu_data_l_ = false;
        }
        if (new_tracking_data_){
            hand_filtered_l = kalman_filter_ptr_l_->update(current_left_hand, imu_ori_l_buffer_[0].matrix(), 0.008);
        }
        state_l_ = kalman_filter_ptr_l_->get_state();
        // RCLCPP_INFO(this->get_logger(), "hand_filtered_l: [%f, %f, %f]", hand_filtered_l(0), hand_filtered_l(1), hand_filtered_l(2));

        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_l = robot_l.start_pose;
        Eigen::Quaterniond robot_ori_start_l = Eigen::Quaterniond(start_pose_l.pose.orientation.w, start_pose_l.pose.orientation.x, start_pose_l.pose.orientation.y, start_pose_l.pose.orientation.z);
        
        /// limit the hand pose for safety before low pass filter -- Dayuan 0730
        // calculate current allowed increased position range
        Eigen::Matrix<double, 3, 2> current_boundary_limit_l = boundary_limit_l_ 
                                                                - Eigen::Vector3d(start_pose_l.pose.position.x, start_pose_l.pose.position.y, start_pose_l.pose.position.z).replicate(1, 2)
                                                                + hand_pose_start_l_.replicate(1, 2);
        Eigen::Vector3d boundaried_hand_pose_filtered_l = hand_filtered_l.block<3,1>(0,0).cwiseMax(current_boundary_limit_l.col(0)).cwiseMin(current_boundary_limit_l.col(1));

        // low pass filter
        Eigen::Vector3d hand_pose_filtered_l = low_pass_filter_ptr_l_->update(boundaried_hand_pose_filtered_l);

        
        // RCLCPP_INFO(this->get_logger(), "hand_pose_filtered_l: [%f, %f, %f]", hand_pose_filtered_l(0), hand_pose_filtered_l(1), hand_pose_filtered_l(2));
        // Eigen::Vector3d hand_pose_filtered_l = low_pass_filter_ptr_l_->update(current_left_hand);

        // Calculate the target pose
        Eigen::Quaterniond ori_inc_l =  hand_ori_start_l_.inverse() * imu_ori_l_buffer_[0];
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

        // target_pose_l.pose.position.x = std::clamp(target_pose_l.pose.position.x, boundary_left_corner_(0), boundary_right_corner_(0));
        // target_pose_l.pose.position.y = std::clamp(target_pose_l.pose.position.y, boundary_left_corner_(1), boundary_right_corner_(1));
        // target_pose_l.pose.position.z = std::clamp(target_pose_l.pose.position.z, boundary_left_corner_(2), boundary_right_corner_(2));

        target_pose_l.pose.orientation.x = robot_ori_target_l.x();
        target_pose_l.pose.orientation.y = robot_ori_target_l.y();
        target_pose_l.pose.orientation.z = robot_ori_target_l.z();
        target_pose_l.pose.orientation.w = robot_ori_target_l.w();
        


        // Right
        // Update kalman filter
        Eigen::Vector3d current_right_hand = (body_neck_start_.inverse() * body_right_hand_).translation();
        
        Eigen::Vector12d hand_filtered_r = kalman_filter_ptr_r_->get_state();

        if (new_imu_data_r_){
        Eigen::Vector3d acc_fir_lf_r = std::accumulate(imu_acc_r_buffer_.begin(), imu_acc_r_buffer_.end(), zero_3d) / imu_acc_r_buffer_.size();
        hand_filtered_r = kalman_filter_ptr_r_->prio_estimation(acc_fir_lf_r, imu_ori_r_buffer_[0].matrix(), 0.008);
            new_imu_data_r_ = false;
        }

        if (new_tracking_data_){
        hand_filtered_r = kalman_filter_ptr_r_->update(current_right_hand, imu_ori_r_buffer_[0].matrix(), 0.008);
            new_tracking_data_ = false;
        }
        // hand_pose_start_r_ = hand_pose_start_l_;
        // hand_filtered_r = hand_filtered_l;
        // Calculate the start pose
        geometry_msgs::msg::PoseStamped start_pose_r = robot_r.start_pose;
        Eigen::Quaterniond robot_ori_start_r = Eigen::Quaterniond(start_pose_r.pose.orientation.w, start_pose_r.pose.orientation.x, start_pose_r.pose.orientation.y, start_pose_r.pose.orientation.z);
        
        /// limit the hand pose for safety before low pass filter -- Dayuan 0730
        // calculate current allowed increased position range
        Eigen::Matrix<double, 3, 2> current_boundary_limit_r = boundary_limit_r_ 
                                                                - Eigen::Vector3d(start_pose_r.pose.position.x, start_pose_r.pose.position.y, start_pose_r.pose.position.z).replicate(1, 2)
                                                                + hand_pose_start_r_.replicate(1, 2);
        Eigen::Vector3d boundaried_hand_pose_filtered_r = hand_filtered_r.block<3,1>(0,0).cwiseMax(current_boundary_limit_r.col(0)).cwiseMin(current_boundary_limit_r.col(1));

        // low pass filter
        Eigen::Vector3d hand_pose_filtered_r = low_pass_filter_ptr_r_->update(boundaried_hand_pose_filtered_r);
        
        // Calculate the target pose
        Eigen::Quaterniond ori_inc_r =  hand_ori_start_r_.inverse() * imu_ori_r_buffer_[0];
        Eigen::Quaterniond ori_inc_trans_r = axis_mirror_ ? Eigen::Quaterniond(ori_inc_r.w(), ori_inc_r.x(), -ori_inc_r.y(), -ori_inc_r.z()) : ori_inc_r; // This is a trick to convert the orientation from the right hand to the left hand rotation
        // Eigen::Quaterniond ori_inc_trans_r = ori_inc_trans_l;

        // get current orientation for a low pass filter quaternion
        Eigen::Quaterniond current_ori_r(current_pose_r.pose.orientation.w,current_pose_r.pose.orientation.x, current_pose_r.pose.orientation.y, current_pose_r.pose.orientation.z);
        Eigen::Quaterniond robot_ori_des_r = ori_inc_trans_r * robot_ori_start_r;
        Eigen::Quaterniond robot_ori_target_r =  current_ori_r.slerp(alpha_ori, robot_ori_des_r); // 0.2 is the interpolation factor, you can adjust it to make the robot orientation more smooth

       
        geometry_msgs::msg::PoseStamped target_pose_r = robot_r.start_pose;
        Eigen::Vector3d position_inc_r = hand_pose_filtered_r - hand_pose_start_r_;
        
        target_pose_r.header.stamp = this->now();
        target_pose_r.pose.position.x = start_pose_r.pose.position.x  + position_inc_r(0) * move_rate_;
        target_pose_r.pose.position.y = start_pose_r.pose.position.y  + position_inc_r(1) * move_rate_;
        target_pose_r.pose.position.z = start_pose_r.pose.position.z  + position_inc_r(2) * move_rate_;

        // target_pose_r.pose.position.x = std::clamp(target_pose_r.pose.position.x, boundary_left_corner_(0), boundary_right_corner_(0));
        // target_pose_r.pose.position.y = std::clamp(target_pose_r.pose.position.y, boundary_left_corner_(1), boundary_right_corner_(1));
        // target_pose_r.pose.position.z = std::clamp(target_pose_r.pose.position.z, boundary_left_corner_(2), boundary_right_corner_(2));
        
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

            geometry_msgs::msg::PointStamped monitor_msg;
            monitor_msg.header.stamp = this->now();
            monitor_msg.header.frame_id = "robot_base_link";
            monitor_msg.point.x = state_l_(6);
            monitor_msg.point.y = state_l_(7);
            monitor_msg.point.z = state_l_(8);
            monitor_pub_->publish(monitor_msg);
        }        
        
        // record data
        {
            auto system_state = get_system_state();

            PointData left_data;
            PointData right_data;

            std::string time;
            time = std::to_string((system_state.current_time - system_state.start_time).seconds());
            left_data.time = std::stod(time);
            right_data.time = std::stod(time);
            
            left_data.position = Eigen::Vector3d(current_pose_l.pose.position.x, current_pose_l.pose.position.y, current_pose_l.pose.position.z);
            right_data.position = Eigen::Vector3d(current_pose_r.pose.position.x, current_pose_r.pose.position.y, current_pose_r.pose.position.z);
            left_data.orientation = Eigen::Quaterniond(current_pose_l.pose.orientation.w, current_pose_l.pose.orientation.x, current_pose_l.pose.orientation.y, current_pose_l.pose.orientation.z);
            right_data.orientation = Eigen::Quaterniond(current_pose_r.pose.orientation.w, current_pose_r.pose.orientation.x, current_pose_r.pose.orientation.y, current_pose_r.pose.orientation.z);
            left_data.force = Eigen::Vector3d(robot_l.current_wrench.wrench.force.x, robot_l.current_wrench.wrench.force.y, robot_l.current_wrench.wrench.force.z);
            right_data.force = Eigen::Vector3d(robot_r.current_wrench.wrench.force.x, robot_r.current_wrench.wrench.force.y, robot_r.current_wrench.wrench.force.z);
            left_data.torque = Eigen::Vector3d(robot_l.current_wrench.wrench.torque.x, robot_l.current_wrench.wrench.torque.y, robot_l.current_wrench.wrench.torque.z);
            right_data.torque = Eigen::Vector3d(robot_r.current_wrench.wrench.torque.x, robot_r.current_wrench.wrench.torque.y, robot_r.current_wrench.wrench.torque.z);

            recorded_trj_l_.push_back(left_data);
            recorded_trj_r_.push_back(right_data);
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
    
    // Going to teleop start position
    task_pushback(TaskPtr("Going to teleop start positions", [this](){

        std::vector<double> left_home_joints = {-0.554431, -0.019080, 2.253256, -3.011834, 1.082631, 3.040541};
        std::vector<double> right_home_joints = {0.486575, -0.113307, 2.366543, 3.091654, 1.115678, -2.921454};

        if(joint_move(left_home_joints, right_home_joints, 4.0)){
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

    // Back to Start
    task_pushback(TaskPtr("Back to Start", [this](){
        recorded_trj_idx_ = 0;
        linear_int_count_ = 0;
    },
    [this](){
        geometry_msgs::msg::PoseStamped start_pose_l;
        geometry_msgs::msg::PoseStamped start_pose_r;

        start_pose_l.pose.position.x = recorded_trj_l_[0].position[0];
        start_pose_l.pose.position.y = recorded_trj_l_[0].position[1];
        start_pose_l.pose.position.z = recorded_trj_l_[0].position[2];

        start_pose_l.pose.orientation = tf2::toMsg(recorded_trj_l_[0].orientation);


        start_pose_r.pose.position.x = recorded_trj_r_[0].position[0];
        start_pose_r.pose.position.y = recorded_trj_r_[0].position[1];
        start_pose_r.pose.position.z = recorded_trj_r_[0].position[2];

        start_pose_r.pose.orientation = tf2::toMsg(recorded_trj_r_[0].orientation);

        // RCLCPP_INFO(this->get_logger(), "Teleop service is on, left hand orientation: [%f, %f, %f, %f], right hand orientation: [%f, %f, %f, %f]",
        //     get_robot_state_l().current_pose.pose.orientation.x,
        //     get_robot_state_l().current_pose.pose.orientation.y,
        //     get_robot_state_l().current_pose.pose.orientation.z,
        //     get_robot_state_l().current_pose.pose.orientation.w,
        //     get_robot_state_r().current_pose.pose.orientation.x,
        //     get_robot_state_r().current_pose.pose.orientation.y,
        //     get_robot_state_r().current_pose.pose.orientation.z.
        //     get_robot_state_r().current_pose.pose.orientation.w);

        if (move(start_pose_l, start_pose_r, 5.0))
        {
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

    // Playback
    task_pushback(TaskPtr("Playback", [this](){
            teleop_start_ = true;
        },
        [this](){
        if (recorded_trj_idx_ < (recorded_trj_l_.size()/10 - 1) && teleop_start_)
        {
            geometry_msgs::msg::PoseStamped target_pose_l, target_pose_r;
            target_pose_l = get_robot_state_l().start_pose;
            target_pose_r = get_robot_state_r().start_pose;

            target_pose_l.pose.position.x = recorded_trj_l_[recorded_trj_idx_ * 10].position[0] + (recorded_trj_l_[(recorded_trj_idx_+1) * 10].position[0] - recorded_trj_l_[recorded_trj_idx_ * 10].position[0]) * (linear_int_count_ / 10.0);
            target_pose_l.pose.position.y = recorded_trj_l_[recorded_trj_idx_ * 10].position[1] + (recorded_trj_l_[(recorded_trj_idx_+1) * 10].position[1] - recorded_trj_l_[recorded_trj_idx_ * 10].position[1]) * (linear_int_count_ / 10.0);
            target_pose_l.pose.position.z = recorded_trj_l_[recorded_trj_idx_ * 10].position[2] + (recorded_trj_l_[(recorded_trj_idx_+1) * 10].position[2] - recorded_trj_l_[recorded_trj_idx_ * 10].position[2]) * (linear_int_count_ / 10.0);
            

            Eigen::Quaterniond q_l = recorded_trj_l_[recorded_trj_idx_ * 10].orientation.slerp(linear_int_count_ / 10.0, recorded_trj_l_[(recorded_trj_idx_+1) * 10].orientation);
            target_pose_l.pose.orientation = tf2::toMsg(q_l);

            target_pose_r.pose.position.x = recorded_trj_r_[recorded_trj_idx_ * 10].position[0] + (recorded_trj_r_[(recorded_trj_idx_+1) * 10].position[0] - recorded_trj_r_[recorded_trj_idx_ * 10].position[0]) * (linear_int_count_ / 10.0);
            target_pose_r.pose.position.y = recorded_trj_r_[recorded_trj_idx_ * 10].position[1] + (recorded_trj_r_[(recorded_trj_idx_+1) * 10].position[1] - recorded_trj_r_[recorded_trj_idx_ * 10].position[1]) * (linear_int_count_ / 10.0);
            target_pose_r.pose.position.z = recorded_trj_r_[recorded_trj_idx_ * 10].position[2] + (recorded_trj_r_[(recorded_trj_idx_+1) * 10].position[2] - recorded_trj_r_[recorded_trj_idx_ * 10].position[2]) * (linear_int_count_ / 10.0);

            Eigen::Quaterniond q_r = recorded_trj_r_[recorded_trj_idx_ * 10].orientation.slerp(linear_int_count_ / 10.0, recorded_trj_r_[(recorded_trj_idx_+1) * 10].orientation);
            target_pose_r.pose.orientation = tf2::toMsg(q_r);

            linear_int_count_+= 1;
            if (linear_int_count_ >= 10)
            {
                linear_int_count_ = 0;
                recorded_trj_idx_++;
            }

            set_target_pose_l(target_pose_l);
            set_target_pose_r(target_pose_r);
        }
        else
        {
            // set_task_finished();
            teleop_start_ = false;
            set_task_finished();
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
    auto node = std::make_shared<garment_research::MmteleopIMU>();

    node->start();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("garment_motion_fsm"), "Garment Motion Test by FSM Node Stopped");
    return 0;
}
