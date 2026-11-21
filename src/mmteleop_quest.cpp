#include "mmteleop/mmteleop_quest.hpp" 
// OC Demo 2025

using namespace std::chrono_literals;

namespace garment_research
{

// For custom initialization such as service client and parameters
void MmteleopIMU::custom_init()
{

    boundary_limit_l_   << 0.3, 0.8,
                           -0.6, 0.3,
                           -1.2, -0.4;

    boundary_limit_r_   << 0.3, 0.8,
                           -0.3, 0.6,
                           -1.2, -0.4;
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

    // pub for haptics
    haptics_pub_l_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/L_haptic_amplitude", rclcpp::SystemDefaultsQoS()
    );

    haptics_pub_r_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/R_haptic_amplitude", rclcpp::SystemDefaultsQoS()
    );

    // service
    system_reset_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/system_reset", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                std_srvs::srv::SetBool::Response::SharedPtr response) {
            system_reset_ = request->data;
            response->success = true;
            response->message = "System reset is " + std::string(system_reset_ ? "on" : "off");
            if (system_reset_)
            {
                RCLCPP_INFO(this->get_logger(), "Teleop service is trying to start");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Teleop service is trying to stop");
            }
            
        });

    // clients
    eval_client_ = std::make_shared<GarmentClient<std_srvs::srv::SetBool>>(
        "/eval_policy",
        [this](GarmentClient<std_srvs::srv::SetBool>::ResponseSharedPtr response) {
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Evaluation service is on");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Evaluation service is off");
            }
        },
        this->shared_from_this()
    );

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
    geometry_msgs::msg::TransformStamped right_hand_transform, right_hand_transform_quest;
    geometry_msgs::msg::TransformStamped left_hand_transform,  left_hand_transform_quest;
    geometry_msgs::msg::TransformStamped neck_transform;
    std::string robot_frame = "robot_base_link";

    try
    {
        #ifndef USE_QUEST_ONLY
        // body
        neck_transform = buffer_.lookupTransform(robot_frame, "body_base_link", tf2::TimePointZero);
        right_hand_transform = buffer_.lookupTransform(robot_frame, "right_hand_link", tf2::TimePointZero);
        left_hand_transform = buffer_.lookupTransform(robot_frame, "left_hand_link", tf2::TimePointZero);
        #endif

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
    #ifndef USE_QUEST_ONLY
    body_neck_ = tf2::transformToEigen(neck_transform);
    body_right_hand_ = tf2::transformToEigen(right_hand_transform);
    body_left_hand_ = tf2::transformToEigen(left_hand_transform);
    #endif

    body_right_hand_quest_ = tf2::transformToEigen(right_hand_transform_quest);
    body_left_hand_quest_  = tf2::transformToEigen(left_hand_transform_quest);
}

void MmteleopIMU::emergenccy_detection()
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

void MmteleopIMU::tasks_init()
{

    // release the force control and go to the initial position
    task_pushback(TaskPtr("Release the force control and go to the initial position",
    [this](){
        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = this->get_robot_state_l().start_target_monitor;
        right_target_pose = this->get_robot_state_r().start_target_monitor;
        if(this->move(left_target_pose, right_target_pose, 1.0))
        {
            set_task_finished();
        }
    }));

    // Leave two manipulators away from each other
    task_pushback(TaskPtr("Leave two manipulators away from each other", [this](){
        left_initial_pose_ = this->get_robot_state_l().target_monitor;
        right_initial_pose_ = this->get_robot_state_r().target_monitor;
    },
    [this](){

        Eigen::Vector3d left_eef_pos = Eigen::Vector3d(left_initial_pose_.pose.position.x, left_initial_pose_.pose.position.y, left_initial_pose_.pose.position.z);
        Eigen::Vector3d right_eef_pos = Eigen::Vector3d(right_initial_pose_.pose.position.x, right_initial_pose_.pose.position.y, right_initial_pose_.pose.position.z);
        Eigen::Vector3d distance_norm = (right_eef_pos - left_eef_pos).normalized();

        Eigen::Vector3d left_target_pos = left_eef_pos - distance_norm * 0.02;
        Eigen::Vector3d right_target_pos = right_eef_pos + distance_norm * 0.0;

        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = left_initial_pose_;
        right_target_pose = right_initial_pose_;

        left_target_pose.pose.position.x = left_target_pos(0);
        left_target_pose.pose.position.y = left_target_pos(1);
        left_target_pose.pose.position.z = left_target_pos(2);

        right_target_pose.pose.position.x = right_target_pos(0);
        right_target_pose.pose.position.y = right_target_pos(1);
        right_target_pose.pose.position.z = right_target_pos(2);

        if(this->move(left_target_pose, right_target_pose, 2.0))
        {
            set_task_finished();
        }
    }));

    // Leave two manipulators away from each other
    task_pushback(TaskPtr("Leave two manipulators away from each other 2", [this](){
        left_initial_pose_ = this->get_robot_state_l().target_monitor;
        right_initial_pose_ = this->get_robot_state_r().target_monitor;
    },
    [this](){

        Eigen::Vector3d left_eef_pos = Eigen::Vector3d(left_initial_pose_.pose.position.x, left_initial_pose_.pose.position.y, left_initial_pose_.pose.position.z);
        Eigen::Vector3d right_eef_pos = Eigen::Vector3d(right_initial_pose_.pose.position.x, right_initial_pose_.pose.position.y, right_initial_pose_.pose.position.z);
        Eigen::Vector3d distance_norm = (right_eef_pos - left_eef_pos).normalized();

        Eigen::Vector3d left_target_pos = left_eef_pos - distance_norm * 0.02;
        Eigen::Vector3d right_target_pos = right_eef_pos + distance_norm * 0.02;

        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = left_initial_pose_;
        right_target_pose = right_initial_pose_;

        left_target_pose.pose.position.x = left_target_pos(0);
        left_target_pose.pose.position.y = left_target_pos(1);
        left_target_pose.pose.position.z = left_target_pos(2);

        right_target_pose.pose.position.x = right_target_pos(0);
        right_target_pose.pose.position.y = right_target_pos(1);
        right_target_pose.pose.position.z = right_target_pos(2);

        if(this->move(left_target_pose, right_target_pose, 2.0))
        {
            set_task_finished();
        }
    }));

    // Go Home
    task_pushback(TaskPtr("Go Home", [this](){

        std::vector<double> left_home_joints = {-0.369624, -0.034632, 1.676058, 1.546698, -1.214689, -1.493617};
        std::vector<double> right_home_joints = {0.235998, -0.104174, 1.737613, -1.558826, -1.327168, 1.498374};

        if(joint_move(left_home_joints, right_home_joints, 5.0)){
            set_task_finished();
        }
    }));


    // Go to the initial position
    task_pushback(TaskPtr("Go to the initial position",[this](){
        // Generate the random initial position for the left arm
        // Initialize the random generator
        std::random_device rd;
        std::mt19937 gen(rd());

        // position distribution
        std::uniform_real_distribution<double> pos_dist_x(-0.01, 0.01);
        std::uniform_real_distribution<double> pos_dist_y(-0.01, 0.01);
        std::uniform_real_distribution<double> pos_dist_z(-0.03, -0.03);

        // orientation distribution [degrees]
        std::uniform_real_distribution<double> ori_dist_x(-10, 10);
        std::uniform_real_distribution<double> ori_dist_y(-10, 10);
        std::uniform_real_distribution<double> ori_dist_z(-10, 10);
        

        left_initial_pose_ = this->get_robot_state_l().start_pose;
        right_initial_pose_ = this->get_robot_state_r().start_pose;

        left_initial_pose_.pose.position.x = 0.476479 + pos_dist_x(gen);
        left_initial_pose_.pose.position.y = -0.038268 + pos_dist_y(gen);
        left_initial_pose_.pose.position.z =  -0.831157 + pos_dist_z(gen);

        Eigen::AngleAxisd rotation_x(ori_dist_x(gen) * M_PI / 180.0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotation_y(ori_dist_y(gen) * M_PI / 180.0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation_z(ori_dist_z(gen) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond random_rotation = rotation_z * rotation_y * rotation_x;
        Eigen::Quaterniond initial_orientation(0.5, -0.5, 0.5, 0.5);
        Eigen::Quaterniond final_orientation = random_rotation * initial_orientation;
        left_initial_pose_.pose.orientation = tf2::toMsg(final_orientation);

        // Fix the right hand pose
        right_initial_pose_.pose.position.x = 0.466051;
        right_initial_pose_.pose.position.y = 0.103459;
        right_initial_pose_.pose.position.z = -0.833544;
        right_initial_pose_.pose.orientation.x = 0.5;
        right_initial_pose_.pose.orientation.y = 0.5;
        right_initial_pose_.pose.orientation.z =  -0.5;
        right_initial_pose_.pose.orientation.w = 0.5;
    }, 
    [this]()
    {
        if(this->move(left_initial_pose_, right_initial_pose_, 5.0))
        {
            set_task_finished();
        }
    }));

    // deactivate the motion
    task_pushback(TaskPtr("Deactivate the motion", [this](){
        this->deactivate();
        set_task_finished();
    }));

    // start evaluation
    task_pushback(TaskPtr("Start Evaluation", [this](){
        RCLCPP_INFO(this->get_logger(), "Start Evaluation");
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        if(!eval_client_->get_service_called())
        {
            eval_client_->call(request);
        }
        if(eval_client_->get_service_finished())
        {
            eval_client_->reset();
            set_task_finished();
            // this->goto_last_task();
        }
    }));

    // wait for the system reset service to be called
    task_pushback(TaskPtr("Wait for the system reset service to be called", [this](){
        if(system_reset_)
        {
            RCLCPP_INFO(this->get_logger(), "System reset service is called");
            system_reset_ = false;
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

    // activate the motion
    task_pushback(TaskPtr("Activate the motion", [this](){
        this->activate();
        set_task_finished();
    }));

    // Go to the initial task
    task_pushback(TaskPtr("Go to the initial task", [this](){
        goto_init_task();
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
