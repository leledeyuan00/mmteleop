#include "mmteleop/body_data_record.hpp"

body_record::body_record() : Node("body_record")
{
    ros_init();
}

void body_record::ros_init()
{
    // ros
    tf_listener_.reset(new tf2_ros::TransformListener(this->buffer_));

    // parameters
    this->declare_parameter<std::string>("robot_frame", "robot_base_link");
    this->get_parameter<std::string>("robot_frame", robot_frame_);
    
    // sub
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_msg_ = *msg;
        }
    );

    // srv
    start_record_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/body_record/start_record", std::bind(&body_record::start_record_cb, this, std::placeholders::_1, std::placeholders::_2)   
    );

    // init variables
    right_current_pose_.setIdentity();
    left_current_pose_.setIdentity();

    right_start_wrench_ << 0, 0, 0;
    left_start_wrench_ << 0, 0, 0;

    ros_clock_ = rclcpp::Clock(RCL_ROS_TIME); 

    // thread
    control_loop_thread_ = std::thread(&body_record::loop, this);
}

void body_record::start_record_cb(const std_srvs::srv::SetBool::Request::SharedPtr request, 
                        std_srvs::srv::SetBool::Response::SharedPtr response)
{
    if (request->data)
    {
        record_data_init();
        start_time_ = ros_clock_.now();
        RCLCPP_INFO(this->get_logger(), "Start recording");
        response->message = "Start recording";
    }
    else
    {
        data_file_.close();
        RCLCPP_INFO(this->get_logger(), "Stop recording");
        response->message = "Stop recording";
    }
    response->success = true;
    start_record_data_ = request->data;
}

void body_record::record_data_init()
{
    // record data
    std::stringstream filename;
    std::time_t now = std::time(NULL);
    std::tm *lt = std::localtime(&now);
    char* home_dir = getenv("HOME");
    filename << home_dir <<"/Documents/log/body_record/" << lt->tm_mon +1 << "-" << lt->tm_mday << "-" << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".txt";
    data_file_.open(filename.str());
    if(!data_file_) std::cout<<"error"<<std::endl;

    data_file_ << "time q_x q_y q_z q_w a_x a_y a_z l_x l_y l_z" << std::endl;
}

void body_record::robot_update()
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

void body_record::loop()
{
    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok())
    {
        robot_update();

        if (start_record_data_)
        {
            data_file_ << (ros_clock_.now() - start_time_).seconds() << " " 
            << imu_msg_.orientation.x << " "
            << imu_msg_.orientation.y << " "
            << imu_msg_.orientation.z << " "
            << imu_msg_.orientation.w << " "
            << imu_msg_.linear_acceleration.x << " "
            << imu_msg_.linear_acceleration.y << " "
            << imu_msg_.linear_acceleration.z << " "
            << body_left_hand_.translation().x() << " "
            << body_left_hand_.translation().y() << " "
            << body_left_hand_.translation().z() << " " << std::endl;
            loop_rate.sleep();
        }
    }
}

int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("body_record"), "body_record Started");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<body_record>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}