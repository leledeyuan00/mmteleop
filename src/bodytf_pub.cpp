#include "mmteleop/mmteleop.hpp"

MMTeleop::MMTeleop() : Node("multi_modal_teleopration")
{
    ros_init();
}

void MMTeleop::ros_init()
{

    current_id_ = 0;
    tracking_ready_ = false;

    body_tracking_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/track/body_tracking_data", rclcpp::SystemDefaultsQoS(),
        std::bind(&MMTeleop::body_arrary_callback, this, std::placeholders::_1)
    );

    switch_id_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/switch_id", 
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
        {
            if (tracking_ready_)
            {
                response->success = false;
                response->message = "Please stop tracking first";
                RCLCPP_WARN(this->get_logger(), "Please stop tracking first");
                return;
            }
            
            auto it = std::find(recorded_id_.begin(), recorded_id_.end(), current_id_);
            if (it != recorded_id_.end())
            {
                get_nearest_id(current_id_, recorded_id_, request->data);                
            }
            else
            {
                current_id_ = recorded_id_.empty() ? 0 : recorded_id_[0];
            }
            response->success = true;
            response->message = "Switch id success. Current id is " + std::to_string(current_id_);
        }
    );

    reset_body_index_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/reset_body_index", 
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
        {
            if (tracking_ready_)
            {
                response->success = false;
                response->message = "Please stop tracking first";
                RCLCPP_WARN(this->get_logger(), "Please stop tracking first");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Current id size: %d", recorded_id_.size());
            recorded_id_.clear();
            response->success = true;
            response->message = "Reset id success";
        }
    );

    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(this));

}

void MMTeleop::body_arrary_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    
    int tracking_body_index = 0;
    size_t size = msg->markers.size();
    if (size < MARKER_NUM)
    {
        return; // Some communication error
    }

    size_t ids_size = size / MARKER_NUM; // The number of people
    
    bool id_found = false;

    // recording all of the body id 
    auto if_not_recorded = [&](int compared_id, std::vector<int> ids)
    {
        for (size_t i = 0; i < ids.size(); i++)
        {
            if (ids[i] == compared_id)
            {
                return false;
            }
        }
        return true;
    };

    for (size_t i = 0; i < ids_size; i++)
    {
        int id = msg->markers[i * MARKER_NUM].id;
        if (if_not_recorded(id, recorded_id_))
        {
            recorded_id_.push_back(msg->markers[i * MARKER_NUM].id);
        }

        // Find the current id
        if (id == current_id_)
        {
            id_found = true;
            tracking_body_index = i * MARKER_NUM;
        }
    }

    // if the recorded id is not found, return
    if (!id_found)
    {
        return;
    }

    std::vector<Eigen::Vector3d> marker_positions;
    for (size_t i = 0; i < MARKER_NUM; i++)
    {
        Eigen::Vector3d marker_position;
        marker_position << msg->markers[tracking_body_index + i].pose.position.x, msg->markers[tracking_body_index + i].pose.position.y, msg->markers[tracking_body_index + i].pose.position.z;
        marker_positions.push_back(marker_position);
    }


    std::vector<Eigen::Vector3d> body_positions = {
        marker_positions[14], // 0 right hand
        marker_positions[7],  // 1 left hand
        marker_positions[3],  // 2 neck upper
        marker_positions[4],  // 3 neck left
        marker_positions[11],  // 4 neck right
    };

    get_hand_position(body_positions, right_hand_position_, left_hand_position_); // Calculate hand position
}

void MMTeleop::get_hand_position(std::vector<Eigen::Vector3d> body_positions, Eigen::Vector3d& right_hand_position, Eigen::Vector3d& left_hand_position)
{
    // Get the middle point of the neck
    Eigen::Vector3d neck_middle_position = (body_positions[2] + body_positions[3] + body_positions[4]) / 3;

    // Get the base rotation matrix of the neck
    Eigen::Vector3d neck_y_axis = (body_positions[4] - body_positions[3]).normalized(); // Neck y axis
    Eigen::Vector3d left2upper_vector = (body_positions[2] - neck_middle_position);
    Eigen::Vector3d neck_z_axis = left2upper_vector.normalized(); // Neck z axis
    Eigen::Vector3d neck_x_axis = neck_y_axis.cross(neck_z_axis); // Neck x axis

    Eigen::Matrix3d neck_rotation_matrix;
    neck_rotation_matrix << neck_x_axis, neck_y_axis, neck_z_axis;

    Eigen::Quaterniond neck_rotation_quaternion = Eigen::Quaterniond(neck_rotation_matrix);

    geometry_msgs::msg::TransformStamped neck_transform;
    neck_transform.header.stamp = my_clock_.now();
    neck_transform.header.frame_id = "track_depth_camera_link";
    neck_transform.child_frame_id = "body_base_link";
    neck_transform.transform.translation.x = neck_middle_position(0);
    neck_transform.transform.translation.y = neck_middle_position(1);
    neck_transform.transform.translation.z = neck_middle_position(2);
    neck_transform.transform.rotation = tf2::toMsg(neck_rotation_quaternion);
    tf_broadcaster_->sendTransform(neck_transform); // Publish the neck transform for visualization in rviz

    // Get the hand position
    Eigen::Vector3d right_hand_position_temp, left_hand_position_temp;
    right_hand_position_temp =  neck_rotation_matrix.inverse() * (body_positions[0] - neck_middle_position);
    left_hand_position_temp =  neck_rotation_matrix.inverse() * (body_positions[1] - neck_middle_position);

    // low pass filter
    for (size_t i = 0; i < 3; i++)
    {
        right_hand_position(i) = filters::exponentialSmoothing(right_hand_position_temp(i), right_hand_position(i), 0.5);  
        left_hand_position(i) = filters::exponentialSmoothing(left_hand_position_temp(i), left_hand_position(i), 0.5); 
    }

    // publish the hand transform
    geometry_msgs::msg::TransformStamped right_hand_transform;
    right_hand_transform.header.stamp = my_clock_.now();
    right_hand_transform.header.frame_id = "body_base_link";
    right_hand_transform.child_frame_id = "right_hand_link";
    right_hand_transform.transform.translation.x = right_hand_position(0);
    right_hand_transform.transform.translation.y = right_hand_position(1);
    right_hand_transform.transform.translation.z = right_hand_position(2);
    tf_broadcaster_->sendTransform(right_hand_transform); // Publish the neck transform for visualization in rviz

    geometry_msgs::msg::TransformStamped left_hand_transform;
    left_hand_transform.header.stamp = my_clock_.now();
    left_hand_transform.header.frame_id = "body_base_link";
    left_hand_transform.child_frame_id = "left_hand_link";
    left_hand_transform.transform.translation.x = left_hand_position(0);
    left_hand_transform.transform.translation.y = left_hand_position(1);
    left_hand_transform.transform.translation.z = left_hand_position(2);
    tf_broadcaster_->sendTransform(left_hand_transform); // Publish the neck transform for visualization in rviz

}

void MMTeleop::get_nearest_id(int &current_id, std::vector<int> ids, bool dir)
{
    auto it = std::find(ids.begin(), ids.end(), current_id);
    if (it != ids.end())
    {
        if (dir)
        {
            if (it != ids.end() - 1)
            {
                current_id = *(it + 1);
            }
            else
            {
                current_id = *(ids.begin());
            }
        }
        else
        {
            if (it != ids.begin())
            {
                current_id = *(it - 1);
            }
            else
            {
                current_id = *(ids.end() - 1);
            }
        }
    }
}

int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("multi_modal_teleopration"), "Multi modal teleopration Started");
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<MMTeleop>();

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("multi_modal_teleopration"), "Multi modal teleopration Stopped");
    return 0;
}
