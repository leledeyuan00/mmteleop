#include <mmteleop/kalman_filter.hpp>

namespace garment_research{

KalmanFilter::KalmanFilter(const Eigen::Matrix3d& initial_covariance,
                        const Eigen::Matrix3d& transition_matrix, const Eigen::Matrix3d& observation_matrix, 
                        const Eigen::Matrix3d& process_noise, const Eigen::Matrix3d& measurement_noise)
{
    covariance_ = initial_covariance;
    transition_matrix_ = transition_matrix;
    observation_matrix_ = observation_matrix;
    process_noise_ = process_noise;
    measurement_noise_ = measurement_noise;
}

Eigen::Vector3d KalmanFilter::prio_estimation()
{
    state_ = transition_matrix_ * state_;
    covariance_ = transition_matrix_ * covariance_ * transition_matrix_.transpose() + process_noise_;
    return state_;
}


// With IMU
Eigen::Vector3d KalmanFilter::update(const double imu_acc,const double hand_position)
{

    // Insert IMU accleration instead of estimated acceleration
    Eigen::Vector3d measurement_v = Eigen::Vector3d(hand_position, 0, imu_acc);

    // Prio estimation
    prio_estimation();

    // Kalman gain
    Eigen::Matrix3d S = observation_matrix_.transpose() * covariance_ * observation_matrix_ + measurement_noise_;
    Eigen::Matrix3d kalman_gain = (covariance_ * observation_matrix_) * S.inverse();

    // Post estimation
    state_ = state_ + kalman_gain * (measurement_v - observation_matrix_.transpose() * state_);

    // Update covariance
    covariance_ = (Eigen::Matrix3d::Identity(state_.size(), state_.size()) - kalman_gain * observation_matrix_.transpose()) * covariance_;
    return state_;
}

// Without IMU
Eigen::Vector3d KalmanFilter::update(const double hand_position)
{

    Eigen::Vector3d measurement_v = Eigen::Vector3d(hand_position, 0, 0);

    // Prio estimation
    prio_estimation();

    // Kalman gain
    Eigen::Matrix3d S = observation_matrix_.transpose() * covariance_ * observation_matrix_ + measurement_noise_;
    Eigen::Matrix3d kalman_gain = (covariance_ * observation_matrix_) * S.inverse();

    // Post estimation
    state_ = state_ + kalman_gain * (measurement_v - observation_matrix_.transpose() * state_);

    // Update covariance
    covariance_ = (Eigen::Matrix3d::Identity(state_.size(), state_.size()) - kalman_gain * observation_matrix_.transpose()) * covariance_;
    return state_;
}

} // namespace