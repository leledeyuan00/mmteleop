#include <mmteleop/kalman_filter.hpp>

namespace garment_research{

KalmanFilter::KalmanFilter(const Matrix9d& initial_covariance,
              const Matrix9d& jacobian_matrix, const MatrixHd& observation_matrix, 
              const Matrix9d& process_noise, const Matrix3d& measurement_noise)
{
    covariance_ = initial_covariance;
    jacobian_matrix_ = jacobian_matrix;
    observation_matrix_ = observation_matrix;
    process_noise_ = process_noise;
    measurement_noise_ = measurement_noise;
}

Vector9d KalmanFilter::prio_estimation(Vector3d imu_acc, Matrix3d rot, double dt)
{
    Vector3d pos, vel, acc_bias;
    pos = state_.block<3,1>(0,0);
    vel = state_.block<3,1>(3,0);
    acc_bias = state_.block<3,1>(6,0);

    // Prediction
    Vector3d g = Vector3d(0, 0, 0.981);
    Vector3d acc = rot * (imu_acc - acc_bias) - g;
    pos = pos + vel * dt + 0.5 * acc * dt * dt;
    vel = vel + acc * dt;

    state_.block<3,1>(0,0) = pos;
    state_.block<3,1>(3,0) = vel;

    // Jacobian matrix
    jacobian_matrix_ << Matrix3d::Identity(), Matrix3d::Identity() * dt, -rot * dt * dt / 2,
                        Matrix3d::Zero(), Matrix3d::Identity(), -rot,
                        Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity();

    covariance_ = jacobian_matrix_ * covariance_ * jacobian_matrix_.transpose() + process_noise_;
    return state_;
}


// With IMU
Vector9d KalmanFilter::update(const Vector3d imu_acc,const Vector3d hand_position, Matrix3d rot, double dt)
{

    // Insert IMU accleration instead of estimated acceleration
    Vector3d measurement_v = hand_position;

    // Prio estimation
    prio_estimation(imu_acc, rot, dt);

    // Kalman gain
    Matrix3d S = observation_matrix_* covariance_ * observation_matrix_.transpose() + measurement_noise_;
    MatrixKd kalman_gain = (covariance_ * observation_matrix_.transpose()) * S.inverse();

    // Post estimation
    state_ = state_ + kalman_gain * (measurement_v - observation_matrix_ * state_);

    // Update covariance
    covariance_ = (Matrix9d::Identity() - kalman_gain * observation_matrix_) * covariance_;
    return state_;
}

} // namespace