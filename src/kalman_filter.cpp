#include <mmteleop/kalman_filter.hpp>

namespace garment_research{

KalmanFilter::KalmanFilter(const Matrix12d& initial_covariance,
              const Matrix12d& jacobian_matrix, const MatrixHd& observation_matrix, 
              const Matrix12d& process_noise, const Matrix3d& measurement_noise)
{
    covariance_ = initial_covariance;
    jacobian_matrix_ = jacobian_matrix;
    observation_matrix_ = observation_matrix;
    process_noise_ = process_noise;
    measurement_noise_ = measurement_noise;
    quat_offset_nominal_ = Quaterniond(1,0,0,0);
}

Vector12d KalmanFilter::prio_estimation(Vector3d imu_acc, Matrix3d rot, double dt)
{
    Vector3d pos, vel, acc_bias, dtheta;
    pos = state_.block<3,1>(0,0);
    vel = state_.block<3,1>(3,0);
    acc_bias = state_.block<3,1>(6,0);

    // Prediction
    Vector3d g = Vector3d(0, 0, 9.81);
    Matrix3d rot_with_offset = quat_offset_nominal_.toRotationMatrix() * rot;
    Vector3d acc_r = rot_with_offset * (imu_acc - acc_bias);
    Matrix3d acc_skew = skew_symmetric(acc_r);
    Vector3d acc = acc_r - g;
    pos = pos + vel * dt + 0.5 * acc * dt * dt;
    vel = vel + acc * dt;

    state_.block<3,1>(0,0) = pos;
    state_.block<3,1>(3,0) = vel;
    state_.block<3,1>(9,0) = Vector3d::Zero(); // reset delta_quat_offset to zero every time

    // Jacobian matrix
    jacobian_matrix_ << Matrix3d::Identity(), Matrix3d::Identity() * dt, -rot_with_offset * dt * dt / 2, acc_skew * dt * dt / 2,
                        Matrix3d::Zero(), Matrix3d::Identity(), -rot_with_offset * dt, acc_skew * dt,
                        Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Zero(),
                        Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity();

    covariance_ = jacobian_matrix_ * covariance_ * jacobian_matrix_.transpose() + process_noise_;
    return state_;
}


// With IMU
Vector12d KalmanFilter::update(const Vector3d hand_position, Matrix3d rot, double dt)
{

    // Insert IMU accleration instead of estimated acceleration
    Vector3d measurement_v = hand_position;

    // Kalman gain
    Matrix3d S = observation_matrix_* covariance_ * observation_matrix_.transpose() + measurement_noise_;
    MatrixKd kalman_gain = (covariance_ * observation_matrix_.transpose()) * S.inverse();

    // Post estimation
    Vector12d weighted_diag;
    weighted_diag << 1, 1, 1, 1, 1, 1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    Matrix12d weighted_update = weighted_diag.asDiagonal();
    state_ = state_ + weighted_update * kalman_gain * (measurement_v - observation_matrix_ * state_);

    // delta_quat_offset
    Vector3d dtheta = state_.block<3,1>(9,0);
    dtheta(0) = 0;
    // dtheta(1) = 0; // only rotation around z axis
    Quaterniond delta_quat_offset = small_angle_quaternion(dtheta);
    quat_offset_nominal_ = (delta_quat_offset.conjugate() * quat_offset_nominal_ ).normalized();

    // Update covariance
    covariance_ = (Matrix12d::Identity() - kalman_gain * observation_matrix_) * covariance_;
    return state_;
}

} // namespace