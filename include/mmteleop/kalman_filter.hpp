#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_


#include <Eigen/Dense>
#include <memory>
namespace garment_research{
class KalmanFilter
{
public:

    KalmanFilter(const Eigen::Matrix3d& initial_covariance,
              const Eigen::Matrix3d& transition_matrix, const Eigen::Vector3d& observation_matrix, 
              const Eigen::Matrix3d& process_noise, const double& measurement_noise);

    void set_initial_state(const Eigen::Vector3d& initial_state){
        state_ = initial_state;
    }

    Eigen::Vector3d prio_estimation();
    Eigen::Vector3d update(const double imu_acc,const double hand_position); // with IMU
    Eigen::Vector3d update(const double hand_position); // without IMU

private:
    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;
    Eigen::Matrix3d transition_matrix_;
    Eigen::Vector3d observation_matrix_;
    Eigen::Matrix3d process_noise_;
    double measurement_noise_;
};

}

#endif // KALMAN_FILTER_HPP_