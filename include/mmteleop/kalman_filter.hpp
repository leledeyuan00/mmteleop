#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_


#include <Eigen/Dense>
#include <memory>
namespace garment_research{

using namespace Eigen;
#define Matrix9d Matrix<double, 9, 9>
#define MatrixHd Matrix<double, 3, 9>
#define MatrixKd Matrix<double, 9, 3>
#define Vector9d Matrix<double, 9, 1>

/*
* Kalman filter class
* state: x, v, a_bias [9x1]
*/
class KalmanFilter
{
public:

    KalmanFilter(const Matrix9d& initial_covariance,
              const Matrix9d& jacobian_matrix, const MatrixHd& observation_matrix, 
              const Matrix9d& process_noise, const Matrix3d& measurement_noise);

    void set_initial_state(const Vector9d& initial_state){
        state_ = initial_state;
    }

    Vector9d prio_estimation(Vector3d imu_acc, Matrix3d rot, double dt);
    Vector9d update(const Vector3d imu_acc,const Vector3d hand_position, Matrix3d rot, double dt); // with IMU
    Vector9d update(const Vector3d hand_position); // without IMU
    Vector9d get_state() const {return state_;}

private:
    Vector9d state_;
    Matrix9d covariance_;
    Matrix9d jacobian_matrix_;
    MatrixHd observation_matrix_;
    Matrix9d process_noise_;
    Matrix3d measurement_noise_;
};

}

#endif // KALMAN_FILTER_HPP_