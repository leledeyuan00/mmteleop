#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_


#include <Eigen/Dense>
#include <memory>
namespace garment_research{

using namespace Eigen;
#define Matrix12d Matrix<double, 12, 12>
#define MatrixHd Matrix<double, 3, 12>
#define MatrixKd Matrix<double, 12, 3>
#define Vector12d Matrix<double, 12, 1>

/*
* Kalman filter class
* state: x, v, a_bias, delta_quat_offset [12x1]
*/
class KalmanFilter
{
public:

    KalmanFilter(const Matrix12d& initial_covariance,
              const Matrix12d& jacobian_matrix, const MatrixHd& observation_matrix, 
              const Matrix12d& process_noise, const Matrix3d& measurement_noise);

    void set_initial_state(const Vector12d& initial_state){
        state_ = initial_state;
    }

    void set_measurement_noise(const Matrix3d& measurement_noise){
        measurement_noise_ = measurement_noise;
    }

    void set_quat_offset_nominal(const Quaterniond& quat_offset_nominal){
        quat_offset_nominal_ = quat_offset_nominal;
    }

    Vector12d prio_estimation(Vector3d imu_acc, Matrix3d rot, double dt);
    Vector12d update(const Vector3d hand_position, Matrix3d rot, double dt); // with IMU
    Vector12d update(const Vector3d hand_position); // without IMU
    Vector12d get_state() const {return state_;}
    Quaterniond get_quat_offset() const {return quat_offset_nominal_;}

    Matrix3d skew_symmetric(const Vector3d& vec){
        Matrix3d skew;
        skew << 0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
        return skew;
    }

    Quaterniond small_angle_quaternion(const Vector3d& dtheta){
        double angle = dtheta.norm();
        if (angle < 1e-5){
            return Quaterniond(1, 0, 0, 0);
        }
        else{
            return Quaterniond(AngleAxisd(angle, dtheta.normalized()));
        }
    }

private:
    Vector12d state_;
    Matrix12d covariance_;
    Matrix12d jacobian_matrix_;
    MatrixHd observation_matrix_;
    Matrix12d process_noise_;
    Matrix3d measurement_noise_;
    Quaterniond quat_offset_nominal_;
};

}

#endif // KALMAN_FILTER_HPP_