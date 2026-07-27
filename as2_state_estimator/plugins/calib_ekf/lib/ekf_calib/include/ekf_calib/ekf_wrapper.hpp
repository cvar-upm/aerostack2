// Copyright 2025 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file ekf_wrapper.hpp
*
* An EKF Wrapper implementation
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef EKF_CALIB__EKF_WRAPPER_HPP
#define EKF_CALIB__EKF_WRAPPER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <vector>

#include <ekf_calib/calib_ekf_c_code.h>

#include "Eigen/src/Core/Matrix.h"
#include "ekf_calib/ekf_datatype.hpp"

namespace calib_ekf_math
{

/**
 * @brief EKFData
 *
 * Data structure to hold the EKF data.
 */
struct EKFData
{
  State state; // Current state of the EKF
  Covariance covariance; // Current covariance of the EKF
  Gravity gravity; // Gravity vector
  Eigen::Matrix4d map_to_odom; // Transformation matrix from map to odometry frame
  Eigen::Vector3d map_to_odom_velocity; // Velocity of the map to odom frame
};

/**
 * @brief A single observed landmark: a known map point seen at a pixel.
 */
struct LandmarkObservation
{
  Eigen::Vector3d p_world;              ///< Landmark position, map frame
  Eigen::Vector2d pixel;                ///< Measured pixel (u, v)
  double sigma_px = 1.0;                ///< Pixel noise standard deviation
};

/**
 * @brief Gating configuration for a points update.
 */
struct PointsUpdateConfig
{
  std::size_t max_landmarks = 16;   ///< Cap on landmarks used in one update
  double chi2_threshold = 5.991;    ///< Mahalanobis gate, 95% for 2 dof
  double min_depth = 0.2;           ///< Reject landmarks closer than this (m)
  double image_width = 0.0;         ///< Image bounds check; 0 disables
  double image_height = 0.0;
  double pixel_margin = 0.0;        ///< Slack around the image bounds (px)
};

/**
 * @brief Outcome of a points update.
 */
struct PointsUpdateResult
{
  bool applied = false;         ///< False if nothing survived gating or S was singular
  std::size_t n_used = 0;       ///< Landmarks fused
  std::size_t n_rejected = 0;   ///< Landmarks gated out
};

/**
 * @brief EKFWrapper
 *
 * Class to wrap the EKF functionality.
 */
class EKFWrapper
{
public:
  /**
   * @brief Default constructor for EKFWrapper
   */
  EKFWrapper();

  /**
   * @brief Constructor for EKFWrapper with initial state and covariance.
   *
   * @param initial_state (State) The initial state vector.
   * @param initial_covariance (Covariance) The initial covariance matrix.
   * @param imu_noise (Eigen::Vector<double, 6>) The IMU noise vector.
   * @param accelerometer_noise_density (double) The accelerometer noise density.
   * @param gyroscope_noise_density (double) The gyroscope noise density.
   * @param accelerometer_random_walk (double) The accelerometer random walk.
   * @param gyroscope_random_walk (double) The gyroscope random walk.
   */
  EKFWrapper(
    State initial_state,
    Covariance initial_covariance,
    Eigen::Vector<double, 6> imu_noise,
    double accelerometer_noise_density,
    double gyroscope_noise_density,
    double accelerometer_random_walk,
    double gyroscope_random_walk);

  /**
   * @brief EKFWrapper destructor
   */
  ~EKFWrapper();


  /**
   * @brief Reset the EKF with a new state and covariance.
   *
   * @param initial_state (State) The new initial state vector.
   * @param initial_covariance (Covariance) The new initial covariance.
   */
  void reset(
    const State & initial_state,
    const Covariance & initial_covariance);


  /**
   * @brief Set the IMU noise parameters.
   * @param imu_noise (Eigen::Vector<double, 6>) The IMU noise vector.
   * @param accelerometer_noise_density (double) The accelerometer noise density.
   * @param gyroscope_noise_density (double) The gyroscope noise density.
   * @param accelerometer_random_walk (double) The accelerometer random walk.
   * @param gyroscope_random_walk (double) The gyroscope random walk.
   * */
  void set_noise_parameters(
    const Eigen::Vector<double, 6> & imu_noise,
    double accelerometer_noise_density,
    double gyroscope_noise_density,
    double accelerometer_random_walk,
    double gyroscope_random_walk);


  /**
   * @brief Set the random walk parameters of the camera calibration states.
   *
   * Setting a value to zero freezes the corresponding states exactly: with no
   * process noise and no initial covariance, their rows of the Kalman gain are
   * identically zero.
   *
   * @param extrinsic_translation_random_walk (double) Lever arm random walk.
   * @param extrinsic_rotation_random_walk (double) Mount angle random walk.
   * @param intrinsics_focal_random_walk (double) Focal length random walk.
   * @param intrinsics_center_random_walk (double) Principal point random walk.
   */
  void set_calibration_noise_parameters(
    double extrinsic_translation_random_walk,
    double extrinsic_rotation_random_walk,
    double intrinsics_focal_random_walk,
    double intrinsics_center_random_walk);


  /**
   * @brief Set gravity vector.
   * @param gravity (Gravity) The gravity vector.
   */
  void set_gravity(const Gravity & gravity);


  /**
   * @brief Set map to odom transformation.
   * @param map_to_odom (Eigen::Matrix4d) The transformation matrix from map to odometry frame.
   */
  void set_map_to_odom(const Eigen::Matrix4d & map_to_odom);

  /**
   * @brief Set map to odom velocity.
   * @param map_to_odom_velocity (Eigen::Vector3d) The velocity of the map to odom frame.
   */
  void set_map_to_odom_velocity(const Eigen::Vector3d & map_to_odom_velocity);

  /**
   * @brief Get the current state.
   *
   * @return The current state vector.
   */
  State get_state();

  /**
   * @brief Set the current state.
   *
   * @param state (State) The new state vector.
   */
  void set_state(const State & state);


  /**
   * @brief Get the current state covariance.
   *
   * @return The current state covariance matrix.
   */
  Covariance get_state_covariance();


  /**
   * @brief Get the current map to odom transformation.
   *
   * @return The current map to odom transformation matrix.
   */
  Eigen::Matrix4d get_map_to_odom();


  /**
   * #brief Get the current map to odom velocity.
   * @return The current map to odom velocity vector.
   */
  Eigen::Vector3d get_map_to_odom_velocity();


  /**
     * @brief Get the gravity vector.
     *
     * @return The gravity vector.
     */
  Gravity get_gravity();


  /**
   * @brief Get the IMU noise vector.
   *
   * @return The IMU noise vector.
   */
  Eigen::Vector<double, 6> get_imu_noise();


  /**
   * @brief Get the noise parameters.
   *
   * @return The noise parameters as a vector.
   */
  Eigen::Vector<double, 4> get_noise_parameters();


  /**
   * @brief Compute the process noise covariance matrix.
   *
   * @param dt (double) The time step.
   * @return The process noise covariance matrix.
   */
  Covariance compute_process_noise_covariance(double dt);


  /**
   * @brief Pose to transform.
   * @param position (Eigen::Vector3d) The position vector.
   * @param orientation (Eigen::Quaterniond) The orientation.
   * @return The transformation matrix.
   */
  static Eigen::Matrix4d pose_to_transform(
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation);


  /**
   * @brief Quaternion rate Jacobian G(q), such that q_dot = 0.5 * G(q) * omega.
   *
   * Used both to build the quaternion block of the process noise and to map a
   * roll/pitch/yaw measurement covariance onto the quaternion measurement.
   *
   * @param q (Eigen::Quaterniond) The quaternion (need not be normalized).
   * @return The 4x3 Jacobian, rows ordered (qw, qx, qy, qz).
   */
  static Eigen::Matrix<double, 4, 3> quaternion_rate_jacobian(const Eigen::Quaterniond & q);


  /**
   * @brief Compute map to odom transformation.
   * @param state (State) The current state vector.
   * @param new_state (State) The new state vector.
   * @param prev_map_to_odom (Eigen::Matrix4d) The previous map to odom transformation matrix.
   * @return The new map to odom transformation matrix.
   */
  static Eigen::Matrix4d compute_map_to_odom(
    const State & state,
    const State & new_state,
    const Eigen::Matrix4d & prev_map_to_odom);
  
  /**
   * @brief Compute map to odom velocity.
   * @param state (State) The current state vector.
   * @param new_state (State) The new state vector.
   * @param prev_map_to_odom_velocity (Eigen::Vector3d) The previous map to odom velocity vector.
   * @return The new map to odom velocity vector.
   */
  static Eigen::Vector3d compute_map_to_odom_velocity(
    const State & state,
    const State & new_state,
    const Eigen::Vector3d & prev_map_to_odom_velocity);


  /**
   * @brief Predict the next state.
   *
   * @param imu_measurement (Input) The IMU measurement vector.
   * @param dt (double) The time step.
   */
  void predict(
    const Input & imu_measurement,
    const double & dt);


  /**
   * @brief Update the state with a new pose measurement.
   *
   * @param z (PoseMeasurement) The measurement (pose) vector.
   * @param measurement_noise_covariance (PoseMeasurementCovariance) The measurement noise covariance matrix.
   */
  void update_pose(
    const PoseMeasurement & z,
    const PoseMeasurementCovariance & measurement_noise_covariance);


  /**
   * @brief Update the state with a new pose measurement from odometry.
   *
   * @param z (PoseMeasurement) The measurement (pose) vector.
   * @param measurement_noise_covariance (PoseMeasurementCovariance) The measurement noise covariance matrix.
   */
  void update_pose_odom(
    const PoseMeasurement & z,
    const PoseMeasurementCovariance & measurement_noise_covariance);


  /**
   * @brief Update the state with a new velocity measurement.
   *
   * @param z (VelocityMeasurement) The measurement (velocity) vector.
   * @param measurement_noise_covariance (VelocityMeasurementCovariance) The measurement noise covariance matrix.
   */
  void update_velocity(
    const VelocityMeasurement & z,
    const VelocityMeasurementCovariance & measurement_noise_covariance);


  /**
   * @brief Update the state with a set of observed landmarks.
   *
   * Each landmark is gated individually (depth, image bounds, chi-square on a
   * 2x2 residual covariance) and the survivors are fused in a single batch.
   * This is the only measurement that touches the calibration states: the
   * calibration block is decoupled in F, so without it those states would only
   * ever grow their covariance through the random walk.
   *
   * @param observations The observed landmarks.
   * @param config Gating configuration.
   * @return Which landmarks were used, and whether a correction was applied.
   */
  PointsUpdateResult update_points(
    const std::vector<LandmarkObservation> & observations,
    const PointsUpdateConfig & config);


  /**
   * @brief Set the nominal camera mount rotation (camera -> body).
   *
   * The estimated extrinsic angles are deltas about this rotation, which keeps
   * them away from the gimbal lock of the Rz*Ry*Rx parameterisation. Defaults to
   * the identity.
   */
  void set_camera_nominal_rotation(const Eigen::Matrix3d & rotation);


  /**
   * @brief Get the nominal camera mount rotation.
   */
  Eigen::Matrix3d get_camera_nominal_rotation() const;


  /**
   * @brief Renormalize the state quaternion and propagate the normalization
   *        through the covariance.
   *
   * Replaces the Euler angle wrapping of the roll/pitch/yaw formulation. The
   * normalization Jacobian is applied to P so that the covariance stays
   * consistent with the unit norm constraint.
   */
  void normalize_quaternion_state();

private:
  EKFData ekf_data_;   // EKF data structure
  Eigen::Vector<double, 6> imu_noise_;   // IMU noise vector
  double accelerometer_noise_density_;   // Accelerometer noise density
  double gyroscope_noise_density_;   // Gyroscope noise density
  double accelerometer_random_walk_;   // Accelerometer random walk
  double gyroscope_random_walk_;   // Gyroscope random walk
  double extrinsic_translation_random_walk_;   // Camera lever arm random walk
  double extrinsic_rotation_random_walk_;   // Camera mount angle random walk
  double intrinsics_focal_random_walk_;   // Focal length random walk
  double intrinsics_center_random_walk_;   // Principal point random walk
  Eigen::Matrix3d camera_nominal_rotation_;   // Nominal mount rotation, camera -> body

  /**
   * @brief Apply a linear measurement correction to the state and covariance.
   *
   * Solves K = P H' S^-1 through a Cholesky factorization of S and applies the
   * Joseph form of the covariance update. The Joseph form is required rather
   * than merely preferable here: the quaternion makes the attitude block of P
   * structurally rank deficient, and (I - K H) P does not keep such a P
   * symmetric positive semi-definite.
   *
   * @param H (Eigen::MatrixXd) The measurement Jacobian, m x State::size.
   * @param y (Eigen::VectorXd) The measurement residual z - h(x), length m.
   * @param R (Eigen::MatrixXd) The measurement noise covariance, m x m.
   * @return True if the correction was applied. When S cannot be factorized the
   *         state and covariance are left untouched and false is returned.
   */
  bool apply_correction(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & y,
    const Eigen::MatrixXd & R);
};

} // namespace calib_ekf_math

#endif // EKF_CALIB__EKF_WRAPPER_HPP
