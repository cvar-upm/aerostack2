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

#include "ekf_calib/ekf_wrapper.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ekf_calib/ekf_datatype.hpp"
#include <algorithm>

namespace calib_ekf_math
{


EKFWrapper::EKFWrapper()
{
  // Initialize the EKF data
  ekf_data_ = EKFData();
  ekf_data_.map_to_odom = Eigen::Matrix4d::Identity();
  ekf_data_.map_to_odom_velocity = Eigen::Vector3d::Zero();
  imu_noise_ = Eigen::Vector<double, 6>::Zero();
  accelerometer_noise_density_ = 0.0;
  gyroscope_noise_density_ = 0.0;
  accelerometer_random_walk_ = 0.0;
  gyroscope_random_walk_ = 0.0;
  extrinsic_translation_random_walk_ = 0.0;
  extrinsic_rotation_random_walk_ = 0.0;
  intrinsics_focal_random_walk_ = 0.0;
  intrinsics_center_random_walk_ = 0.0;
  camera_nominal_rotation_ = Eigen::Matrix3d::Identity();
}


EKFWrapper::EKFWrapper(
  State initial_state,
  Covariance initial_covariance,
  Eigen::Vector<double, 6> imu_noise,
  double accelerometer_noise_density,
  double gyroscope_noise_density,
  double accelerometer_random_walk,
  double gyroscope_random_walk)
{
  // Initialize the EKF data with provided parameters
  ekf_data_ = EKFData();
  ekf_data_.state = initial_state;
  ekf_data_.covariance = initial_covariance;
  ekf_data_.map_to_odom = Eigen::Matrix4d::Identity();
  ekf_data_.map_to_odom_velocity = Eigen::Vector3d::Zero();
  imu_noise_ = imu_noise;
  accelerometer_noise_density_ = accelerometer_noise_density;
  gyroscope_noise_density_ = gyroscope_noise_density;
  accelerometer_random_walk_ = accelerometer_random_walk;
  gyroscope_random_walk_ = gyroscope_random_walk;
  extrinsic_translation_random_walk_ = 0.0;
  extrinsic_rotation_random_walk_ = 0.0;
  intrinsics_focal_random_walk_ = 0.0;
  intrinsics_center_random_walk_ = 0.0;
  camera_nominal_rotation_ = Eigen::Matrix3d::Identity();
}


EKFWrapper::~EKFWrapper()
{
  // Destructor logic if needed
}


void EKFWrapper::reset(
  const State & initial_state,
  const Covariance & initial_covariance)
{
  ekf_data_.state = initial_state;
  ekf_data_.covariance = initial_covariance;
}


void EKFWrapper::set_noise_parameters(
  const Eigen::Vector<double, 6> & imu_noise,
  double accelerometer_noise_density,
  double gyroscope_noise_density,
  double accelerometer_random_walk,
  double gyroscope_random_walk)
{
  imu_noise_ = imu_noise;
  accelerometer_noise_density_ = accelerometer_noise_density;
  gyroscope_noise_density_ = gyroscope_noise_density;
  accelerometer_random_walk_ = accelerometer_random_walk;
  gyroscope_random_walk_ = gyroscope_random_walk;
}


void EKFWrapper::set_calibration_noise_parameters(
  double extrinsic_translation_random_walk,
  double extrinsic_rotation_random_walk,
  double intrinsics_focal_random_walk,
  double intrinsics_center_random_walk)
{
  extrinsic_translation_random_walk_ = extrinsic_translation_random_walk;
  extrinsic_rotation_random_walk_ = extrinsic_rotation_random_walk;
  intrinsics_focal_random_walk_ = intrinsics_focal_random_walk;
  intrinsics_center_random_walk_ = intrinsics_center_random_walk;
}


void EKFWrapper::set_camera_nominal_rotation(const Eigen::Matrix3d & rotation)
{
  camera_nominal_rotation_ = rotation;
}


Eigen::Matrix3d EKFWrapper::get_camera_nominal_rotation() const
{
  return camera_nominal_rotation_;
}


void EKFWrapper::set_gravity(const Gravity & gravity)
{
  ekf_data_.gravity = gravity;
}


void EKFWrapper::set_map_to_odom(const Eigen::Matrix4d & map_to_odom)
{
  ekf_data_.map_to_odom = map_to_odom;
}


void EKFWrapper::set_map_to_odom_velocity(const Eigen::Vector3d & map_to_odom_velocity)
{
  ekf_data_.map_to_odom_velocity = map_to_odom_velocity;
}


State EKFWrapper::get_state()
{
  return ekf_data_.state;
}

void EKFWrapper::set_state(const State & state)
{
  ekf_data_.state.set(state.data);
}


Covariance EKFWrapper::get_state_covariance()
{
  return ekf_data_.covariance;
}


Eigen::Matrix4d EKFWrapper::get_map_to_odom()
{
  return ekf_data_.map_to_odom;
}


Eigen::Vector3d EKFWrapper::get_map_to_odom_velocity()
{
  return ekf_data_.map_to_odom_velocity;
}


Gravity EKFWrapper::get_gravity()
{
  return ekf_data_.gravity;
}


Eigen::Vector<double, 6> EKFWrapper::get_imu_noise()
{
  return imu_noise_;
}


Eigen::Vector<double, 4> EKFWrapper::get_noise_parameters()
{
  return Eigen::Vector<double, 4>(
    accelerometer_noise_density_,
    gyroscope_noise_density_,
    accelerometer_random_walk_,
    gyroscope_random_walk_);
}


Covariance EKFWrapper::compute_process_noise_covariance(
  double dt)
{
  Eigen::Matrix<double, Covariance::rows, Covariance::cols> process_noise_covariance =
    Eigen::Matrix<double, Covariance::rows, Covariance::cols>::Zero();
  Eigen::Matrix3d q_pp =
    pow(accelerometer_noise_density_, 2) *
    pow(dt, 3) /
    3.0 *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_pv =
    pow(accelerometer_noise_density_, 2) *
    pow(dt, 2) /
    2.0 *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_vv =
    pow(accelerometer_noise_density_, 2) *
    dt *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_baba =
    pow(accelerometer_random_walk_, 2) *
    dt *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_bwbw =
    pow(gyroscope_random_walk_, 2) *
    dt *
    Eigen::Matrix3d::Identity();

  // Gyroscope white noise mapped onto the quaternion. Since q_dot = 0.5 G(q) w,
  // the 4x4 attitude block is 0.25 sigma_g^2 dt G(q) G(q)'. For a unit
  // quaternion G G' = I - q q', so this injects no variance along q itself,
  // which is what keeps P consistent with the unit norm constraint.
  const std::array<double, 4> q_arr = ekf_data_.state.get_orientation_quaternion();
  const Eigen::Quaterniond q(q_arr[3], q_arr[0], q_arr[1], q_arr[2]);
  const Eigen::Matrix<double, 4, 3> G = quaternion_rate_jacobian(q);
  Eigen::Matrix4d q_qq =
    0.25 *
    pow(gyroscope_noise_density_, 2) *
    dt *
    (G * G.transpose());

  process_noise_covariance.block<3, 3>(State::X, State::X) = q_pp;
  process_noise_covariance.block<3, 3>(State::X, State::VX) = q_pv;
  process_noise_covariance.block<3, 3>(State::VX, State::X) = q_pv;
  process_noise_covariance.block<3, 3>(State::VX, State::VX) = q_vv;
  process_noise_covariance.block<4, 4>(State::QW, State::QW) = q_qq;
  process_noise_covariance.block<3, 3>(State::ABX, State::ABX) = q_baba;
  process_noise_covariance.block<3, 3>(State::WBX, State::WBX) = q_bwbw;

  // Camera calibration random walks. A zero parameter freezes those states
  // exactly, which is how the calibration is staged during bring-up.
  process_noise_covariance.block<3, 3>(State::EX, State::EX) =
    pow(extrinsic_translation_random_walk_, 2) * dt * Eigen::Matrix3d::Identity();
  process_noise_covariance.block<3, 3>(State::EPHI, State::EPHI) =
    pow(extrinsic_rotation_random_walk_, 2) * dt * Eigen::Matrix3d::Identity();
  process_noise_covariance.block<2, 2>(State::FX, State::FX) =
    pow(intrinsics_focal_random_walk_, 2) * dt * Eigen::Matrix2d::Identity();
  process_noise_covariance.block<2, 2>(State::CX, State::CX) =
    pow(intrinsics_center_random_walk_, 2) * dt * Eigen::Matrix2d::Identity();
  Covariance pnc = Covariance();
  std::array<double, Covariance::size> process_noise_covariance_array;
  for (std::size_t i = 0; i < Covariance::size; ++i) {
    process_noise_covariance_array[i] =
      process_noise_covariance(i / Covariance::cols, i % Covariance::cols);
  }
  pnc.set(process_noise_covariance_array);
  return pnc;
}


Eigen::Matrix<double, 4, 3> EKFWrapper::quaternion_rate_jacobian(const Eigen::Quaterniond & q)
{
  Eigen::Matrix<double, 4, 3> G;
  G << -q.x(), -q.y(), -q.z(),
    q.w(), -q.z(), q.y(),
    q.z(), q.w(), -q.x(),
    -q.y(), q.x(), q.w();
  return G;
}


Eigen::Matrix4d EKFWrapper::pose_to_transform(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
  transform(0, 3) = position[0];
  transform(1, 3) = position[1];
  transform(2, 3) = position[2];
  return transform;
}


Eigen::Matrix4d EKFWrapper::compute_map_to_odom(
  const State & state,
  const State & new_state,
  const Eigen::Matrix4d & prev_map_to_odom)
{
  // get_orientation_quaternion() returns (qx, qy, qz, qw); reading the
  // quaternion directly avoids a quaternion -> Euler -> quaternion round trip.
  const std::array<double, 4> q_prev_arr = state.get_orientation_quaternion();
  const std::array<double, 4> q_new_arr = new_state.get_orientation_quaternion();
  Eigen::Quaterniond q_prev(q_prev_arr[3], q_prev_arr[0], q_prev_arr[1], q_prev_arr[2]);
  Eigen::Quaterniond q_new(q_new_arr[3], q_new_arr[0], q_new_arr[1], q_new_arr[2]);

  Eigen::Vector3d p_prev = Eigen::Vector3d(state.get_position().data());
  Eigen::Vector3d p_new = Eigen::Vector3d(new_state.get_position().data());

  Eigen::Matrix4d T_map_base_prev =
    pose_to_transform(p_prev, q_prev);
  Eigen::Matrix4d T_base_map_prev = T_map_base_prev.inverse();
  Eigen::Matrix4d T_map_base_new =
    pose_to_transform(p_new, q_new);
  Eigen::Matrix4d delta = T_map_base_new * T_base_map_prev;

  Eigen::Matrix4d T_map_odom_new = delta * prev_map_to_odom;
  return T_map_odom_new;
}


Eigen::Vector3d EKFWrapper::compute_map_to_odom_velocity(
  const State & state,
  const State & new_state,
  const Eigen::Vector3d & prev_map_to_odom_velocity)
{
  Eigen::Vector3d v_prev = Eigen::Vector3d(state.get_velocity().data());
  Eigen::Vector3d v_new = Eigen::Vector3d(new_state.get_velocity().data());

  Eigen::Vector3d delta_v = v_new - v_prev;
  Eigen::Vector3d map_to_odom_velocity_new = prev_map_to_odom_velocity + delta_v;
  return map_to_odom_velocity_new;
}


void EKFWrapper::normalize_quaternion_state()
{
  double * q = ekf_data_.state.data.data() + State::QW;
  const double norm = std::sqrt(
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm <= 0.0 || !std::isfinite(norm)) {
    // Unrecoverable attitude: fall back to identity rather than propagate NaN.
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
    return;
  }

  const double inv_norm = 1.0 / norm;
  Eigen::Vector4d q_hat;
  q_hat << q[0] * inv_norm, q[1] * inv_norm, q[2] * inv_norm, q[3] * inv_norm;

  for (int i = 0; i < 4; ++i) {
    q[i] = q_hat[i];
  }

  // Propagate the normalization through the covariance. J = d(q/|q|)/dq is the
  // projector that removes the component along q, scaled by 1/|q|. Without this
  // the covariance would keep claiming variance along a direction the state can
  // no longer move in.
  Eigen::Matrix<double, State::size, State::size> T =
    Eigen::Matrix<double, State::size, State::size>::Identity();
  T.block<4, 4>(State::QW, State::QW) =
    inv_norm * (Eigen::Matrix4d::Identity() - q_hat * q_hat.transpose());

  Eigen::Map<Eigen::Matrix<double, Covariance::rows, Covariance::cols,
    Eigen::RowMajor>> P(ekf_data_.covariance.data.data());
  P = T * P * T.transpose();
  P = 0.5 * (P + P.transpose()).eval();
}


bool EKFWrapper::apply_correction(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & y,
  const Eigen::MatrixXd & R)
{
  Eigen::Map<Eigen::Matrix<double, Covariance::rows, Covariance::cols,
    Eigen::RowMajor>> P(ekf_data_.covariance.data.data());

  const Eigen::MatrixXd PHt = P * H.transpose();          // n x m
  const Eigen::MatrixXd S = H * PHt + R;                  // m x m

  Eigen::LLT<Eigen::MatrixXd> llt(S);
  if (llt.info() != Eigen::Success) {
    return false;
  }

  // K = P H' S^-1, obtained as (S^-1 (P H')')' so that S is never inverted.
  const Eigen::MatrixXd K = llt.solve(PHt.transpose()).transpose();   // n x m

  Eigen::Map<Eigen::Matrix<double, State::size, 1>> x(ekf_data_.state.data.data());
  x += K * y;

  // Joseph form: keeps P symmetric positive semi-definite even though the
  // quaternion makes the attitude block structurally rank deficient.
  const Eigen::MatrixXd IKH =
    Eigen::Matrix<double, State::size, State::size>::Identity() - K * H;
  P = IKH * P * IKH.transpose() + K * R * K.transpose();
  P = 0.5 * (P + P.transpose()).eval();
  return true;
}


void EKFWrapper::predict(
  const Input & input,
  const double & dt)
{
  const Covariance process_noise_covariance =
    compute_process_noise_covariance(dt);

  // CasADi writes dense matrices in column-major order.
  double f_out[State::size];
  double F_out[State::size * State::size];
  double L_out[State::size * Input::size];

  const double * arg[calib_predict_function_SZ_ARG] = {
    ekf_data_.state.data.data(),
    input.data.data(),
    &dt,
    ekf_data_.gravity.data.data()
  };
  double * res[calib_predict_function_SZ_RES] = {f_out, F_out, L_out};

  calib_predict_function(arg, res, nullptr, nullptr, 0);

  Eigen::Map<const Eigen::Matrix<double, State::size, State::size,
    Eigen::ColMajor>> F(F_out);
  Eigen::Map<Eigen::Matrix<double, Covariance::rows, Covariance::cols,
    Eigen::RowMajor>> P(ekf_data_.covariance.data.data());
  Eigen::Map<const Eigen::Matrix<double, Covariance::rows, Covariance::cols,
    Eigen::RowMajor>> Q(process_noise_covariance.data.data());

  P = F * P * F.transpose() + Q;
  for (std::size_t i = 0; i < State::size; ++i) {
    ekf_data_.state.data[i] = f_out[i];
  }

  normalize_quaternion_state();
}


void EKFWrapper::update_pose(
  const PoseMeasurement & z,
  const PoseMeasurementCovariance & measurement_noise_covariance)
{
  const State prev_state = get_state();

  update_pose_odom(z, measurement_noise_covariance);

  // Update the map to odom Transformation
  set_map_to_odom(
    compute_map_to_odom(
      prev_state,
      get_state(),
      get_map_to_odom()));
  // Update the map to odom Velocity
  set_map_to_odom_velocity(
    compute_map_to_odom_velocity(
      prev_state,
      get_state(),
      get_map_to_odom_velocity()));
}


void EKFWrapper::update_pose_odom(
  const PoseMeasurement & z,
  const PoseMeasurementCovariance & measurement_noise_covariance)
{
  double h_out[PoseMeasurement::size];
  double H_out[PoseMeasurement::size * State::size];

  const double * arg[calib_pose_function_SZ_ARG] = {ekf_data_.state.data.data()};
  double * res[calib_pose_function_SZ_RES] = {h_out, H_out};
  calib_pose_function(arg, res, nullptr, nullptr, 0);

  Eigen::Map<const Eigen::Matrix<double, PoseMeasurement::size, State::size,
    Eigen::ColMajor>> H(H_out);

  Eigen::VectorXd y(PoseMeasurement::size);
  for (std::size_t i = 0; i < PoseMeasurement::size; ++i) {
    y[i] = z.data[i] - h_out[i];
  }

  // Map the roll/pitch/yaw variances onto the quaternion block through the same
  // G(q) used for the process noise. The resulting R is not diagonal, which is
  // only representable now that the gain is computed here rather than baked into
  // the generated code.
  const std::array<double, 4> q_arr = ekf_data_.state.get_orientation_quaternion();
  const Eigen::Quaterniond q(q_arr[3], q_arr[0], q_arr[1], q_arr[2]);
  const Eigen::Matrix<double, 4, 3> G = quaternion_rate_jacobian(q);
  Eigen::Vector3d rpy_var;
  rpy_var << measurement_noise_covariance.data[PoseMeasurementCovariance::ROLL],
    measurement_noise_covariance.data[PoseMeasurementCovariance::PITCH],
    measurement_noise_covariance.data[PoseMeasurementCovariance::YAW];

  // G G' = I - q q', so G * diag(rpy_var) * G' carries no variance along q and is
  // rank 3. The attitude block of P is rank deficient for the same reason, which
  // would leave S = H P H' + R singular and unfactorizable. Adding the q q'
  // direction back at the same scale makes R full rank, and exactly isotropic
  // when the three variances are equal. This is physically inert: the residual
  // between two same-hemisphere unit quaternions has no component along q, and
  // normalize_quaternion_state() projects out any motion along q afterwards.
  Eigen::Vector4d q_vec;
  q_vec << q.w(), q.x(), q.y(), q.z();
  q_vec.normalize();
  const double norm_var = rpy_var.mean();

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(PoseMeasurement::size, PoseMeasurement::size);
  R(0, 0) = measurement_noise_covariance.data[PoseMeasurementCovariance::X];
  R(1, 1) = measurement_noise_covariance.data[PoseMeasurementCovariance::Y];
  R(2, 2) = measurement_noise_covariance.data[PoseMeasurementCovariance::Z];
  R.block<4, 4>(3, 3) =
    0.25 * (G * rpy_var.asDiagonal() * G.transpose() + norm_var * q_vec * q_vec.transpose());

  apply_correction(H, y, R);
  normalize_quaternion_state();
}


PointsUpdateResult EKFWrapper::update_points(
  const std::vector<LandmarkObservation> & observations,
  const PointsUpdateConfig & config)
{
  PointsUpdateResult result;

  const std::size_t n_max = std::min(observations.size(), config.max_landmarks);
  if (n_max == 0) {
    return result;
  }

  Eigen::Map<const Eigen::Matrix<double, Covariance::rows, Covariance::cols,
    Eigen::RowMajor>> P(ekf_data_.covariance.data.data());

  // R_nom is passed to the generated code column-major, which for a rotation
  // matrix means the transpose unless we say so explicitly.
  Eigen::Matrix<double, 3, 3, Eigen::ColMajor> R_nom = camera_nominal_rotation_;

  std::vector<Eigen::Matrix<double, 2, State::size>> H_rows;
  std::vector<Eigen::Vector2d> residuals;
  std::vector<double> sigmas;
  H_rows.reserve(n_max);
  residuals.reserve(n_max);
  sigmas.reserve(n_max);

  for (std::size_t i = 0; i < n_max; ++i) {
    const LandmarkObservation & obs = observations[i];

    double h_out[2];
    double H_out[2 * State::size];
    double depth = 0.0;
    const double * arg[calib_point_function_SZ_ARG] = {
      ekf_data_.state.data.data(), obs.p_world.data(), R_nom.data()};
    double * res[calib_point_function_SZ_RES] = {h_out, H_out, &depth};
    calib_point_function(arg, res, nullptr, nullptr, 0);

    // Mandatory: the projection divides by depth with no guard, so a landmark
    // at or behind the camera would produce an infinite or sign-flipped pixel.
    if (!(depth > config.min_depth)) {
      ++result.n_rejected;
      continue;
    }
    if (!std::isfinite(h_out[0]) || !std::isfinite(h_out[1])) {
      ++result.n_rejected;
      continue;
    }
    if (config.image_width > 0.0 && config.image_height > 0.0) {
      const double m = config.pixel_margin;
      if (h_out[0] < -m || h_out[0] > config.image_width + m ||
        h_out[1] < -m || h_out[1] > config.image_height + m)
      {
        ++result.n_rejected;
        continue;
      }
    }

    Eigen::Map<const Eigen::Matrix<double, 2, State::size, Eigen::ColMajor>> H_i(H_out);
    const Eigen::Vector2d r_i(obs.pixel[0] - h_out[0], obs.pixel[1] - h_out[1]);

    // Individual chi-square gate on a 2x2 residual covariance.
    const Eigen::Matrix2d S_i =
      H_i * P * H_i.transpose() +
      obs.sigma_px * obs.sigma_px * Eigen::Matrix2d::Identity();
    const double d2 = r_i.transpose() * S_i.inverse() * r_i;
    if (!std::isfinite(d2) || d2 > config.chi2_threshold) {
      ++result.n_rejected;
      continue;
    }

    H_rows.push_back(H_i);
    residuals.push_back(r_i);
    sigmas.push_back(obs.sigma_px);
  }

  const std::size_t n = H_rows.size();
  if (n == 0) {
    return result;
  }

  // Fuse the survivors in a single batch.
  const std::size_t m = 2 * n;
  Eigen::MatrixXd H(m, State::size);
  Eigen::VectorXd y(m);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
  for (std::size_t i = 0; i < n; ++i) {
    H.middleRows<2>(2 * i) = H_rows[i];
    y.segment<2>(2 * i) = residuals[i];
    const double var = sigmas[i] * sigmas[i];
    R(2 * i, 2 * i) = var;
    R(2 * i + 1, 2 * i + 1) = var;
  }

  if (!apply_correction(H, y, R)) {
    return result;
  }
  normalize_quaternion_state();

  result.applied = true;
  result.n_used = n;
  return result;
}


void EKFWrapper::update_velocity(
  const VelocityMeasurement & z,
  const VelocityMeasurementCovariance & measurement_noise_covariance)
{
  double h_out[VelocityMeasurement::size];
  double H_out[VelocityMeasurement::size * State::size];

  const double * arg[calib_velocity_function_SZ_ARG] = {ekf_data_.state.data.data()};
  double * res[calib_velocity_function_SZ_RES] = {h_out, H_out};
  calib_velocity_function(arg, res, nullptr, nullptr, 0);

  Eigen::Map<const Eigen::Matrix<double, VelocityMeasurement::size, State::size,
    Eigen::ColMajor>> H(H_out);

  Eigen::VectorXd y(VelocityMeasurement::size);
  for (std::size_t i = 0; i < VelocityMeasurement::size; ++i) {
    y[i] = z.data[i] - h_out[i];
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(
    VelocityMeasurement::size, VelocityMeasurement::size);
  for (std::size_t i = 0; i < VelocityMeasurement::size; ++i) {
    R(i, i) = measurement_noise_covariance.data[i];
  }

  apply_correction(H, y, R);
  normalize_quaternion_state();
}


}  // namespace calib_ekf_math
