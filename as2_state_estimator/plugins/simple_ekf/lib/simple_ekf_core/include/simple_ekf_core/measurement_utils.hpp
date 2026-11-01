// Copyright 2024 Universidad Politécnica de Madrid
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
* @file measurement_utils.hpp
*
* What a measurement goes through between arriving and being believed: its covariance,
* the components nobody measured, the angles' branch and the innovation gate
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__MEASUREMENT_UTILS_HPP_
#define SIMPLE_EKF_CORE__MEASUREMENT_UTILS_HPP_

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <cstddef>

#include "ekf/ekf_datatype.hpp"

#include "simple_ekf_core/rigid.hpp"
#include "simple_ekf_core/types.hpp"

namespace simple_ekf_core
{

/// The state each position component of a pose measurement measures, in measurement order.
/// The orientation is measured as a quaternion and compared as a rotation: see
/// @ref poseInnovations
inline constexpr std::array<int, 3> kPositionStateIndices = {
  ekf::State::X, ekf::State::Y, ekf::State::Z};

/// The state each component of a velocity measurement measures, in measurement order
inline constexpr std::array<int, 3> kVelocityStateIndices = {
  ekf::State::VX, ekf::State::VY, ekf::State::VZ};

/// Flat index in ekf::Covariance of the variance of a state
constexpr int varianceIndex(int state_index)
{
  return state_index * (ekf::Covariance::cols + 1);
}

/// Flat index in a 6x6 row-major covariance of the variance of component `index`
constexpr std::size_t diagonalIndex(std::size_t index)
{
  return index * 6 + index;
}

/**
 * @brief The 6x6 covariance a source configured with fixed variances is believed with
 *
 * Diagonal, from `position_values` and `orientation_values`. All zeros when the source is
 * configured to use the covariance its measurements carry, since then there is nothing
 * fixed to give it.
 */
inline std::array<double, 36> generateCovarianceFromConfig(const SourceConfig & config)
{
  std::array<double, 36> covariance{};
  if (config.use_message_covariance) {
    return covariance;
  }

  for (std::size_t index = 0; index < 3; ++index) {
    covariance[diagonalIndex(index)] = config.position_values[index];
    covariance[diagonalIndex(index + 3)] = config.orientation_values[index];
  }
  return covariance;
}

/**
 * @brief The covariance a pose measurement is believed with, from what it carries
 *
 * With `use_message_covariance` the carried variances are scaled by the configured values;
 * without it they are replaced by them.
 *
 * A negative variance is the source saying it does not measure that component, so the
 * configured values never overwrite one: that would turn a component nobody measured into
 * a reading of whatever the message left in that field. A zero is not the same statement:
 * it is what a message that carries no covariance at all is full of, and supplying the
 * number it lacks is the whole point of `use_message_covariance: false`.
 */
inline std::array<double, 36> getCovarianceWithConfig(
  const std::array<double, 36> & input_covariance,
  const SourceConfig & config)
{
  if (!config.use_message_covariance) {
    std::array<double, 36> covariance = generateCovarianceFromConfig(config);
    for (std::size_t index = 0; index < 6; ++index) {
      if (input_covariance[diagonalIndex(index)] < 0.0) {
        covariance[diagonalIndex(index)] = input_covariance[diagonalIndex(index)];
      }
    }
    return covariance;
  }

  std::array<double, 36> covariance = input_covariance;
  for (std::size_t index = 0; index < 3; ++index) {
    covariance[diagonalIndex(index)] *= config.position_values[index];
    covariance[diagonalIndex(index + 3)] *= config.orientation_values[index];
  }
  return covariance;
}

/**
 * @brief The velocity counterpart of @ref getCovarianceWithConfig, applied to the linear
 *        block only: no correction reads the angular one.
 */
inline std::array<double, 36> getLinearCovarianceWithConfig(
  const std::array<double, 36> & input_covariance,
  const SourceConfig & config)
{
  std::array<double, 36> covariance = input_covariance;
  for (std::size_t index = 0; index < 3; ++index) {
    double & variance = covariance[diagonalIndex(index)];
    if (config.use_message_covariance) {
      variance *= config.linear_values[index];
    } else if (input_covariance[diagonalIndex(index)] >= 0.0) {
      variance = config.linear_values[index];
    }
  }
  return covariance;
}

/**
 * @brief A pose as the EKF measures it: position and the unit quaternion, w first
 *
 * The quaternion's sign is whatever the pose's rotation gives, not matched to any state:
 * see @ref unwrapPoseMeasurement.
 */
inline ekf::PoseMeasurement poseToRawEkfMeasurement(const Rigid & pose)
{
  Quaternion rotation = pose.getRotation();
  rotation.normalize();
  return ekf::PoseMeasurement(
    {
      pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
      rotation.w(), rotation.x(), rotation.y(), rotation.z()});
}

/**
 * @brief Put the measured quaternion on the state's hemisphere
 *
 * q and -q are the same rotation, but the residual 2 vec(q_meas x q_state^-1) is only the
 * small rotation between them when the two have a non-negative dot product.
 */
inline ekf::PoseMeasurement unwrapPoseMeasurement(
  const ekf::PoseMeasurement & raw,
  const ekf::State & reference_state)
{
  ekf::PoseMeasurement aligned = raw;
  double dot = 0.0;
  for (int i = 0; i < 4; ++i) {
    dot += raw.data[ekf::PoseMeasurement::QW + i] * reference_state.data[ekf::State::QW + i];
  }
  if (dot < 0.0) {
    for (int i = 0; i < 4; ++i) {
      aligned.data[ekf::PoseMeasurement::QW + i] = -raw.data[ekf::PoseMeasurement::QW + i];
    }
  }
  return aligned;
}

/**
 * @brief How the state's quaternion moves under a small rotation about the map's axes
 *
 * Exp(dtheta) x q = q + E dtheta / 2. For a unit q, E^T E = I and E^T q = 0: E's columns are
 * the three directions a unit quaternion can move in, and q itself, its length, is not one.
 */
inline Eigen::Matrix<double, 4, 3> rotationToQuaternionJacobian(const ekf::State & state)
{
  const auto [qx, qy, qz, qw] = state.get_orientation_quaternion();
  Eigen::Matrix<double, 4, 3> E;
  E << -qx, -qy, -qz,
    qw, qz, -qy,
    -qz, qw, qx,
    qy, -qx, qw;
  return E;
}

/**
 * @brief The state's orientation covariance as rotations about the map's x, y and z axes
 *
 * 4 E^T P_qq E (@ref rotationToQuaternionJacobian). Whatever P_qq holds along the quaternion
 * itself does not show here: nothing in the filter reads it.
 */
inline Eigen::Matrix3d orientationCovariance(
  const ekf::State & state,
  const ekf::Covariance & state_covariance)
{
  const Eigen::Matrix<double, 4, 3> E = rotationToQuaternionJacobian(state);
  const Eigen::Map<const Eigen::Matrix<double, ekf::Covariance::rows, ekf::Covariance::cols,
    Eigen::RowMajor>> P(state_covariance.data.data());
  return 4.0 * E.transpose() * P.block<4, 4>(ekf::State::QW, ekf::State::QW) * E;
}

/**
 * @brief The state covariance with its orientation carried from one quaternion to another
 *
 * The quaternion block holds rotations about the map's axes in the directions the quaternion
 * it was built for can move in. Overwriting the quaternion alone, as a reset to a pose does,
 * leaves the block describing the old orientation: from the identity to a yaw psi, the
 * variance about z would read s cos^2(psi / 2), none at psi = pi, and the rest would sit along
 * the new quaternion, where nothing reads it. T = E_to E_from^T keeps every orientation
 * variance and cross-covariance about the map's axes and moves it to the new directions.
 */
inline ekf::Covariance carryOrientationCovariance(
  const ekf::Covariance & state_covariance,
  const ekf::State & from,
  const ekf::State & to)
{
  using Matrix = Eigen::Matrix<double, ekf::Covariance::rows, ekf::Covariance::cols,
      Eigen::RowMajor>;
  Matrix T = Matrix::Identity();
  T.block<4, 4>(ekf::State::QW, ekf::State::QW) =
    rotationToQuaternionJacobian(to) * rotationToQuaternionJacobian(from).transpose();

  ekf::Covariance carried;
  Eigen::Map<Matrix>(carried.data.data()) =
    T * Eigen::Map<const Matrix>(state_covariance.data.data()) * T.transpose();
  return carried;
}

/**
 * @brief Each component of a pose measurement's innovation, and the state's variance along it
 *
 * Position: the difference and the position variances. Orientation: the rotation from the
 * state's quaternion to the measured one about the map's x, y and z axes,
 * 2 vec(q_meas x q_state^-1), the residual the update uses, and the state's variance about
 * those axes (@ref orientationCovariance).
 *
 * @param aligned Measurement already on the state's hemisphere (@ref unwrapPoseMeasurement)
 */
inline void poseInnovations(
  const ekf::PoseMeasurement & aligned,
  const ekf::State & state,
  const ekf::Covariance & state_covariance,
  std::array<double, 6> & innovations,
  std::array<double, 6> & state_variances)
{
  for (std::size_t i = 0; i < kPositionStateIndices.size(); ++i) {
    const int state_index = kPositionStateIndices[i];
    innovations[i] = aligned.data[i] - state.data[state_index];
    state_variances[i] = state_covariance.data[state_index * (ekf::Covariance::cols + 1)];
  }

  const auto [qx, qy, qz, qw] = state.get_orientation_quaternion();
  const Eigen::Quaterniond q_state(qw, qx, qy, qz);
  const Eigen::Quaterniond q_meas(
    aligned.data[ekf::PoseMeasurement::QW], aligned.data[ekf::PoseMeasurement::QX],
    aligned.data[ekf::PoseMeasurement::QY], aligned.data[ekf::PoseMeasurement::QZ]);
  const Eigen::Vector3d rotation = 2.0 * (q_meas * q_state.conjugate()).vec();
  const Eigen::Matrix3d rotation_covariance = orientationCovariance(state, state_covariance);

  for (int i = 0; i < 3; ++i) {
    innovations[3 + i] = rotation[i];
    state_variances[3 + i] = rotation_covariance(i, i);
  }
}

/**
 * @brief The diagonal of a 6x6 pose covariance, which is all the EKF takes
 *
 * The orientation block's diagonal is read as the variance of the rotation about the map's
 * x, y and z axes, the residual the quaternion update uses.
 */
inline ekf::PoseMeasurementCovariance covarianceToEkfMeasurementCovariance(
  const std::array<double, 36> & covariance)
{
  ekf::PoseMeasurementCovariance measurement_cov;
  for (std::size_t index = 0; index < 6; ++index) {
    measurement_cov.data[index] = covariance[diagonalIndex(index)];
  }
  return measurement_cov;
}

/**
 * @brief Which components of a measurement carry no information: those with a
 *        non-positive variance
 */
inline std::array<bool, 6> unobservedComponents(const std::array<double, 36> & covariance)
{
  std::array<bool, 6> unobserved{};
  for (std::size_t index = 0; index < 6; ++index) {
    unobserved[index] = covariance[diagonalIndex(index)] <= 0.0;
  }
  return unobserved;
}

/**
 * @brief Which components carry a variance of exactly zero
 *
 * The only non-positive variance that is usually a mistake rather than a statement: a
 * source filling no covariance in leaves zeros, and reading them as unobserved drops the
 * measurement.
 */
inline std::array<bool, 6> zeroVarianceComponents(const std::array<double, 36> & covariance)
{
  std::array<bool, 6> zeroed{};
  for (std::size_t index = 0; index < 6; ++index) {
    zeroed[index] = covariance[diagonalIndex(index)] == 0.0;
  }
  return zeroed;
}

/**
 * @brief Replace every non-positive variance with a usable one
 *
 * Applied before anything else touches the measurement, so that the rest of the pipeline,
 * the rotation into the map frame included, only ever sees usable numbers.
 */
inline void resolveUnobservedVariances(
  std::array<double, 36> & covariance, double unobserved_variance)
{
  for (std::size_t index = 0; index < 6; ++index) {
    double & variance = covariance[diagonalIndex(index)];
    if (variance <= 0.0) {
      variance = unobserved_variance;
    }
  }
}

/**
 * @brief Give the unobserved components of a pose measurement nothing to say
 *
 * Each flagged position component is overwritten with the state's own value and given the
 * unobserved variance, so its innovation is zero and so is the correction it produces,
 * whatever the gain turns out to be. The orientation is one quaternion: it takes the state's
 * only when all three rotations are unobserved; a partly observed one keeps its value and
 * only the flagged rotations get the unobserved variance.
 *
 * The flags are read in the source's frame and applied after the rotation into the map one,
 * which is exact for the sets that survive a yaw-only rotation: all three, none, or the
 * horizontal pair.
 */
inline void neutraliseUnobservedComponents(
  ekf::PoseMeasurement & measurement,
  ekf::PoseMeasurementCovariance & covariance,
  const std::array<bool, 6> & unobserved,
  const ekf::State & state,
  double unobserved_variance)
{
  for (std::size_t index = 0; index < kPositionStateIndices.size(); ++index) {
    if (unobserved[index]) {
      measurement.data[index] = state.data[kPositionStateIndices[index]];
      covariance.data[index] = unobserved_variance;
    }
  }
  for (std::size_t index = 3; index < 6; ++index) {
    if (unobserved[index]) {
      covariance.data[index] = unobserved_variance;
    }
  }
  if (unobserved[3] && unobserved[4] && unobserved[5]) {
    for (int i = 0; i < 4; ++i) {
      measurement.data[ekf::PoseMeasurement::QW + i] = state.data[ekf::State::QW + i];
    }
  }
}

/**
 * @brief The velocity counterpart of @ref neutraliseUnobservedComponents
 */
inline void neutraliseUnobservedVelocityComponents(
  ekf::VelocityMeasurement & measurement,
  ekf::VelocityMeasurementCovariance & covariance,
  const std::array<bool, 6> & unobserved,
  const ekf::State & state,
  double unobserved_variance)
{
  for (std::size_t index = 0; index < kVelocityStateIndices.size(); ++index) {
    if (unobserved[index]) {
      measurement.data[index] = state.data[kVelocityStateIndices[index]];
      covariance.data[index] = unobserved_variance;
    }
  }
}

inline ekf::VelocityMeasurement twistToEkfVelocityMeasurement(const TwistSample & twist)
{
  return ekf::VelocityMeasurement(twist.linear);
}

inline ekf::VelocityMeasurementCovariance twistToEkfVelocityCovariance(const TwistSample & twist)
{
  return ekf::VelocityMeasurementCovariance(
    {
      twist.covariance[diagonalIndex(0)],
      twist.covariance[diagonalIndex(1)],
      twist.covariance[diagonalIndex(2)]});
}

/**
 * @brief Whether a measurement is close enough to the prediction to be believed
 *
 * The measurement models select states directly, so a component's innovation is the
 * difference between measurement and prediction, and its variance is the sum of the two
 * variances. A component further than `gate` standard deviations away is not a
 * measurement of this vehicle: an obstacle under the rangefinder, a motion capture frame
 * that swapped bodies, a sensor gone wrong. A component the measurement does not observe
 * passes on its own, since the unobserved variance dwarfs any innovation it could produce.
 *
 * @param gate Number of standard deviations allowed. Non-positive disables the check
 * @return true when every component is within the gate
 */
template<std::size_t N>
bool isWithinInnovationGate(
  const std::array<double, N> & innovations,
  const std::array<double, N> & state_variances,
  const std::array<double, N> & measurement_variances,
  double gate)
{
  if (gate <= 0.0) {
    return true;
  }

  for (std::size_t index = 0; index < N; ++index) {
    const double innovation_variance = state_variances[index] + measurement_variances[index];
    if (innovation_variance > 0.0 &&
      innovations[index] * innovations[index] > gate * gate * innovation_variance)
    {
      return false;
    }
  }
  return true;
}

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__MEASUREMENT_UTILS_HPP_
