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
* @file ekf_datatype.cpp
*
* An EKF Wrapper implementation
*
* @authors Rodrigo Da Silva Gómez
*/

#include "ekf_calib/ekf_datatype.hpp"

namespace calib_ekf_math
{


State::State()
{
  data.fill(0.0);
  data[QW] = 1.0;
}


State::State(const std::array<double, size> & values)
{
  set(values);
}


std::array<double, 3> State::get_position() const
{
  return {data[X], data[Y], data[Z]};
}


std::array<double, 3> State::get_velocity() const
{
  return {data[VX], data[VY], data[VZ]};
}


std::array<double, 3> State::get_orientation() const
{
  // ZYX intrinsic (yaw-pitch-roll) Euler angles from the state quaternion.
  const std::array<double, 4> q = get_orientation_quaternion();
  const double qx = q[0];
  const double qy = q[1];
  const double qz = q[2];
  const double qw = q[3];

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  // Clamped so that a slightly out-of-range argument cannot produce NaN.
  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = std::abs(sinp) >= 1.0 ?
    std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}


std::array<double, 4> State::get_orientation_quaternion() const
{
  // Note the ordering: qx, qy, qz, qw (tf2 convention), while the state stores
  // qw first.
  const double norm = std::sqrt(
    data[QW] * data[QW] + data[QX] * data[QX] +
    data[QY] * data[QY] + data[QZ] * data[QZ]);
  if (norm <= 0.0) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {data[QX] / norm, data[QY] / norm, data[QZ] / norm, data[QW] / norm};
}


std::array<double, 3> State::get_accelerometer_bias() const
{
  return {data[ABX], data[ABY], data[ABZ]};
}


std::array<double, 3> State::get_gyroscope_bias() const
{
  return {data[WBX], data[WBY], data[WBZ]};
}


void State::set(const std::array<double, size> & values)
{
  data = values;
}


std::string State::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


Covariance::Covariance()
{
  data.fill(0.0);
}


Covariance::Covariance(const std::array<double, size> & values)
{
  set(values);
}


void Covariance::set(const std::array<double, size> & values)
{
  data = values;
}


std::string Covariance::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % Covariance::cols == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


std::string Covariance::to_string_diagonal() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < Covariance::rows; ++i) {
    oss << data[i * (Covariance::rows + 1)];

    if (i + 1 != Covariance::rows) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


Gravity::Gravity()
{
  data.fill(0.0);
  data[2] = 9.81; // Default gravity value in m/s^2
}


Gravity::Gravity(const std::array<double, size> & values)
{
  set(values);
}


void Gravity::set(const std::array<double, size> & values)
{
  data = values;
}


Input::Input()
{
  data.fill(0.0);
}


Input::Input(const std::array<double, size> & values)
{
  set(values);
}


void Input::set(const std::array<double, size> & values)
{
  data = values;
}


std::string Input::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


PoseMeasurement::PoseMeasurement()
{
  data.fill(0.0);
}


PoseMeasurement::PoseMeasurement(const std::array<double, size> & values)
{
  set(values);
}


void PoseMeasurement::set(const std::array<double, size> & values)
{
  data = values;
}


std::string PoseMeasurement::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


PoseMeasurementCovariance::PoseMeasurementCovariance()
{
  data.fill(0.0);
}


PoseMeasurementCovariance::PoseMeasurementCovariance(const std::array<double, size> & values)
{
  set(values);
}


void PoseMeasurementCovariance::set(const std::array<double, size> & values)
{
  data = values;
}


std::string PoseMeasurementCovariance::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


VelocityMeasurement::VelocityMeasurement()
{
  data.fill(0.0);
}


VelocityMeasurement::VelocityMeasurement(const std::array<double, size> & values)
{
  set(values);
}


void VelocityMeasurement::set(const std::array<double, size> & values)
{
  data = values;
}


std::string VelocityMeasurement::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


VelocityMeasurementCovariance::VelocityMeasurementCovariance()
{
  data.fill(0.0);
}


VelocityMeasurementCovariance::VelocityMeasurementCovariance(
  const std::array<double,
  size> & values)
{
  set(values);
}


void VelocityMeasurementCovariance::set(const std::array<double, size> & values)
{
  data = values;
}


std::string VelocityMeasurementCovariance::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


Odometry::Odometry()
{
  data.fill(0.0);
}

Odometry::Odometry(const std::array<double, size> & values)
{
  set(values);
}

void Odometry::set(const std::array<double, size> & values)
{
  data = values;
}

std::string Odometry::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


} // namespace calib_ekf_math
