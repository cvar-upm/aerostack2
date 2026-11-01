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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
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
 * @file ekf_wrapper_gtest.cpp
 *
 * Unit tests for ekf::EKFWrapper — mathematical correctness of prediction,
 * update, angle normalization, and map-to-odom tracking.
 * No ROS2 required.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>

#include "ekf/ekf_wrapper.hpp"
#include "ekf/ekf_datatype.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Pose measurement from a position and roll, pitch, yaw (Z-Y-X)
static ekf::PoseMeasurement poseMeasurement(
  double x, double y, double z, double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
{
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  return ekf::PoseMeasurement({x, y, z, q.w(), q.x(), q.y(), q.z()});
}

// The state's orientation quaternion, w first
static Eigen::Quaterniond stateQuaternion(const ekf::State & s)
{
  return Eigen::Quaterniond(
    s.data[ekf::State::QW], s.data[ekf::State::QX], s.data[ekf::State::QY],
    s.data[ekf::State::QZ]);
}

static ekf::Covariance makeNonZeroCovariance()
{
  std::array<double, ekf::Covariance::size> vals = {};
  vals[ekf::Covariance::X] = 1.0;
  vals[ekf::Covariance::Y] = 1.0;
  vals[ekf::Covariance::Z] = 1.0;
  vals[ekf::Covariance::VX] = 1.0;
  vals[ekf::Covariance::VY] = 1.0;
  vals[ekf::Covariance::VZ] = 1.0;
  // 1 rad^2 about each axis of the identity orientation is 1/4 on qx, qy, qz
  vals[ekf::Covariance::QX] = 0.25;
  vals[ekf::Covariance::QY] = 0.25;
  vals[ekf::Covariance::QZ] = 0.25;
  return ekf::Covariance(vals);
}

// IMU measurement for a hovering drone: gravity-compensated az = 9.81
static ekf::Input stationaryImu()
{
  return ekf::Input({0.0, 0.0, 9.81, 0.0, 0.0, 0.0});
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class EkfWrapperTest : public ::testing::Test
{
protected:
  ekf::EKFWrapper ekf_;

  void SetUp() override
  {
    ekf_.set_gravity(ekf::Gravity({0.0, 0.0, 9.81}));
    Eigen::Vector<double, 6> imu_noise = Eigen::Vector<double, 6>::Zero();
    ekf_.set_noise_parameters(imu_noise, 1e-3, 1e-4, 1e-4, 1e-5);
    // Start with zero state and zero covariance (tight prior)
    ekf_.reset(ekf::State(), ekf::Covariance());
  }

  // Run N prediction steps with the stationary IMU input
  void runStationary(int n_steps, double dt)
  {
    ekf::Input imu = stationaryImu();
    for (int i = 0; i < n_steps; ++i) {
      ekf_.predict(imu, dt);
    }
  }

  // Switch to non-zero initial covariance so Kalman gain is nonzero
  void setNonZeroCov()
  {
    ekf_.reset(ekf::State(), makeNonZeroCovariance());
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_F(EkfWrapperTest, DefaultInit_MapToOdomIsIdentity)
{
  EXPECT_TRUE(ekf_.get_map_to_odom().isApprox(Eigen::Matrix4d::Identity(), 1e-12));
}

TEST_F(EkfWrapperTest, ResetRestoresState)
{
  runStationary(50, 0.005);

  // State should have changed (covariance grew, even if position is small)
  ekf_.reset(ekf::State(), ekf::Covariance());

  // A default state is at rest at the origin, with the identity orientation
  ekf::State s = ekf_.get_state();
  for (std::size_t i = 0; i < ekf::State::size; ++i) {
    const double expected = (i == ekf::State::QW) ? 1.0 : 0.0;
    EXPECT_DOUBLE_EQ(s.data[i], expected) << "state[" << i << "] not reset";
  }
  ekf::Covariance c = ekf_.get_state_covariance();
  for (std::size_t i = 0; i < ekf::Covariance::size; ++i) {
    EXPECT_DOUBLE_EQ(c.data[i], 0.0) << "covariance[" << i << "] != 0 after reset";
  }
}

// Port of Python test_predict_1: stationary hover → position should not move.
// Uses zero initial covariance so process noise does not open the filter.
TEST_F(EkfWrapperTest, StationaryDrone_PositionUnchanged)
{
  // 200 steps × 5 ms = 1 second
  runStationary(200, 0.005);

  ekf::State s = ekf_.get_state();
  EXPECT_NEAR(s.data[ekf::State::X], 0.0, 1e-6);
  EXPECT_NEAR(s.data[ekf::State::Y], 0.0, 1e-6);
  EXPECT_NEAR(s.data[ekf::State::Z], 0.0, 1e-6);
  EXPECT_NEAR(s.data[ekf::State::VX], 0.0, 1e-6);
  EXPECT_NEAR(s.data[ekf::State::VY], 0.0, 1e-6);
  EXPECT_NEAR(s.data[ekf::State::VZ], 0.0, 1e-6);
  EXPECT_NEAR(s.get_orientation()[2], 0.0, 1e-6);
}

// With P=0 the Kalman gain is 0: measurements must NOT change state.
TEST_F(EkfWrapperTest, ZeroCovariance_UpdateHasNoEffect)
{
  // P = 0 (default after reset), state at origin
  ekf::PoseMeasurement meas = poseMeasurement(5.0, 5.0, 5.0, 0.1, 0.1, 0.1);
  ekf::PoseMeasurementCovariance R;
  R.data.fill(1e-9);  // very confident measurement

  ekf_.update_pose(meas, R);

  ekf::State s = ekf_.get_state();
  EXPECT_NEAR(s.data[ekf::State::X], 0.0, 1e-10);
  EXPECT_NEAR(s.data[ekf::State::Y], 0.0, 1e-10);
  EXPECT_NEAR(s.data[ekf::State::Z], 0.0, 1e-10);
}

// With non-zero P, a confident measurement pulls the state toward it.
TEST_F(EkfWrapperTest, NonZeroCovariance_PoseUpdatePullsState)
{
  setNonZeroCov();
  runStationary(50, 0.005);

  ekf::State before = ekf_.get_state();
  double cov_x_before = ekf_.get_state_covariance().data[ekf::Covariance::X];

  ekf::PoseMeasurement meas = poseMeasurement(1.0, 0.0, 0.0);
  ekf::PoseMeasurementCovariance R;
  R.data.fill(1e-4);

  ekf_.update_pose(meas, R);

  ekf::State after = ekf_.get_state();
  double cov_x_after = ekf_.get_state_covariance().data[ekf::Covariance::X];

  // State x moved toward measurement (which was at 1.0, prior was at ~0.0)
  EXPECT_GT(after.data[ekf::State::X], before.data[ekf::State::X]);
  // Covariance decreased (more certain after fusing a measurement)
  EXPECT_LT(cov_x_after, cov_x_before);
}

// update_pose must update the map-to-odom transform when state changes.
TEST_F(EkfWrapperTest, UpdatePose_ChangesMapToOdom)
{
  setNonZeroCov();
  runStationary(50, 0.005);

  Eigen::Matrix4d map_to_odom_before = ekf_.get_map_to_odom();

  ekf::PoseMeasurement meas = poseMeasurement(2.0, 0.0, 0.0);
  ekf::PoseMeasurementCovariance R;
  R.data.fill(1e-4);
  ekf_.update_pose(meas, R);

  Eigen::Matrix4d map_to_odom_after = ekf_.get_map_to_odom();

  // map_to_odom should have changed after a pose update that moved the state
  EXPECT_FALSE(map_to_odom_after.isApprox(map_to_odom_before, 1e-10));
}

// update_pose_odom must NOT update the map-to-odom transform.
TEST_F(EkfWrapperTest, UpdatePoseOdom_DoesNotChangeMapToOdom)
{
  setNonZeroCov();
  runStationary(50, 0.005);

  Eigen::Matrix4d map_to_odom_before = ekf_.get_map_to_odom();

  ekf::PoseMeasurement meas = poseMeasurement(2.0, 0.0, 0.0);
  ekf::PoseMeasurementCovariance R;
  R.data.fill(1e-4);
  ekf_.update_pose_odom(meas, R);

  Eigen::Matrix4d map_to_odom_after = ekf_.get_map_to_odom();

  EXPECT_TRUE(map_to_odom_after.isApprox(map_to_odom_before, 1e-12));
}

// A yaw rotation through ±π is continuous: the quaternion has no branch to wrap.
TEST_F(EkfWrapperTest, QuaternionNormalization_YawPastPi)
{
  // Start with yaw just below π
  ekf::State s;
  const Eigen::Quaterniond q0(Eigen::AngleAxisd(3.10, Eigen::Vector3d::UnitZ()));
  s.data[ekf::State::QW] = q0.w();
  s.data[ekf::State::QZ] = q0.z();
  ekf_.reset(s, ekf::Covariance());

  // Predict with positive yaw rate: wz = 0.5 rad/s for 0.1 s → Δyaw = 0.05 rad
  ekf::Input imu({0.0, 0.0, 9.81, 0.0, 0.0, 0.5});
  ekf_.predict(imu, 0.1);

  const Eigen::Quaterniond q = stateQuaternion(ekf_.get_state());
  EXPECT_NEAR(q.norm(), 1.0, 1e-12);
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(3.15, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(q.angularDistance(expected), 0.0, 1e-6);
}

// A quaternion off unit norm is renormalized by the next step, keeping its rotation.
TEST_F(EkfWrapperTest, QuaternionNormalization_NonUnitState)
{
  ekf::State s;
  const Eigen::Quaterniond q0(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  s.data[ekf::State::QW] = 2.0 * q0.w();
  s.data[ekf::State::QZ] = 2.0 * q0.z();
  ekf_.reset(s, ekf::Covariance());

  ekf_.predict(stationaryImu(), 0.005);

  const Eigen::Quaterniond q = stateQuaternion(ekf_.get_state());
  EXPECT_NEAR(q.norm(), 1.0, 1e-12);
  EXPECT_NEAR(q.angularDistance(q0), 0.0, 1e-9);
}

// Process noise should inflate the covariance during prediction.
TEST_F(EkfWrapperTest, CovarianceGrows_DuringPrediction)
{
  setNonZeroCov();
  double cov_x_before = ekf_.get_state_covariance().data[ekf::Covariance::X];

  runStationary(100, 0.005);

  double cov_x_after = ekf_.get_state_covariance().data[ekf::Covariance::X];
  EXPECT_GT(cov_x_after, cov_x_before);
}

// A wrapper with correct gravity compensation keeps z-velocity near zero.
// One without gravity compensation accumulates velocity from the net acceleration.
TEST_F(EkfWrapperTest, GravityCompensation_NoGravity_Diverges)
{
  // Wrapper with gravity compensation (default fixture)
  ekf::EKFWrapper ekf_grav;
  ekf_grav.set_gravity(ekf::Gravity({0.0, 0.0, 9.81}));
  Eigen::Vector<double, 6> noise = Eigen::Vector<double, 6>::Zero();
  ekf_grav.set_noise_parameters(noise, 0.0, 0.0, 0.0, 0.0);
  ekf_grav.reset(ekf::State(), ekf::Covariance());

  // Wrapper without gravity compensation
  ekf::EKFWrapper ekf_nograv;
  ekf_nograv.set_gravity(ekf::Gravity({0.0, 0.0, 0.0}));
  ekf_nograv.set_noise_parameters(noise, 0.0, 0.0, 0.0, 0.0);
  ekf_nograv.reset(ekf::State(), ekf::Covariance());

  ekf::Input imu = stationaryImu();  // az = 9.81
  for (int i = 0; i < 50; ++i) {
    ekf_grav.predict(imu, 0.005);
    ekf_nograv.predict(imu, 0.005);
  }

  // Gravity-compensated: z-velocity stays near zero
  EXPECT_NEAR(ekf_grav.get_state().data[ekf::State::VZ], 0.0, 1e-6);
  // No gravity: z-velocity grows (net acceleration = 9.81 m/s²)
  EXPECT_GT(std::abs(ekf_nograv.get_state().data[ekf::State::VZ]), 0.1);
}

// Predicting with dt=0 must not crash and must not change state.
TEST_F(EkfWrapperTest, PredictDtZero_StateUnchanged)
{
  setNonZeroCov();
  runStationary(10, 0.005);

  ekf::State before = ekf_.get_state();
  EXPECT_NO_THROW(ekf_.predict(stationaryImu(), 0.0));
  ekf::State after = ekf_.get_state();

  for (std::size_t i = 0; i < ekf::State::size; ++i) {
    EXPECT_NEAR(after.data[i], before.data[i], 1e-10)
      << "state[" << i << "] changed after dt=0 predict";
    EXPECT_FALSE(std::isnan(after.data[i])) << "state[" << i << "] is NaN";
  }
}
