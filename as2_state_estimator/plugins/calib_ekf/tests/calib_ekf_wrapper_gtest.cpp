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
 * Unit tests for calib_ekf_math::EKFWrapper — mathematical correctness of prediction,
 * update, angle normalization, and map-to-odom tracking.
 * No ROS2 required.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>

#include "ekf_calib/calib_ekf_c_code.h"
#include "ekf_calib/ekf_wrapper.hpp"
#include "ekf_calib/ekf_datatype.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static calib_ekf_math::Covariance makeNonZeroCovariance()
{
  std::array<double, calib_ekf_math::Covariance::size> vals = {};
  vals[calib_ekf_math::Covariance::X]     = 1.0;
  vals[calib_ekf_math::Covariance::Y]     = 1.0;
  vals[calib_ekf_math::Covariance::Z]     = 1.0;
  vals[calib_ekf_math::Covariance::VX]    = 1.0;
  vals[calib_ekf_math::Covariance::VY]    = 1.0;
  vals[calib_ekf_math::Covariance::VZ]    = 1.0;
  // No variance on QW: at the identity attitude the admissible perturbations lie
  // in the vector part, and the unit norm constraint forbids motion along q.
  vals[calib_ekf_math::Covariance::QX]    = 1.0;
  vals[calib_ekf_math::Covariance::QY]    = 1.0;
  vals[calib_ekf_math::Covariance::QZ]    = 1.0;
  return calib_ekf_math::Covariance(vals);
}

// IMU measurement for a hovering drone: gravity-compensated az = 9.81
static calib_ekf_math::Input stationaryImu()
{
  return calib_ekf_math::Input({0.0, 0.0, 9.81, 0.0, 0.0, 0.0});
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class EkfWrapperTest : public ::testing::Test
{
protected:
  calib_ekf_math::EKFWrapper ekf_;

  void SetUp() override
  {
    ekf_.set_gravity(calib_ekf_math::Gravity({0.0, 0.0, 9.81}));
    Eigen::Vector<double, 6> imu_noise = Eigen::Vector<double, 6>::Zero();
    ekf_.set_noise_parameters(imu_noise, 1e-3, 1e-4, 1e-4, 1e-5);
    // Start with zero state and zero covariance (tight prior)
    ekf_.reset(calib_ekf_math::State(), calib_ekf_math::Covariance());
  }

  // Run N prediction steps with the stationary IMU input
  void runStationary(int n_steps, double dt)
  {
    calib_ekf_math::Input imu = stationaryImu();
    for (int i = 0; i < n_steps; ++i) {
      ekf_.predict(imu, dt);
    }
  }

  // Switch to non-zero initial covariance so Kalman gain is nonzero
  void setNonZeroCov()
  {
    ekf_.reset(calib_ekf_math::State(), makeNonZeroCovariance());
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
  ekf_.reset(calib_ekf_math::State(), calib_ekf_math::Covariance());

  calib_ekf_math::State s = ekf_.get_state();
  const calib_ekf_math::State expected;  // identity attitude, everything else zero
  for (std::size_t i = 0; i < calib_ekf_math::State::size; ++i) {
    EXPECT_DOUBLE_EQ(s.data[i], expected.data[i]) << "state[" << i << "] wrong after reset";
  }
  calib_ekf_math::Covariance c = ekf_.get_state_covariance();
  for (std::size_t i = 0; i < calib_ekf_math::Covariance::size; ++i) {
    EXPECT_DOUBLE_EQ(c.data[i], 0.0) << "covariance[" << i << "] != 0 after reset";
  }
}

// Port of Python test_predict_1: stationary hover → position should not move.
// Uses zero initial covariance so process noise does not open the filter.
TEST_F(EkfWrapperTest, StationaryDrone_PositionUnchanged)
{
  // 200 steps × 5 ms = 1 second
  runStationary(200, 0.005);

  calib_ekf_math::State s = ekf_.get_state();
  EXPECT_NEAR(s.data[calib_ekf_math::State::X],  0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::Y],  0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::Z],  0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::VX], 0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::VY], 0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::VZ], 0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::QW], 1.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::QX], 0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::QY], 0.0, 1e-6);
  EXPECT_NEAR(s.data[calib_ekf_math::State::QZ], 0.0, 1e-6);
}

// With P=0 the Kalman gain is 0: measurements must NOT change state.
TEST_F(EkfWrapperTest, ZeroCovariance_UpdateHasNoEffect)
{
  // P = 0 (default after reset), state at origin
  calib_ekf_math::PoseMeasurement meas({5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 0.0});
  calib_ekf_math::PoseMeasurementCovariance R;
  R.data.fill(1e-9);  // very confident measurement

  ekf_.update_pose(meas, R);

  calib_ekf_math::State s = ekf_.get_state();
  EXPECT_NEAR(s.data[calib_ekf_math::State::X], 0.0, 1e-10);
  EXPECT_NEAR(s.data[calib_ekf_math::State::Y], 0.0, 1e-10);
  EXPECT_NEAR(s.data[calib_ekf_math::State::Z], 0.0, 1e-10);
}

// With non-zero P, a confident measurement pulls the state toward it.
TEST_F(EkfWrapperTest, NonZeroCovariance_PoseUpdatePullsState)
{
  setNonZeroCov();
  runStationary(50, 0.005);

  calib_ekf_math::State before = ekf_.get_state();
  double cov_x_before = ekf_.get_state_covariance().data[calib_ekf_math::Covariance::X];

  calib_ekf_math::PoseMeasurement meas({1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  calib_ekf_math::PoseMeasurementCovariance R;
  R.data.fill(1e-4);

  ekf_.update_pose(meas, R);

  calib_ekf_math::State after = ekf_.get_state();
  double cov_x_after = ekf_.get_state_covariance().data[calib_ekf_math::Covariance::X];

  // State x moved toward measurement (which was at 1.0, prior was at ~0.0)
  EXPECT_GT(after.data[calib_ekf_math::State::X], before.data[calib_ekf_math::State::X]);
  // Covariance decreased (more certain after fusing a measurement)
  EXPECT_LT(cov_x_after, cov_x_before);
}

// update_pose must update the map-to-odom transform when state changes.
TEST_F(EkfWrapperTest, UpdatePose_ChangesMapToOdom)
{
  setNonZeroCov();
  runStationary(50, 0.005);

  Eigen::Matrix4d map_to_odom_before = ekf_.get_map_to_odom();

  calib_ekf_math::PoseMeasurement meas({2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  calib_ekf_math::PoseMeasurementCovariance R;
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

  calib_ekf_math::PoseMeasurement meas({2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  calib_ekf_math::PoseMeasurementCovariance R;
  R.data.fill(1e-4);
  ekf_.update_pose_odom(meas, R);

  Eigen::Matrix4d map_to_odom_after = ekf_.get_map_to_odom();

  EXPECT_TRUE(map_to_odom_after.isApprox(map_to_odom_before, 1e-12));
}

// The quaternion replaces Euler angle wrapping: there is no +/-pi branch to
// normalize, but the norm must stay at 1 no matter how long the filter runs.
TEST_F(EkfWrapperTest, QuaternionStaysNormalized_DuringPrediction)
{
  setNonZeroCov();

  // Rotate continuously about a tilted axis so the attitude keeps moving.
  calib_ekf_math::Input imu({0.1, -0.2, 9.81, 0.3, -0.4, 0.5});
  for (int i = 0; i < 1000; ++i) {
    ekf_.predict(imu, 0.005);
  }

  const calib_ekf_math::State s = ekf_.get_state();
  const double norm = std::sqrt(
    s.data[calib_ekf_math::State::QW] * s.data[calib_ekf_math::State::QW] +
    s.data[calib_ekf_math::State::QX] * s.data[calib_ekf_math::State::QX] +
    s.data[calib_ekf_math::State::QY] * s.data[calib_ekf_math::State::QY] +
    s.data[calib_ekf_math::State::QZ] * s.data[calib_ekf_math::State::QZ]);
  EXPECT_NEAR(norm, 1.0, 1e-12);
}

// A denormalized quaternion must be renormalized without corrupting the state.
TEST_F(EkfWrapperTest, QuaternionNormalization_RecoversUnitNorm)
{
  calib_ekf_math::State s;
  s.data[calib_ekf_math::State::QW] = 2.0;   // |q| = 2, same rotation as identity
  ekf_.reset(s, makeNonZeroCovariance());

  ekf_.predict(stationaryImu(), 0.005);

  const calib_ekf_math::State out = ekf_.get_state();
  EXPECT_NEAR(out.data[calib_ekf_math::State::QW], 1.0, 1e-9);
  EXPECT_NEAR(out.data[calib_ekf_math::State::QX], 0.0, 1e-9);
  EXPECT_NEAR(out.data[calib_ekf_math::State::QY], 0.0, 1e-9);
  EXPECT_NEAR(out.data[calib_ekf_math::State::QZ], 0.0, 1e-9);
}

// Normalization must leave P symmetric positive semi-definite, and must remove
// the variance along q itself (the direction the unit norm constraint forbids).
TEST_F(EkfWrapperTest, QuaternionNormalization_KeepsCovarianceConsistent)
{
  calib_ekf_math::State s;
  ekf_.reset(s, makeNonZeroCovariance());
  ekf_.predict(stationaryImu(), 0.005);

  const calib_ekf_math::Covariance c = ekf_.get_state_covariance();
  Eigen::Map<const Eigen::Matrix<double, calib_ekf_math::Covariance::rows,
    calib_ekf_math::Covariance::cols, Eigen::RowMajor>> P(c.data.data());

  EXPECT_TRUE(P.isApprox(P.transpose(), 1e-12)) << "P is not symmetric";

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9) << "P is not positive semi-definite";

  // Variance along the quaternion direction must be ~0.
  const calib_ekf_math::State out = ekf_.get_state();
  Eigen::Vector4d q;
  q << out.data[calib_ekf_math::State::QW], out.data[calib_ekf_math::State::QX],
    out.data[calib_ekf_math::State::QY], out.data[calib_ekf_math::State::QZ];
  const Eigen::Matrix4d Pqq =
    P.block<4, 4>(calib_ekf_math::State::QW, calib_ekf_math::State::QW);
  EXPECT_NEAR((q.transpose() * Pqq * q)(0, 0), 0.0, 1e-9);
}

// Process noise should inflate the covariance during prediction.
TEST_F(EkfWrapperTest, CovarianceGrows_DuringPrediction)
{
  setNonZeroCov();
  double cov_x_before = ekf_.get_state_covariance().data[calib_ekf_math::Covariance::X];

  runStationary(100, 0.005);

  double cov_x_after = ekf_.get_state_covariance().data[calib_ekf_math::Covariance::X];
  EXPECT_GT(cov_x_after, cov_x_before);
}

// A wrapper with correct gravity compensation keeps z-velocity near zero.
// One without gravity compensation accumulates velocity from the net acceleration.
TEST_F(EkfWrapperTest, GravityCompensation_NoGravity_Diverges)
{
  // Wrapper with gravity compensation (default fixture)
  calib_ekf_math::EKFWrapper ekf_grav;
  ekf_grav.set_gravity(calib_ekf_math::Gravity({0.0, 0.0, 9.81}));
  Eigen::Vector<double, 6> noise = Eigen::Vector<double, 6>::Zero();
  ekf_grav.set_noise_parameters(noise, 0.0, 0.0, 0.0, 0.0);
  ekf_grav.reset(calib_ekf_math::State(), calib_ekf_math::Covariance());

  // Wrapper without gravity compensation
  calib_ekf_math::EKFWrapper ekf_nograv;
  ekf_nograv.set_gravity(calib_ekf_math::Gravity({0.0, 0.0, 0.0}));
  ekf_nograv.set_noise_parameters(noise, 0.0, 0.0, 0.0, 0.0);
  ekf_nograv.reset(calib_ekf_math::State(), calib_ekf_math::Covariance());

  calib_ekf_math::Input imu = stationaryImu();  // az = 9.81
  for (int i = 0; i < 50; ++i) {
    ekf_grav.predict(imu, 0.005);
    ekf_nograv.predict(imu, 0.005);
  }

  // Gravity-compensated: z-velocity stays near zero
  EXPECT_NEAR(ekf_grav.get_state().data[calib_ekf_math::State::VZ], 0.0, 1e-6);
  // No gravity: z-velocity grows (net acceleration = 9.81 m/s²)
  EXPECT_GT(std::abs(ekf_nograv.get_state().data[calib_ekf_math::State::VZ]), 0.1);
}

// Predicting with dt=0 must not crash and must not change state.
TEST_F(EkfWrapperTest, PredictDtZero_StateUnchanged)
{
  setNonZeroCov();
  runStationary(10, 0.005);

  calib_ekf_math::State before = ekf_.get_state();
  EXPECT_NO_THROW(ekf_.predict(stationaryImu(), 0.0));
  calib_ekf_math::State after = ekf_.get_state();

  for (std::size_t i = 0; i < calib_ekf_math::State::size; ++i) {
    EXPECT_NEAR(after.data[i], before.data[i], 1e-10)
      << "state[" << i << "] changed after dt=0 predict";
    EXPECT_FALSE(std::isnan(after.data[i])) << "state[" << i << "] is NaN";
  }
}

// ---------------------------------------------------------------------------
// Model / generated-code checks
// ---------------------------------------------------------------------------

// The state layout and the covariance index constants must agree. A mismatch
// here silently mis-addresses every covariance entry.
TEST(EkfModelTest, CovarianceIndicesMatchStride)
{
  // The unary + forces an rvalue: these are in-class static const members with
  // no out-of-class definition, so binding them to EXPECT_EQ's const& would be
  // an ODR-use and fail to link.
  EXPECT_EQ(+calib_ekf_math::State::size, 26u);
  EXPECT_EQ(+calib_ekf_math::Covariance::size, 676u);
  EXPECT_EQ(+calib_ekf_math::Covariance::rows, 26);

  const int stride = calib_ekf_math::Covariance::rows + 1;
  EXPECT_EQ(+calib_ekf_math::Covariance::X, calib_ekf_math::State::X * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::VZ, calib_ekf_math::State::VZ * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::QW, calib_ekf_math::State::QW * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::QZ, calib_ekf_math::State::QZ * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::WBZ, calib_ekf_math::State::WBZ * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::EX, calib_ekf_math::State::EX * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::EPSI, calib_ekf_math::State::EPSI * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::FX, calib_ekf_math::State::FX * stride);
  EXPECT_EQ(+calib_ekf_math::Covariance::CY, calib_ekf_math::State::CY * stride);
}

// A default-constructed State must be a usable attitude. An all-zero quaternion
// has no norm and turns the whole filter into NaN on the first predict.
TEST(EkfModelTest, DefaultStateHasIdentityAttitude)
{
  const calib_ekf_math::State s;
  EXPECT_DOUBLE_EQ(s.data[calib_ekf_math::State::QW], 1.0);
  EXPECT_DOUBLE_EQ(s.data[calib_ekf_math::State::QX], 0.0);
  EXPECT_DOUBLE_EQ(s.data[calib_ekf_math::State::QY], 0.0);
  EXPECT_DOUBLE_EQ(s.data[calib_ekf_math::State::QZ], 0.0);
}

// Compare the generated F against a central finite difference of the generated
// f. This is the single strongest check on the port: it fails if the CasADi
// output is mapped with the wrong storage order, if any state index is wrong, or
// if the model and its Jacobian disagree.
TEST(EkfModelTest, PredictJacobianMatchesFiniteDifference)
{
  constexpr std::size_t N = calib_ekf_math::State::size;
  const double g[3] = {0.0, 0.0, 9.81};
  const double dt = 0.013;

  auto eval_f = [&](const std::array<double, N> & x,
      const std::array<double, 6> & u, std::array<double, N> & f_out) {
      double F[N * N];
      double L[N * 6];
      const double * arg[calib_predict_function_SZ_ARG] = {x.data(), u.data(), &dt, g};
      double * res[calib_predict_function_SZ_RES] = {f_out.data(), F, L};
      calib_predict_function(arg, res, nullptr, nullptr, 0);
    };

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-0.7, 0.7);

  double worst = 0.0;
  for (int trial = 0; trial < 20; ++trial) {
    std::array<double, N> x{};
    std::array<double, 6> u{};
    for (std::size_t i = 0; i < N; ++i) {x[i] = dist(rng);}
    for (std::size_t i = 0; i < 6; ++i) {u[i] = dist(rng);}
    // Keep the attitude a valid unit quaternion.
    const double n = std::sqrt(
      x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
    for (std::size_t i = 6; i < 10; ++i) {x[i] /= n;}

    double F[N * N];
    double L[N * 6];
    std::array<double, N> f{};
    {
      const double * arg[calib_predict_function_SZ_ARG] = {x.data(), u.data(), &dt, g};
      double * res[calib_predict_function_SZ_RES] = {f.data(), F, L};
      calib_predict_function(arg, res, nullptr, nullptr, 0);
    }
    // CasADi writes dense matrices column-major.
    Eigen::Map<const Eigen::Matrix<double, N, N, Eigen::ColMajor>> F_gen(F);

    const double eps = 1e-7;
    for (std::size_t j = 0; j < N; ++j) {
      std::array<double, N> xp = x, xm = x;
      xp[j] += eps;
      xm[j] -= eps;
      std::array<double, N> fp{}, fm{};
      eval_f(xp, u, fp);
      eval_f(xm, u, fm);
      for (std::size_t i = 0; i < N; ++i) {
        const double fd = (fp[i] - fm[i]) / (2.0 * eps);
        worst = std::max(worst, std::abs(F_gen(i, j) - fd));
      }
    }
  }
  EXPECT_LT(worst, 1e-6) << "max |F_generated - F_finite_difference| = " << worst;
}

// A level, unbiased drone reading +9.81 in body z must not accelerate. This pins
// the ENU convention: gravity is subtracted as a world-frame vector.
TEST(EkfModelTest, HoverIsAnEquilibrium)
{
  constexpr std::size_t N = calib_ekf_math::State::size;
  const double g[3] = {0.0, 0.0, 9.81};
  const double dt = 0.01;

  calib_ekf_math::State x;   // identity attitude, zero velocity, zero bias
  const calib_ekf_math::Input u({0.0, 0.0, 9.81, 0.0, 0.0, 0.0});

  std::array<double, N> f{};
  double F[N * N];
  double L[N * 6];
  const double * arg[calib_predict_function_SZ_ARG] = {
    x.data.data(), u.data.data(), &dt, g};
  double * res[calib_predict_function_SZ_RES] = {f.data(), F, L};
  calib_predict_function(arg, res, nullptr, nullptr, 0);

  EXPECT_NEAR(f[calib_ekf_math::State::VX], 0.0, 1e-12);
  EXPECT_NEAR(f[calib_ekf_math::State::VY], 0.0, 1e-12);
  EXPECT_NEAR(f[calib_ekf_math::State::VZ], 0.0, 1e-12);
}
