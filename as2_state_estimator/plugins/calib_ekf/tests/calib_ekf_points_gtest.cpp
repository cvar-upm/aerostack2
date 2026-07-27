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
 * @file calib_ekf_points_gtest.cpp
 *
 * Tests for the landmark reprojection model and the points update: the
 * projection itself, its Jacobian, the gating, and the observability
 * properties of the camera calibration states.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "ekf_calib/calib_ekf_c_code.h"
#include "ekf_calib/ekf_wrapper.hpp"
#include "ekf_calib/ekf_datatype.hpp"

namespace
{

constexpr std::size_t N = calib_ekf_math::State::size;

// A state with a plausible camera: 10 cm forward lever arm, 500 px focal.
calib_ekf_math::State makeCameraState()
{
  calib_ekf_math::State s;
  s.data[calib_ekf_math::State::EX] = 0.10;
  s.data[calib_ekf_math::State::EY] = 0.02;
  s.data[calib_ekf_math::State::EZ] = 0.05;
  s.data[calib_ekf_math::State::FX] = 500.0;
  s.data[calib_ekf_math::State::FY] = 520.0;
  s.data[calib_ekf_math::State::CX] = 320.0;
  s.data[calib_ekf_math::State::CY] = 240.0;
  return s;
}

// Independent reference implementation of the reprojection, written directly
// from the geometry rather than from the generated code.
Eigen::Vector2d reprojectReference(
  const calib_ekf_math::State & s,
  const Eigen::Vector3d & p_world,
  const Eigen::Matrix3d & R_nom,
  double * depth_out = nullptr)
{
  const Eigen::Vector3d t(
    s.data[calib_ekf_math::State::X], s.data[calib_ekf_math::State::Y],
    s.data[calib_ekf_math::State::Z]);
  Eigen::Quaterniond q(
    s.data[calib_ekf_math::State::QW], s.data[calib_ekf_math::State::QX],
    s.data[calib_ekf_math::State::QY], s.data[calib_ekf_math::State::QZ]);
  q.normalize();
  const Eigen::Vector3d e(
    s.data[calib_ekf_math::State::EX], s.data[calib_ekf_math::State::EY],
    s.data[calib_ekf_math::State::EZ]);

  const Eigen::Vector3d p_body = q.toRotationMatrix().transpose() * (p_world - t);

  const double phi = s.data[calib_ekf_math::State::EPHI];
  const double th = s.data[calib_ekf_math::State::ETHETA];
  const double psi = s.data[calib_ekf_math::State::EPSI];
  Eigen::Matrix3d Rx, Ry, Rz;
  Rx << 1, 0, 0, 0, std::cos(phi), -std::sin(phi), 0, std::sin(phi), std::cos(phi);
  Ry << std::cos(th), 0, std::sin(th), 0, 1, 0, -std::sin(th), 0, std::cos(th);
  Rz << std::cos(psi), -std::sin(psi), 0, std::sin(psi), std::cos(psi), 0, 0, 0, 1;
  const Eigen::Matrix3d Re = R_nom * (Rz * Ry * Rx);

  const Eigen::Vector3d p_cam = Re.transpose() * (p_body - e);
  if (depth_out) {*depth_out = p_cam[0];}

  return Eigen::Vector2d(
    s.data[calib_ekf_math::State::FX] * p_cam[1] / p_cam[0] +
    s.data[calib_ekf_math::State::CX],
    s.data[calib_ekf_math::State::FY] * p_cam[2] / p_cam[0] +
    s.data[calib_ekf_math::State::CY]);
}

// Call the generated point function.
void evalPoint(
  const calib_ekf_math::State & s, const Eigen::Vector3d & p_world,
  const Eigen::Matrix3d & R_nom_in,
  double h[2], double H[2 * N], double * depth)
{
  const Eigen::Matrix<double, 3, 3, Eigen::ColMajor> R_nom = R_nom_in;
  const double * arg[calib_point_function_SZ_ARG] = {
    s.data.data(), p_world.data(), R_nom.data()};
  double * res[calib_point_function_SZ_RES] = {h, H, depth};
  calib_point_function(arg, res, nullptr, nullptr, 0);
}

}  // namespace

// The generated projection must agree with an independent implementation. This
// pins the forward-right-down to OpenCV axis permutation and the meaning of
// R_nom, neither of which any other test would catch.
TEST(PointsModelTest, ReprojectionMatchesIndependentImplementation)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> d(-0.4, 0.4);
  const Eigen::Matrix3d R_nom = Eigen::Matrix3d::Identity();

  double worst = 0.0;
  for (int trial = 0; trial < 50; ++trial) {
    calib_ekf_math::State s = makeCameraState();
    for (int i = 0; i < 3; ++i) {s.data[i] = d(rng);}
    Eigen::Quaterniond q(1.0 + d(rng), d(rng), d(rng), d(rng));
    q.normalize();
    s.data[calib_ekf_math::State::QW] = q.w();
    s.data[calib_ekf_math::State::QX] = q.x();
    s.data[calib_ekf_math::State::QY] = q.y();
    s.data[calib_ekf_math::State::QZ] = q.z();
    s.data[calib_ekf_math::State::EPHI] = d(rng) * 0.2;
    s.data[calib_ekf_math::State::ETHETA] = d(rng) * 0.2;
    s.data[calib_ekf_math::State::EPSI] = d(rng) * 0.2;

    // Landmark comfortably in front of the camera (optical axis = camera x).
    const Eigen::Vector3d p_world = q * Eigen::Vector3d(6.0, d(rng), d(rng)) +
      Eigen::Vector3d(s.data[0], s.data[1], s.data[2]);

    double h[2], H[2 * N], depth = 0.0;
    evalPoint(s, p_world, R_nom, h, H, &depth);

    double depth_ref = 0.0;
    const Eigen::Vector2d ref = reprojectReference(s, p_world, R_nom, &depth_ref);

    worst = std::max(worst, std::abs(h[0] - ref[0]));
    worst = std::max(worst, std::abs(h[1] - ref[1]));
    worst = std::max(worst, std::abs(depth - depth_ref));
  }
  EXPECT_LT(worst, 1e-8) << "max |h_generated - h_reference| = " << worst;
}

// The intrinsics enter the projection linearly, so this Jacobian block is exact
// and must be [[xn, 0, 1, 0], [0, yn, 0, 1]]. Cheap, and a very strong check on
// the whole index map: any misalignment of the intrinsic indices breaks it.
TEST(PointsModelTest, IntrinsicsJacobianIsExact)
{
  calib_ekf_math::State s = makeCameraState();
  s.data[calib_ekf_math::State::EPHI] = 0.01;
  s.data[calib_ekf_math::State::ETHETA] = -0.02;
  s.data[calib_ekf_math::State::EPSI] = 0.03;
  const Eigen::Vector3d p_world(4.0, 0.7, -0.3);

  double h[2], H[2 * N], depth = 0.0;
  evalPoint(s, p_world, Eigen::Matrix3d::Identity(), h, H, &depth);

  Eigen::Map<const Eigen::Matrix<double, 2, N, Eigen::ColMajor>> H_gen(H);

  const double xn = (h[0] - s.data[calib_ekf_math::State::CX]) /
    s.data[calib_ekf_math::State::FX];
  const double yn = (h[1] - s.data[calib_ekf_math::State::CY]) /
    s.data[calib_ekf_math::State::FY];

  Eigen::Matrix<double, 2, 4> expected;
  expected << xn, 0.0, 1.0, 0.0,
    0.0, yn, 0.0, 1.0;

  // Assigned to a local first: the comma inside block<2, 4> would otherwise be
  // parsed as a macro argument separator.
  const Eigen::Matrix<double, 2, 4> intrinsics_block =
    H_gen.block(0, calib_ekf_math::State::FX, 2, 4);

  EXPECT_TRUE(intrinsics_block.isApprox(expected, 1e-12))
    << "got:\n" << intrinsics_block << "\nwant:\n" << expected;
}

// Central finite difference of the generated h against the generated H. Catches
// a wrong storage order, a wrong state index, or a chain rule error.
TEST(PointsModelTest, PointJacobianMatchesFiniteDifference)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> d(-0.3, 0.3);
  const Eigen::Matrix3d R_nom = Eigen::Matrix3d::Identity();

  double worst_rel = 0.0;
  for (int trial = 0; trial < 10; ++trial) {
    calib_ekf_math::State s = makeCameraState();
    for (int i = 0; i < 3; ++i) {s.data[i] = d(rng);}
    Eigen::Quaterniond q(1.0 + d(rng), d(rng), d(rng), d(rng));
    q.normalize();
    s.data[calib_ekf_math::State::QW] = q.w();
    s.data[calib_ekf_math::State::QX] = q.x();
    s.data[calib_ekf_math::State::QY] = q.y();
    s.data[calib_ekf_math::State::QZ] = q.z();

    const Eigen::Vector3d p_world = q * Eigen::Vector3d(6.0, d(rng), d(rng)) +
      Eigen::Vector3d(s.data[0], s.data[1], s.data[2]);

    double h[2], H[2 * N], depth = 0.0;
    evalPoint(s, p_world, R_nom, h, H, &depth);
    Eigen::Map<const Eigen::Matrix<double, 2, N, Eigen::ColMajor>> H_gen(H);

    const double eps = 1e-7;
    for (std::size_t j = 0; j < N; ++j) {
      calib_ekf_math::State sp = s, sm = s;
      sp.data[j] += eps;
      sm.data[j] -= eps;
      double hp[2], hm[2], Hd[2 * N], dd = 0.0;
      evalPoint(sp, p_world, R_nom, hp, Hd, &dd);
      evalPoint(sm, p_world, R_nom, hm, Hd, &dd);
      for (int i = 0; i < 2; ++i) {
        const double fd = (hp[i] - hm[i]) / (2.0 * eps);
        // Relative: with fx ~ 500 the derivatives are O(100), so an absolute
        // tolerance would be meaningless.
        const double scale = std::max(1.0, std::abs(fd));
        worst_rel = std::max(worst_rel, std::abs(H_gen(i, j) - fd) / scale);
      }
    }
  }
  EXPECT_LT(worst_rel, 1e-5) << "max relative Jacobian error = " << worst_rel;
}

// The projection divides by depth without a guard, so a landmark behind the
// camera must be rejected rather than fused.
TEST(PointsUpdateTest, RejectsLandmarkBehindCamera)
{
  calib_ekf_math::EKFWrapper ekf;
  calib_ekf_math::State s = makeCameraState();
  std::array<double, calib_ekf_math::Covariance::size> c{};
  c[calib_ekf_math::Covariance::EX] = 1e-2;
  ekf.reset(s, calib_ekf_math::Covariance(c));

  std::vector<calib_ekf_math::LandmarkObservation> obs(1);
  obs[0].p_world = Eigen::Vector3d(-5.0, 0.0, 0.0);   // behind (optical axis +x)
  obs[0].pixel = Eigen::Vector2d(320.0, 240.0);
  obs[0].sigma_px = 1.0;

  const calib_ekf_math::State before = ekf.get_state();
  const auto result = ekf.update_points(obs, calib_ekf_math::PointsUpdateConfig());

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.n_rejected, 1u);
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_DOUBLE_EQ(ekf.get_state().data[i], before.data[i]);
    EXPECT_FALSE(std::isnan(ekf.get_state().data[i]));
  }
}

// An empty observation list is a no-op, not a crash or a NaN.
TEST(PointsUpdateTest, EmptyObservationsIsNoOp)
{
  calib_ekf_math::EKFWrapper ekf;
  ekf.reset(makeCameraState(), calib_ekf_math::Covariance());

  const auto result = ekf.update_points({}, calib_ekf_math::PointsUpdateConfig());
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.n_used, 0u);
}

// A gross outlier must be gated out while the good landmarks are still fused.
TEST(PointsUpdateTest, ChiSquareGateRejectsOutlier)
{
  calib_ekf_math::EKFWrapper ekf;
  calib_ekf_math::State s = makeCameraState();
  std::array<double, calib_ekf_math::Covariance::size> c{};
  c[calib_ekf_math::Covariance::X] = 1e-4;
  c[calib_ekf_math::Covariance::Y] = 1e-4;
  c[calib_ekf_math::Covariance::Z] = 1e-4;
  ekf.reset(s, calib_ekf_math::Covariance(c));

  const Eigen::Matrix3d R_nom = Eigen::Matrix3d::Identity();
  std::vector<Eigen::Vector3d> pts = {
    {6.0, 0.5, 0.3}, {5.0, -0.4, 0.2}, {7.0, 0.1, -0.5}, {6.5, -0.2, 0.4}};

  std::vector<calib_ekf_math::LandmarkObservation> obs;
  for (const auto & p : pts) {
    calib_ekf_math::LandmarkObservation o;
    o.p_world = p;
    o.pixel = reprojectReference(s, p, R_nom);   // perfect measurement
    o.sigma_px = 1.0;
    obs.push_back(o);
  }
  // Corrupt one of them far beyond the gate.
  obs[2].pixel += Eigen::Vector2d(120.0, -90.0);

  const auto result = ekf.update_points(obs, calib_ekf_math::PointsUpdateConfig());

  EXPECT_EQ(result.n_used, 3u);
  EXPECT_EQ(result.n_rejected, 1u);
}

// The exact algebraic statement of the lever arm / position degeneracy: moving
// the camera origin by delta and the drone by -R*delta leaves every pixel bit
// for bit unchanged. This is the root cause of the extrinsic translation being
// unobservable, and it holds for any landmark at any depth.
TEST(PointsObservabilityTest, LeverArmTradesExactlyAgainstPosition)
{
  const Eigen::Matrix3d R_nom = Eigen::Matrix3d::Identity();

  std::mt19937 rng(3);
  std::uniform_real_distribution<double> d(-0.3, 0.3);

  for (int trial = 0; trial < 20; ++trial) {
    calib_ekf_math::State s = makeCameraState();
    for (int i = 0; i < 3; ++i) {s.data[i] = d(rng);}
    Eigen::Quaterniond q(1.0 + d(rng), d(rng), d(rng), d(rng));
    q.normalize();
    s.data[calib_ekf_math::State::QW] = q.w();
    s.data[calib_ekf_math::State::QX] = q.x();
    s.data[calib_ekf_math::State::QY] = q.y();
    s.data[calib_ekf_math::State::QZ] = q.z();

    const Eigen::Vector3d p_world = q * Eigen::Vector3d(6.0, d(rng), d(rng)) +
      Eigen::Vector3d(s.data[0], s.data[1], s.data[2]);

    const Eigen::Vector2d before = reprojectReference(s, p_world, R_nom);

    // e -> e + delta, t -> t - R*delta
    const Eigen::Vector3d delta(0.03, -0.02, 0.05);
    const Eigen::Vector3d compensation = -(q.toRotationMatrix() * delta);
    calib_ekf_math::State s2 = s;
    s2.data[calib_ekf_math::State::EX] += delta[0];
    s2.data[calib_ekf_math::State::EY] += delta[1];
    s2.data[calib_ekf_math::State::EZ] += delta[2];
    s2.data[calib_ekf_math::State::X] += compensation[0];
    s2.data[calib_ekf_math::State::Y] += compensation[1];
    s2.data[calib_ekf_math::State::Z] += compensation[2];

    const Eigen::Vector2d after = reprojectReference(s2, p_world, R_nom);
    ASSERT_NEAR(before[0], after[0], 1e-9) << "trial " << trial;
    ASSERT_NEAR(before[1], after[1], 1e-9) << "trial " << trial;

    // And the generated model must agree that this direction is in the null
    // space of H: H * (the degenerate direction) == 0.
    double h[2], H[2 * N], depth = 0.0;
    evalPoint(s, p_world, R_nom, h, H, &depth);
    Eigen::Map<const Eigen::Matrix<double, 2, N, Eigen::ColMajor>> H_gen(H);

    Eigen::Matrix<double, N, 1> dir = Eigen::Matrix<double, N, 1>::Zero();
    dir.segment(calib_ekf_math::State::EX, 3) = delta;
    dir.segment(calib_ekf_math::State::X, 3) = compensation;
    const Eigen::Vector2d Hd = H_gen * dir;
    EXPECT_NEAR(Hd.norm(), 0.0, 1e-6)
      << "H does not annihilate the lever-arm/position degeneracy direction";
  }
}

// With the drone position free, the lever arm can hide behind it: pure
// translation must not resolve it, while rotation must. A change that appears to
// make the lever arm observable while flying a straight line is a bug, and this
// is the test that would catch it.
TEST(PointsObservabilityTest, LeverArmNeedsRotation)
{
  const Eigen::Matrix3d R_nom = Eigen::Matrix3d::Identity();
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 8; ++i) {
    pts.emplace_back(6.0 + 0.1 * i, -1.0 + 0.3 * i, -0.8 + 0.2 * i);
  }

  auto run = [&](bool rotate) {
      calib_ekf_math::EKFWrapper ekf;
      calib_ekf_math::State s = makeCameraState();
      std::array<double, calib_ekf_math::Covariance::size> c{};
      // Both the lever arm AND the drone position are uncertain, which is what
      // lets the two trade against each other.
      c[calib_ekf_math::Covariance::X] = 1e-2;
      c[calib_ekf_math::Covariance::Y] = 1e-2;
      c[calib_ekf_math::Covariance::Z] = 1e-2;
      c[calib_ekf_math::Covariance::EX] = 1e-2;
      c[calib_ekf_math::Covariance::EY] = 1e-2;
      c[calib_ekf_math::Covariance::EZ] = 1e-2;
      ekf.reset(s, calib_ekf_math::Covariance(c));

      calib_ekf_math::State truth = s;

      for (int k = 0; k < 60; ++k) {
        // Teleport the drone along a path; only the attitude differs between
        // the two cases. set_state leaves P untouched, so the only thing that
        // shrinks P is the measurement itself.
        const double yaw = rotate ? 0.03 * k : 0.0;
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

        calib_ekf_math::State moved = ekf.get_state();
        moved.data[calib_ekf_math::State::X] = 0.02 * k;
        moved.data[calib_ekf_math::State::QW] = q.w();
        moved.data[calib_ekf_math::State::QX] = q.x();
        moved.data[calib_ekf_math::State::QY] = q.y();
        moved.data[calib_ekf_math::State::QZ] = q.z();
        ekf.set_state(moved);

        truth.data[calib_ekf_math::State::X] = 0.02 * k;
        truth.data[calib_ekf_math::State::QW] = q.w();
        truth.data[calib_ekf_math::State::QX] = q.x();
        truth.data[calib_ekf_math::State::QY] = q.y();
        truth.data[calib_ekf_math::State::QZ] = q.z();

        std::vector<calib_ekf_math::LandmarkObservation> obs;
        for (const auto & p : pts) {
          calib_ekf_math::LandmarkObservation o;
          o.p_world = p;
          o.pixel = reprojectReference(truth, p, R_nom);
          o.sigma_px = 0.5;
          obs.push_back(o);
        }
        calib_ekf_math::PointsUpdateConfig cfg;
        cfg.chi2_threshold = 1e9;   // no gating: this is an observability test
        ekf.update_points(obs, cfg);
      }
      const calib_ekf_math::Covariance cov = ekf.get_state_covariance();
      // Trace of the lever arm block: total remaining uncertainty in e.
      return cov.data[calib_ekf_math::Covariance::EX] +
             cov.data[calib_ekf_math::Covariance::EY] +
             cov.data[calib_ekf_math::Covariance::EZ];
    };

  const double p_translation_only = run(false);
  const double p_with_rotation = run(true);

  EXPECT_LT(p_with_rotation, p_translation_only)
    << "rotation did not improve lever arm observability: "
    << "translation-only trace = " << p_translation_only
    << ", with-rotation trace = " << p_with_rotation;
}
