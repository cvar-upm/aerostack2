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
* @file ekf_history_buffer_gtest.cpp
*
* Unit tests for EkfHistoryBuffer: out-of-sequence (delayed) pose/odom update
* handling via rewind + replay.
*
* @authors Rodrigo Da Silva Gómez
*/

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>

#include <ekf_calib/ekf_datatype.hpp>
#include <ekf_calib/ekf_wrapper.hpp>

#include "calib_ekf/ekf_history_buffer.hpp"
#include "calib_ekf/calib_ekf_utils.hpp"

namespace calib_ekf
{
namespace
{

// Offset all test timestamps away from t=0 so that `now - max_update_latency`
// never produces a negative-seconds rclcpp::Time (which the (sec, nsec)
// constructors reject).
constexpr double kBaseTime = 10.0;

rclcpp::Time t(double seconds)
{
  return rclcpp::Time(0, 0, RCL_ROS_TIME) + rclcpp::Duration::from_seconds(kBaseTime + seconds);
}

calib_ekf_math::Covariance makeDiagonalCovariance(double value)
{
  std::array<double, calib_ekf_math::Covariance::size> data{};
  for (std::size_t i = 0; i < calib_ekf_math::State::size; ++i) {
    data[i * (calib_ekf_math::State::size + 1)] = value;
  }
  return calib_ekf_math::Covariance(data);
}

// Returned as a unique_ptr: EKFWrapper's CasADi argument/result tables hold
// raw pointers into its own members, so it must never be copied or moved.
std::unique_ptr<calib_ekf_math::EKFWrapper> makeWrapper(const calib_ekf_math::State & initial_state = calib_ekf_math::State())
{
  auto wrapper = std::make_unique<calib_ekf_math::EKFWrapper>(
    initial_state,
    makeDiagonalCovariance(0.01),
    Eigen::Vector<double, 6>::Constant(1e-3),
    1e-3, 1e-4, 1e-4, 1e-5);
  wrapper->set_gravity(calib_ekf_math::Gravity({0.0, 0.0, 0.0}));
  return wrapper;
}

calib_ekf_math::Input zeroInput()
{
  return calib_ekf_math::Input({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

// Takes roll/pitch/yaw for convenience and converts to the quaternion the
// measurement actually carries.
calib_ekf_math::PoseMeasurement makeMeasurement(
  double x, double y, double z,
  double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  return calib_ekf_math::PoseMeasurement({x, y, z, q.w(), q.x(), q.y(), q.z()});
}

calib_ekf_math::PoseMeasurementCovariance makeMeasurementCovariance(double value)
{
  return calib_ekf_math::PoseMeasurementCovariance({value, value, value, value, value, value});
}

}  // namespace

// ---------------------------------------------------------------------------
// (a) No-delay update matches a direct EKFWrapper call
// ---------------------------------------------------------------------------

TEST(EkfHistoryBufferTest, NoDelayUpdatePoseMatchesDirectCall)
{
  auto direct_wrapper = makeWrapper();
  auto buffered_wrapper = makeWrapper();
  EkfHistoryBuffer buffer(*buffered_wrapper, 1000.0);

  const calib_ekf_math::PoseMeasurement raw_meas = makeMeasurement(1.0, 2.0, 3.0, 0.1, -0.2, 0.3);
  const calib_ekf_math::PoseMeasurementCovariance cov = makeMeasurementCovariance(1e-4);

  direct_wrapper->update_pose(raw_meas, cov);

  UpdateResult result = buffer.updateAndRecord(
    t(0.0), EkfOperationType::UPDATE_POSE, raw_meas, cov, t(0.0));

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(direct_wrapper->get_state().data, buffered_wrapper->get_state().data);
  EXPECT_EQ(
    direct_wrapper->get_state_covariance().data, buffered_wrapper->get_state_covariance().data);
  EXPECT_EQ(direct_wrapper->get_map_to_odom(), buffered_wrapper->get_map_to_odom());
  EXPECT_EQ(
    direct_wrapper->get_map_to_odom_velocity(), buffered_wrapper->get_map_to_odom_velocity());
}

TEST(EkfHistoryBufferTest, NoDelayUpdatePoseOdomMatchesDirectCall)
{
  auto direct_wrapper = makeWrapper();
  auto buffered_wrapper = makeWrapper();
  EkfHistoryBuffer buffer(*buffered_wrapper, 1000.0);

  const calib_ekf_math::PoseMeasurement raw_meas = makeMeasurement(1.0, 2.0, 3.0, 0.1, -0.2, 0.3);
  const calib_ekf_math::PoseMeasurementCovariance cov = makeMeasurementCovariance(1e-4);

  direct_wrapper->update_pose_odom(raw_meas, cov);

  UpdateResult result = buffer.updateAndRecord(
    t(0.0), EkfOperationType::UPDATE_POSE_ODOM, raw_meas, cov, t(0.0));

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(direct_wrapper->get_state().data, buffered_wrapper->get_state().data);
  EXPECT_EQ(
    direct_wrapper->get_state_covariance().data, buffered_wrapper->get_state_covariance().data);
  // update_pose_odom never touches map_to_odom — both remain identity.
  EXPECT_EQ(buffered_wrapper->get_map_to_odom(), Eigen::Matrix4d::Identity());
  EXPECT_EQ(direct_wrapper->get_map_to_odom(), buffered_wrapper->get_map_to_odom());
  EXPECT_EQ(
    direct_wrapper->get_map_to_odom_velocity(), buffered_wrapper->get_map_to_odom_velocity());
}

// ---------------------------------------------------------------------------
// (b) Delayed update reproduces the in-order result
// ---------------------------------------------------------------------------

TEST(EkfHistoryBufferTest, DelayedUpdateReproducesInOrderResult)
{
  calib_ekf_math::State initial_state;
  initial_state.data[calib_ekf_math::State::VX] = 1.0;

  auto in_order_wrapper = makeWrapper(initial_state);
  auto delayed_wrapper = makeWrapper(initial_state);
  EkfHistoryBuffer in_order_buffer(*in_order_wrapper, 1000.0);
  EkfHistoryBuffer delayed_buffer(*delayed_wrapper, 1000.0);

  const calib_ekf_math::Input input = zeroInput();
  const double dt = 0.1;
  const calib_ekf_math::PoseMeasurement raw_meas = makeMeasurement(0.05, 0.0, 0.0);
  const calib_ekf_math::PoseMeasurementCovariance cov = makeMeasurementCovariance(1e-4);

  // In-order: predict to t=0.1, apply the correction immediately, then
  // continue predicting to t=0.3.
  in_order_buffer.predictAndRecord(t(0.1), input, dt);
  UpdateResult r1 = in_order_buffer.updateAndRecord(
    t(0.1), EkfOperationType::UPDATE_POSE, raw_meas, cov, t(0.1));
  ASSERT_TRUE(r1.applied);
  in_order_buffer.predictAndRecord(t(0.2), input, dt);
  in_order_buffer.predictAndRecord(t(0.3), input, dt);

  // Delayed: predict all the way to t=0.3 first, then apply the *same*
  // correction late, stamped at t=0.1.
  delayed_buffer.predictAndRecord(t(0.1), input, dt);
  delayed_buffer.predictAndRecord(t(0.2), input, dt);
  delayed_buffer.predictAndRecord(t(0.3), input, dt);
  UpdateResult r2 = delayed_buffer.updateAndRecord(
    t(0.1), EkfOperationType::UPDATE_POSE, raw_meas, cov, t(0.3));
  ASSERT_TRUE(r2.applied);

  // Rewind + replay must reproduce exactly the in-order state/covariance.
  EXPECT_EQ(in_order_wrapper->get_state().data, delayed_wrapper->get_state().data);
  EXPECT_EQ(
    in_order_wrapper->get_state_covariance().data, delayed_wrapper->get_state_covariance().data);
  EXPECT_EQ(in_order_buffer.size(), delayed_buffer.size());
}

// ---------------------------------------------------------------------------
// (c) Stale measurements are dropped, state left untouched
// ---------------------------------------------------------------------------

TEST(EkfHistoryBufferTest, StaleMeasurementIsDropped)
{
  auto wrapper = makeWrapper();
  EkfHistoryBuffer buffer(*wrapper, 300.0);  // max_update_latency = 300 ms

  buffer.predictAndRecord(t(1.0), zeroInput(), 0.1);

  const calib_ekf_math::State state_before = wrapper->get_state();
  const calib_ekf_math::Covariance covariance_before = wrapper->get_state_covariance();
  const Eigen::Matrix4d map_to_odom_before = wrapper->get_map_to_odom();
  const Eigen::Vector3d map_to_odom_velocity_before = wrapper->get_map_to_odom_velocity();
  const std::size_t size_before = buffer.size();

  // now - stamp = 0.5 s > 300 ms -> dropped.
  UpdateResult result = buffer.updateAndRecord(
    t(0.5), EkfOperationType::UPDATE_POSE,
    makeMeasurement(1.0, 2.0, 3.0), makeMeasurementCovariance(1e-4), t(1.0));

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(wrapper->get_state().data, state_before.data);
  EXPECT_EQ(wrapper->get_state_covariance().data, covariance_before.data);
  EXPECT_EQ(wrapper->get_map_to_odom(), map_to_odom_before);
  EXPECT_EQ(wrapper->get_map_to_odom_velocity(), map_to_odom_velocity_before);
  EXPECT_EQ(buffer.size(), size_before);
}

// ---------------------------------------------------------------------------
// (d) Interleaved sources
// ---------------------------------------------------------------------------

TEST(EkfHistoryBufferTest, EqualStampInsertedAfterExistingEntry)
{
  auto wrapper = makeWrapper();
  EkfHistoryBuffer buffer(*wrapper, 1000.0);

  const calib_ekf_math::Input input = zeroInput();
  buffer.predictAndRecord(t(0.1), input, 0.1);
  buffer.predictAndRecord(t(0.2), input, 0.1);

  UpdateResult result = buffer.updateAndRecord(
    t(0.1), EkfOperationType::UPDATE_POSE_ODOM,
    makeMeasurement(0.01, 0.0, 0.0), makeMeasurementCovariance(1e-4), t(0.2));
  ASSERT_TRUE(result.applied);

  // The new entry (stamp == 0.1) must land *after* the existing PREDICT
  // entry with the same stamp (std::upper_bound tie-break).
  ASSERT_EQ(buffer.size(), 3u);
  EXPECT_EQ(buffer.at(0).stamp, t(0.1));
  EXPECT_EQ(buffer.at(0).type, EkfOperationType::PREDICT);
  EXPECT_EQ(buffer.at(1).stamp, t(0.1));
  EXPECT_EQ(buffer.at(1).type, EkfOperationType::UPDATE_POSE_ODOM);
  EXPECT_EQ(buffer.at(2).stamp, t(0.2));
  EXPECT_EQ(buffer.at(2).type, EkfOperationType::PREDICT);
}

TEST(EkfHistoryBufferTest, InterleavedSourcesPreserveTypeAndMapToOdom)
{
  auto wrapper = makeWrapper();
  EkfHistoryBuffer buffer(*wrapper, 1000.0);

  const calib_ekf_math::Input input = zeroInput();
  const double dt = 0.1;
  const calib_ekf_math::PoseMeasurementCovariance cov = makeMeasurementCovariance(1e-4);

  // Predict to t=0.4 and apply a non-delayed UPDATE_POSE there (jumps map_to_odom).
  buffer.predictAndRecord(t(0.1), input, dt);
  buffer.predictAndRecord(t(0.2), input, dt);
  buffer.predictAndRecord(t(0.3), input, dt);
  buffer.predictAndRecord(t(0.4), input, dt);

  const calib_ekf_math::PoseMeasurement meas_a = makeMeasurement(0.5, 0.0, 0.0);
  UpdateResult r_a = buffer.updateAndRecord(
    t(0.4), EkfOperationType::UPDATE_POSE, meas_a, cov, t(0.4));
  ASSERT_TRUE(r_a.applied);

  const Eigen::Matrix4d map_to_odom_after_a = wrapper->get_map_to_odom();
  const Eigen::Vector3d map_to_odom_velocity_after_a = wrapper->get_map_to_odom_velocity();
  EXPECT_NE(map_to_odom_after_a, Eigen::Matrix4d::Identity());

  // Continue predicting to t=0.6.
  buffer.predictAndRecord(t(0.5), input, dt);
  buffer.predictAndRecord(t(0.6), input, dt);

  // A delayed UPDATE_POSE_ODOM measurement arrives, stamped *before* the
  // already-applied t=0.4 UPDATE_POSE.
  const calib_ekf_math::PoseMeasurement meas_c = makeMeasurement(0.0, 0.3, 0.0);
  UpdateResult r_c = buffer.updateAndRecord(
    t(0.2), EkfOperationType::UPDATE_POSE_ODOM, meas_c, cov, t(0.6));
  ASSERT_TRUE(r_c.applied);

  // The replayed t=0.4 entry must still be applied via update_pose_odom (no
  // re-jump), so map_to_odom / map_to_odom_velocity are unchanged from the
  // values captured right after the first correction.
  EXPECT_EQ(wrapper->get_map_to_odom(), map_to_odom_after_a);
  EXPECT_EQ(wrapper->get_map_to_odom_velocity(), map_to_odom_velocity_after_a);

  // The t=0.4 entry's recorded type must be preserved as UPDATE_POSE after
  // replay (note: the PREDICT entry recorded at t=0.4 shares the same stamp,
  // so distinguish by type, not stamp).
  bool found_update_pose = false;
  for (std::size_t i = 0; i < buffer.size(); ++i) {
    if (buffer.at(i).type == EkfOperationType::UPDATE_POSE) {
      EXPECT_EQ(buffer.at(i).stamp, t(0.4));
      found_update_pose = true;
    }
  }
  EXPECT_TRUE(found_update_pose);

  // Final state must reflect both corrections: a control run that only
  // applies the t=0.4 correction must end up in a different state.
  auto control_wrapper = makeWrapper();
  EkfHistoryBuffer control_buffer(*control_wrapper, 1000.0);
  control_buffer.predictAndRecord(t(0.1), input, dt);
  control_buffer.predictAndRecord(t(0.2), input, dt);
  control_buffer.predictAndRecord(t(0.3), input, dt);
  control_buffer.predictAndRecord(t(0.4), input, dt);
  control_buffer.updateAndRecord(t(0.4), EkfOperationType::UPDATE_POSE, meas_a, cov, t(0.4));
  control_buffer.predictAndRecord(t(0.5), input, dt);
  control_buffer.predictAndRecord(t(0.6), input, dt);

  EXPECT_NE(wrapper->get_state().data, control_wrapper->get_state().data);
}

// ---------------------------------------------------------------------------
// (e) Trim respects max_update_latency
// ---------------------------------------------------------------------------

TEST(EkfHistoryBufferTest, TrimRespectsMaxUpdateLatency)
{
  auto wrapper = makeWrapper();
  const double max_latency_ms = 300.0;
  EkfHistoryBuffer buffer(*wrapper, max_latency_ms);

  const calib_ekf_math::Input input = zeroInput();
  const double dt = 0.1;

  // 1.0 s of predicts with a 300 ms horizon -> the buffer must shrink, but
  // never below 1 entry.
  for (int i = 1; i <= 10; ++i) {
    buffer.predictAndRecord(t(0.1 * i), input, dt);
  }

  EXPECT_GE(buffer.size(), 1u);
  EXPECT_LT(buffer.size(), 10u);

  const rclcpp::Time now = t(1.0);
  const rclcpp::Time horizon = now - rclcpp::Duration::from_seconds(max_latency_ms / 1000.0);

  // A measurement exactly at the latency horizon (age == max_update_latency) is accepted.
  UpdateResult at_horizon = buffer.updateAndRecord(
    horizon, EkfOperationType::UPDATE_POSE_ODOM,
    makeMeasurement(0.0, 0.0, 0.0), makeMeasurementCovariance(1e-4), now);
  EXPECT_TRUE(at_horizon.applied);

  // A measurement older than the latency horizon (age > max_update_latency) is dropped.
  const rclcpp::Time past_horizon = horizon - rclcpp::Duration::from_seconds(0.01);
  UpdateResult past = buffer.updateAndRecord(
    past_horizon, EkfOperationType::UPDATE_POSE_ODOM,
    makeMeasurement(0.0, 0.0, 0.0), makeMeasurementCovariance(1e-4), now);
  EXPECT_FALSE(past.applied);

  // Still never empty.
  EXPECT_GE(buffer.size(), 1u);
}

// ---------------------------------------------------------------------------
// Additional cases
// ---------------------------------------------------------------------------

TEST(EkfHistoryBufferTest, FirstCallOnEmptyBufferDoesNotCrash)
{
  auto wrapper = makeWrapper();
  EkfHistoryBuffer buffer(*wrapper, 1000.0);

  ASSERT_EQ(buffer.size(), 0u);

  UpdateResult result = buffer.updateAndRecord(
    t(0.0), EkfOperationType::UPDATE_POSE_ODOM,
    makeMeasurement(1.0, 2.0, 3.0), makeMeasurementCovariance(1e-4), t(0.0));

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(buffer.size(), 1u);
}

// Near a half turn the two quaternion representatives lie on opposite
// hemispheres. The alignment must flip the measurement onto the reference's
// hemisphere so the residual stays small for a physically small error.
TEST(AlignPoseMeasurementSignTest, FlipsOntoReferenceHemisphere)
{
  calib_ekf_math::State reference_state;
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI - 0.05);
    reference_state.data[calib_ekf_math::State::QW] = q.w();
    reference_state.data[calib_ekf_math::State::QX] = q.x();
    reference_state.data[calib_ekf_math::State::QY] = q.y();
    reference_state.data[calib_ekf_math::State::QZ] = q.z();
  }

  // Physically 0.1 rad away from the reference, but on the far hemisphere.
  calib_ekf_math::PoseMeasurement raw = makeMeasurement(0.0, 0.0, 0.0, 0.0, 0.0, -M_PI + 0.05);
  for (std::size_t i = calib_ekf_math::PoseMeasurement::QW;
    i <= calib_ekf_math::PoseMeasurement::QZ; ++i)
  {
    raw.data[i] = -raw.data[i];
  }

  const calib_ekf_math::PoseMeasurement aligned =
    alignPoseMeasurementSign(raw, reference_state);

  const double dot =
    aligned.data[calib_ekf_math::PoseMeasurement::QW] *
    reference_state.data[calib_ekf_math::State::QW] +
    aligned.data[calib_ekf_math::PoseMeasurement::QX] *
    reference_state.data[calib_ekf_math::State::QX] +
    aligned.data[calib_ekf_math::PoseMeasurement::QY] *
    reference_state.data[calib_ekf_math::State::QY] +
    aligned.data[calib_ekf_math::PoseMeasurement::QZ] *
    reference_state.data[calib_ekf_math::State::QZ];
  EXPECT_GT(dot, 0.0) << "measurement was not flipped onto the reference hemisphere";
}

}  // namespace calib_ekf
