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
 * @file calib_ekf_integration_gtest.cpp
 *
 * Integration tests for the calib_ekf plugin loaded into a real
 * StateEstimator ROS2 node. Tests verify that the plugin loads, processes
 * messages, and produces TF output without crashing.
 */

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <as2_msgs/msg/platform_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "as2_state_estimator/as2_state_estimator.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helper: create a StateEstimator node with the calib_ekf plugin
// ---------------------------------------------------------------------------

static std::shared_ptr<as2_state_estimator::StateEstimator> getSimpleEkfNode(
  const std::string & node_name_prefix = "test_calib_ekf",
  const std::vector<std::string> & extra_params = {})
{
  const std::string ns = node_name_prefix;
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_state_estimator");
  const std::string state_estimator_cfg = package_path + "/config/state_estimator_default.yaml";
  const std::string plugin_cfg =
    package_path + "/plugins/calib_ekf/config/plugin_default.yaml";

  std::vector<std::string> args = {
    "--ros-args",
    "-r", "__ns:=/" + ns,
    "-p", "namespace:=" + ns,
    "-p", "plugin_name:=calib_ekf",
    "--params-file", state_estimator_cfg,
    "--params-file", plugin_cfg,
  };
  for (const std::string & p : extra_params) {
    args.push_back("-p");
    args.push_back(p);
  }

  auto opts = rclcpp::NodeOptions().arguments(args);
  auto node = std::make_shared<as2_state_estimator::StateEstimator>(opts);
  // The PluginWrapper calls StateEstimator::getInstance() to get the current node.
  // Since we created the node directly (not via getInstance()), we must register
  // it as the singleton so that the deferred setup() can find it.
  as2_state_estimator::StateEstimator::instance_ = node;
  return node;
}

// Spin the executor for a real wall-clock duration. spin_some() returns as soon
// as the queue is empty, so iterating it is not enough to let a wall timer fire.
static void spinFor(
  rclcpp::executors::MultiThreadedExecutor & exec,
  std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(10ms);
    std::this_thread::sleep_for(5ms);
  }
}

// Spin the executor for a fixed number of iterations
static void spinSome(
  rclcpp::executors::MultiThreadedExecutor & exec,
  int iterations = 10)
{
  for (int i = 0; i < iterations; ++i) {
    exec.spin_some(50ms);
  }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(SimpleEkfIntegrationTest, PluginLoads_NoThrow)
{
  EXPECT_NO_THROW(getSimpleEkfNode("test_load"));
}

TEST(SimpleEkfIntegrationTest, PluginSpins_NoThrow)
{
  auto node = getSimpleEkfNode("test_spin");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  EXPECT_NO_THROW(spinSome(exec, 5));
}

TEST(SimpleEkfIntegrationTest, ImuMessage_NodeDoesNotCrash)
{
  auto node = getSimpleEkfNode("test_imu");

  // Helper node to publish IMU messages
  auto pub_node = rclcpp::Node::make_shared("test_imu_publisher");
  auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>(
    "/test_imu/sensor_measurements/imu", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);

  // Let the subscription set up
  spinSome(exec, 3);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = pub_node->now();
  imu.header.frame_id = "imu";
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 9.81;
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  // Identity orientation covariance
  imu.orientation_covariance[0] = -1.0;  // indicates orientation not estimated

  EXPECT_NO_THROW({
    imu_pub->publish(imu);
    spinSome(exec, 10);
  });
}

// After publishing a PoseStamped in the "map" frame, earth→map TF must appear.
// The earth frame is global ("earth"); the map frame is namespaced ("test_pose/map").
TEST(SimpleEkfIntegrationTest, PoseStamped_SetsEarthToMap_TfAvailable)
{
  auto node = getSimpleEkfNode("test_pose");

  auto pub_node = rclcpp::Node::make_shared("test_pose_publisher");
  // Match the plugin's SensorDataQoS so publisher and subscriber are compatible
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test_pose/ground_truth/pose", rclcpp::SensorDataQoS());
  auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>(
    "/test_pose/sensor_measurements/imu", rclcpp::SensorDataQoS());

  // TF listener uses a dedicated node so it stays independent of the test node
  auto tf_node = rclcpp::Node::make_shared("test_pose_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);

  // Give nodes time to discover each other
  spinSome(exec, 10);

  // Prepare messages upfront
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "test_pose/map";  // namespace-prefixed map frame
  pose.pose.orientation.w = 1.0;

  sensor_msgs::msg::Imu imu;
  imu.linear_acceleration.z = 9.81;
  imu.orientation_covariance[0] = -1.0;

  // Spin in a loop (up to 2 s), publishing on every iteration until TF appears.
  // Re-publishing ensures the message is sent after subscriptions are matched.
  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    pose.header.stamp = pub_node->now();
    imu.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    imu_pub->publish(imu);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(
      "test_pose/map", "earth", tf2::TimePointZero);
  }

  EXPECT_TRUE(tf_available) << "earth→map TF should be available after publishing first pose";
}

TEST(SimpleEkfIntegrationTest, MocapZeroPose_Rejected_NodeDoesNotCrash)
{
  auto node = getSimpleEkfNode("test_mocap_zero");
  auto pub_node = rclcpp::Node::make_shared("test_mocap_zero_pub");

  // The default config has no mocap topic, so this test just confirms that
  // even if we publish on that topic, the node doesn't crash.
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/test_mocap_zero/mocap/rigid_bodies", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  spinSome(exec, 3);

  mocap4r2_msgs::msg::RigidBodies msg;
  msg.header.stamp = pub_node->now();
  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "33";
  // All-zero pose: position=(0,0,0), orientation=(0,0,0,0) — should be rejected
  body.pose.position.x = 0.0;
  body.pose.position.y = 0.0;
  body.pose.position.z = 0.0;
  body.pose.orientation.x = 0.0;
  body.pose.orientation.y = 0.0;
  body.pose.orientation.z = 0.0;
  body.pose.orientation.w = 0.0;
  msg.rigidbodies.push_back(body);

  EXPECT_NO_THROW({
    mocap_pub->publish(msg);
    spinSome(exec, 10);
  });
}

TEST(SimpleEkfIntegrationTest, MocapDuplicatePose_Rejected_NodeDoesNotCrash)
{
  auto node = getSimpleEkfNode("test_mocap_dup");
  auto pub_node = rclcpp::Node::make_shared("test_mocap_dup_pub");

  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/test_mocap_dup/mocap/rigid_bodies", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  spinSome(exec, 3);

  // Build a valid (non-zero) mocap message
  mocap4r2_msgs::msg::RigidBodies msg;
  msg.header.stamp = pub_node->now();
  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "33";
  body.pose.position.x = 1.0;
  body.pose.position.y = 2.0;
  body.pose.position.z = 3.0;
  body.pose.orientation.w = 1.0;
  msg.rigidbodies.push_back(body);

  EXPECT_NO_THROW({
    mocap_pub->publish(msg);
    spinSome(exec, 5);
    // Publish the identical message a second time — should be silently rejected
    mocap_pub->publish(msg);
    spinSome(exec, 5);
  });
}

TEST(SimpleEkfIntegrationTest, PlatformInfoOffboardFalse_NodeDoesNotCrash)
{
  auto node = getSimpleEkfNode("test_offboard");
  auto pub_node = rclcpp::Node::make_shared("test_offboard_pub");

  auto info_pub = pub_node->create_publisher<as2_msgs::msg::PlatformInfo>(
    "/test_offboard/platform/info", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  spinSome(exec, 3);

  as2_msgs::msg::PlatformInfo info;
  info.offboard = false;

  EXPECT_NO_THROW({
    info_pub->publish(info);
    // Spin long enough for the timer callback to fire (timer_hz=100 → 10ms per tick)
    spinSome(exec, 20);
  });
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// The points update is the only measurement that observes the camera
// calibration, so its ROS path needs its own coverage: with it enabled and a
// landmark map configured, the plugin must load, subscribe, and actually consume
// a detection message.
//
// Note the spin budget: StateEstimator::setup() runs off a 1 second wall timer,
// so the plugin's onSetup() (and hence the landmark subscription) does not exist
// until the executor has been spun for longer than that.
TEST(CalibEkfIntegrationTest, PointsUpdate_Enabled_ConsumesDetections)
{
  auto node = getSimpleEkfNode(
    "test_points", {
      "calib_ekf.points_update.enabled:=true",
      "calib_ekf.points_update.landmark_ids:=[l0, l1]",
      "calib_ekf.points_update.landmarks.l0:=[5.0, 0.3, 0.1]",
      "calib_ekf.points_update.landmarks.l1:=[6.0, -0.4, 0.2]",
      "calib_ekf.initial_covariance.extrinsic_translation:=1.0e-2",
    });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  // Past the 1 second setup timer, so the plugin is actually configured.
  spinFor(exec, 1500ms);

  auto pub_node = std::make_shared<rclcpp::Node>("points_publisher");
  auto pub = pub_node->create_publisher<vision_msgs::msg::Detection2DArray>(
    "/test_points/sensor_measurements/landmarks",
    rclcpp::SensorDataQoS());
  exec.add_node(pub_node);
  spinFor(exec, 300ms);

  // The plugin must have subscribed; otherwise this test proves nothing.
  ASSERT_GT(pub->get_subscription_count(), 0u)
    << "plugin did not subscribe to the landmark topic";

  vision_msgs::msg::Detection2DArray msg;
  msg.header.stamp = node->now();
  msg.header.frame_id = "camera";
  for (const auto & id : {std::string("l0"), std::string("l1"), std::string("unknown")}) {
    vision_msgs::msg::Detection2D d;
    d.id = id;
    d.bbox.center.position.x = 320.0;
    d.bbox.center.position.y = 240.0;
    msg.detections.push_back(d);
  }

  EXPECT_NO_THROW(pub->publish(msg));
  EXPECT_NO_THROW(spinFor(exec, 300ms));

  exec.remove_node(pub_node);
  exec.remove_node(node);
}
