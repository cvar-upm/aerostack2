// Copyright 2026 Universidad Politécnica de Madrid
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
* @file as2_safety_monitor_gtest.cpp
*
* as2_safety_monitor test file.
*
* @author Javilinos
*/

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "as2_safety_monitor/as2_safety_monitor.hpp"
#include "as2_safety_monitor/geocage_check.hpp"
#include "as2_safety_monitor/safety_check.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using as2_safety_monitor::AlertTrigger;
using as2_safety_monitor::SafetyMonitor;

rclcpp::NodeOptions monitorOptions(
  const std::string & ns, const std::vector<rclcpp::Parameter> & parameters)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/" + ns});
  options.parameter_overrides(parameters);
  return options;
}

std::vector<rclcpp::Parameter> replaced(
  std::vector<rclcpp::Parameter> parameters, const rclcpp::Parameter & parameter)
{
  for (rclcpp::Parameter & current : parameters) {
    if (current.get_name() == parameter.get_name()) {
      current = parameter;
    }
  }
  return parameters;
}

std::vector<rclcpp::Parameter> batteryParameters(double seconds)
{
  return {
    rclcpp::Parameter("battery.enabled", true),
    rclcpp::Parameter("battery.topic", "sensor_measurements/battery"),
    rclcpp::Parameter("battery.alert", 0),
    rclcpp::Parameter("battery.description", "Battery voltage low"),
    rclcpp::Parameter("battery.seconds", seconds),
    rclcpp::Parameter("battery.frequency", 50.0),
    rclcpp::Parameter("battery.voltage_threshold", 21.0),
  };
}

std::vector<rclcpp::Parameter> geocageParameters(const std::string & frame_id)
{
  return {
    rclcpp::Parameter("geocage.enabled", true),
    rclcpp::Parameter("geocage.topic", "self_localization/pose"),
    rclcpp::Parameter("geocage.alert", 0),
    rclcpp::Parameter("geocage.description", "Out of the geocage"),
    rclcpp::Parameter("geocage.seconds", 0.0),
    rclcpp::Parameter("geocage.frequency", 50.0),
    rclcpp::Parameter("geocage.frame_id", frame_id),
    rclcpp::Parameter("geocage.polygon", "[[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]"),
    rclcpp::Parameter("geocage.z_min", 0.0),
    rclcpp::Parameter("geocage.z_max", 2.0),
  };
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point position;
  position.x = x;
  position.y = y;
  position.z = z;
  return position;
}

}  // namespace

TEST(AlertTriggerTest, InstantAlertOncePerEpisode) {
  AlertTrigger trigger(0.0);
  EXPECT_FALSE(trigger.update(false, 0.0));
  EXPECT_TRUE(trigger.update(true, 1.0));
  EXPECT_FALSE(trigger.update(true, 2.0));
  EXPECT_FALSE(trigger.update(false, 3.0));
  EXPECT_TRUE(trigger.update(true, 4.0));
}

TEST(AlertTriggerTest, ViolationMustLast) {
  AlertTrigger trigger(2.0);
  EXPECT_FALSE(trigger.update(true, 10.0));
  EXPECT_FALSE(trigger.update(true, 11.9));
  EXPECT_TRUE(trigger.update(true, 12.0));
  EXPECT_FALSE(trigger.update(true, 13.0));
  EXPECT_FALSE(trigger.update(false, 14.0));
  // A cleared violation counts again from its restart
  EXPECT_FALSE(trigger.update(true, 15.0));
  EXPECT_FALSE(trigger.update(true, 16.0));
  EXPECT_TRUE(trigger.update(true, 17.0));
}

TEST(GeocageTest, InsideConcavePolygonAndHeightRange) {
  // U shape, whose notch between x = 1 and x = 2 above y = 1 is outside
  std::vector<std::array<double, 2>> polygon = as2_safety_monitor::toPolygon(
    "[[0.0, 0.0], [3.0, 0.0], [3.0, 3.0], [2.0, 3.0], [2.0, 1.0], [1.0, 1.0], [1.0, 3.0], "
    "[0.0, 3.0]]");
  EXPECT_TRUE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(0.5, 2.0, 1.0)));
  EXPECT_TRUE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(2.5, 2.0, 1.0)));
  EXPECT_TRUE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(1.5, 0.5, 1.0)));
  EXPECT_FALSE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(1.5, 2.0, 1.0)));
  EXPECT_FALSE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(4.0, 2.0, 1.0)));
  EXPECT_FALSE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(0.5, 2.0, 2.5)));
  EXPECT_FALSE(as2_safety_monitor::isInsideCage(polygon, 0.0, 2.0, point(0.5, 2.0, -0.5)));
}

TEST(GeocageTest, PolygonIsListOfPairs) {
  const std::vector<std::array<double, 2>> triangle =
    as2_safety_monitor::toPolygon("[[0.0, 0.0], [1.5, 0.0], [1.0, -2.0]]");
  ASSERT_EQ(triangle.size(), 3u);
  EXPECT_DOUBLE_EQ(triangle[1][0], 1.5);
  EXPECT_DOUBLE_EQ(triangle[2][1], -2.0);
  EXPECT_EQ(as2_safety_monitor::toPolygon("[[0, 0], [1, 0], [1, 1]]").size(), 3u);

  EXPECT_THROW(as2_safety_monitor::toPolygon("[[0.0, 0.0], [1.0, 0.0]]"), std::invalid_argument);
  EXPECT_THROW(
    as2_safety_monitor::toPolygon("[[0.0, 0.0], [1.0, 0.0, 2.0], [1.0, 1.0]]"),
    std::invalid_argument);
  EXPECT_THROW(
    as2_safety_monitor::toPolygon("[0.0, 0.0, 1.0, 0.0, 1.0, 1.0]"), std::invalid_argument);
  EXPECT_THROW(
    as2_safety_monitor::toPolygon("[[0.0, 0.0], [x, 0.0], [1.0, 1.0]]"), std::invalid_argument);
  EXPECT_THROW(
    as2_safety_monitor::toPolygon("[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]"), std::invalid_argument);
  EXPECT_THROW(as2_safety_monitor::toPolygon(""), std::invalid_argument);
}

TEST(SafetyMonitorTest, DefaultConfigLoads) {
  const std::string config_file =
    ament_index_cpp::get_package_share_directory("as2_safety_monitor") +
    "/config/config_default.yaml";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/default_config", "--params-file", config_file});
  const std::vector<rclcpp::Parameter> enabled{
    rclcpp::Parameter("battery.enabled", true), rclcpp::Parameter("geocage.enabled", true)};
  options.parameter_overrides(enabled);
  EXPECT_NO_THROW(std::make_shared<SafetyMonitor>(options));
}

TEST(SafetyMonitorTest, NoCheckEnabled) {
  EXPECT_NO_THROW(std::make_shared<SafetyMonitor>(monitorOptions("no_check", {})));
}

TEST(SafetyMonitorTest, RejectsInvalidParameters) {
  const std::vector<rclcpp::Parameter> battery = batteryParameters(0.0);
  const std::vector<rclcpp::Parameter> geocage = geocageParameters("/earth");

  const rclcpp::NodeOptions missing =
    monitorOptions("invalid", {rclcpp::Parameter("battery.enabled", true)});
  EXPECT_THROW(
    std::make_shared<SafetyMonitor>(missing),
    rclcpp::exceptions::UninitializedStaticallyTypedParameterException);

  const rclcpp::NodeOptions integer_seconds =
    monitorOptions("invalid", replaced(battery, rclcpp::Parameter("battery.seconds", 5)));
  EXPECT_THROW(
    std::make_shared<SafetyMonitor>(integer_seconds),
    rclcpp::exceptions::InvalidParameterTypeException);

  const rclcpp::NodeOptions zero_frequency =
    monitorOptions("invalid", replaced(battery, rclcpp::Parameter("battery.frequency", 0.0)));
  EXPECT_THROW(std::make_shared<SafetyMonitor>(zero_frequency), std::invalid_argument);

  const rclcpp::NodeOptions wide_alert =
    monitorOptions("invalid", replaced(battery, rclcpp::Parameter("battery.alert", 200)));
  EXPECT_THROW(std::make_shared<SafetyMonitor>(wide_alert), std::invalid_argument);

  const rclcpp::NodeOptions inverted_heights =
    monitorOptions("invalid", replaced(geocage, rclcpp::Parameter("geocage.z_max", -1.0)));
  EXPECT_THROW(std::make_shared<SafetyMonitor>(inverted_heights), std::invalid_argument);

  const rclcpp::NodeOptions two_vertices = monitorOptions(
    "invalid",
    replaced(geocage, rclcpp::Parameter("geocage.polygon", "[[0.0, 0.0], [1.0, 0.0]]")));
  EXPECT_THROW(std::make_shared<SafetyMonitor>(two_vertices), std::invalid_argument);

  const std::vector<double> flat{0.0, 0.0, 1.0, 0.0, 1.0, 1.0};
  const rclcpp::NodeOptions flat_polygon =
    monitorOptions("invalid", replaced(geocage, rclcpp::Parameter("geocage.polygon", flat)));
  EXPECT_THROW(
    std::make_shared<SafetyMonitor>(flat_polygon),
    rclcpp::exceptions::InvalidParameterTypeException);
}

class SafetyMonitorNodeTest : public ::testing::Test
{
protected:
  void start(const std::string & ns, const std::vector<rclcpp::Parameter> & parameters)
  {
    monitor_ = std::make_shared<SafetyMonitor>(monitorOptions(ns, parameters));
    helper_ = std::make_shared<rclcpp::Node>("helper", ns);
    alert_sub_ = helper_->create_subscription<as2_msgs::msg::AlertEvent>(
      "alert_event", rclcpp::QoS(10),
      [this](as2_msgs::msg::AlertEvent::ConstSharedPtr msg) {alerts_.push_back(*msg);});
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(monitor_);
    executor_->add_node(helper_);
  }

  // Spin, publishing before every spin, until done or the timeout
  bool spinUntil(
    const std::function<void()> & publish, const std::function<bool()> & done,
    std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      executor_->spin_some(std::chrono::milliseconds(10));
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return done();
  }

  // Spin until the input reaches the monitor and its alerts reach the helper
  bool waitForMatch(const rclcpp::PublisherBase & input)
  {
    const std::function<bool()> matched = [this, &input]() {
        return input.get_subscription_count() > 0 && alert_sub_->get_publisher_count() > 0;
      };
    return spinUntil([]() {}, matched, std::chrono::seconds(5));
  }

  std::shared_ptr<SafetyMonitor> monitor_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Subscription<as2_msgs::msg::AlertEvent>::SharedPtr alert_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<as2_msgs::msg::AlertEvent> alerts_;
};

TEST_F(SafetyMonitorNodeTest, BatteryAlertsOncePerEpisode) {
  start("battery_episode", batteryParameters(0.0));
  auto battery_pub = helper_->create_publisher<sensor_msgs::msg::BatteryState>(
    "sensor_measurements/battery", rclcpp::SensorDataQoS());
  ASSERT_TRUE(waitForMatch(*battery_pub));

  sensor_msgs::msg::BatteryState battery;
  battery.voltage = 20.0f;
  const std::function<void()> publish = [&]() {battery_pub->publish(battery);};
  ASSERT_TRUE(spinUntil(publish, [this]() {return alerts_.size() == 1;}, std::chrono::seconds(5)));
  EXPECT_EQ(alerts_.front().alert, 0);
  EXPECT_EQ(alerts_.front().description, "Battery voltage low");

  // Still violated, so no second alert
  spinUntil(publish, []() {return false;}, std::chrono::milliseconds(500));
  EXPECT_EQ(alerts_.size(), 1u);

  // Cleared, then violated again: a new episode
  battery.voltage = 24.0f;
  spinUntil(publish, []() {return false;}, std::chrono::milliseconds(500));
  battery.voltage = 20.0f;
  ASSERT_TRUE(spinUntil(publish, [this]() {return alerts_.size() == 2;}, std::chrono::seconds(5)));

  // A non positive reading is not evaluated
  battery.voltage = 0.0f;
  spinUntil(publish, []() {return false;}, std::chrono::milliseconds(300));
  EXPECT_EQ(alerts_.size(), 2u);
}

TEST_F(SafetyMonitorNodeTest, BatteryWaitsForSeconds) {
  start("battery_seconds", batteryParameters(0.5));
  auto battery_pub = helper_->create_publisher<sensor_msgs::msg::BatteryState>(
    "sensor_measurements/battery", rclcpp::SensorDataQoS());
  ASSERT_TRUE(waitForMatch(*battery_pub));

  sensor_msgs::msg::BatteryState battery;
  battery.voltage = 20.0f;
  const std::function<void()> publish = [&]() {battery_pub->publish(battery);};
  const auto start_time = std::chrono::steady_clock::now();
  ASSERT_TRUE(spinUntil(publish, [this]() {return !alerts_.empty();}, std::chrono::seconds(5)));
  EXPECT_GE(std::chrono::steady_clock::now() - start_time, std::chrono::milliseconds(450));
  EXPECT_EQ(alerts_.front().description, "Battery voltage low");
}

TEST_F(SafetyMonitorNodeTest, GeocageAlertsWhenLeaving) {
  start("geocage_earth", geocageParameters("/earth"));
  auto pose_pub = helper_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", rclcpp::QoS(10));
  ASSERT_TRUE(waitForMatch(*pose_pub));

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.position = point(0.0, 0.0, 1.0);
  pose.pose.orientation.w = 1.0;
  const std::function<void()> publish = [&]() {pose_pub->publish(pose);};
  spinUntil(publish, []() {return false;}, std::chrono::milliseconds(300));
  EXPECT_TRUE(alerts_.empty());

  pose.pose.position = point(0.0, 0.0, 2.5);
  ASSERT_TRUE(spinUntil(publish, [this]() {return alerts_.size() == 1;}, std::chrono::seconds(5)));
  EXPECT_EQ(alerts_.front().description, "Out of the geocage");
}

TEST_F(SafetyMonitorNodeTest, GeocageResolvesFrameThroughTf) {
  start("geocage_map", geocageParameters("map"));

  // map sits 10 m along x of earth, so the cage spans x from 9 to 11 in earth
  tf2_ros::StaticTransformBroadcaster broadcaster(helper_);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = helper_->now();
  transform.header.frame_id = "earth";
  transform.child_frame_id = "geocage_map/map";
  transform.transform.translation.x = 10.0;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);

  auto pose_pub = helper_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", rclcpp::QoS(10));
  ASSERT_TRUE(waitForMatch(*pose_pub));

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.position = point(0.0, 0.0, 1.0);
  pose.pose.orientation.w = 1.0;
  const std::function<void()> publish = [&]() {pose_pub->publish(pose);};
  ASSERT_TRUE(spinUntil(publish, [this]() {return alerts_.size() == 1;}, std::chrono::seconds(5)));

  // Inside once expressed in map, which re-arms the check only if the pose is transformed
  pose.pose.position = point(10.0, 0.0, 1.0);
  spinUntil(publish, []() {return false;}, std::chrono::milliseconds(500));
  EXPECT_EQ(alerts_.size(), 1u);
  pose.pose.position = point(0.0, 0.0, 1.0);
  ASSERT_TRUE(spinUntil(publish, [this]() {return alerts_.size() == 2;}, std::chrono::seconds(5)));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
