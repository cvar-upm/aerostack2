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
* @file safety_check.cpp
*
* Safety check templates source file.
*
* @author Javilinos
*/

#include "as2_safety_monitor/safety_check.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace as2_safety_monitor
{

namespace
{
// Age after which the last message is reported as old (s)
constexpr double kMessageTimeout = 1.0;
// Minimum time between two warnings of the same check (s)
constexpr double kWarningPeriod = 5.0;
}  // namespace

AlertTrigger::AlertTrigger(double seconds)
: seconds_(seconds)
{
}

bool AlertTrigger::update(bool violated, double time)
{
  if (!violated) {
    violating_ = false;
    fired_ = false;
    return false;
  }
  if (!violating_) {
    violating_ = true;
    violation_start_ = time;
  }
  if (fired_ || time - violation_start_ < seconds_) {
    return false;
  }
  fired_ = true;
  return true;
}

CheckConfig readCheckConfig(as2::Node * node, const std::string & name)
{
  CheckConfig config;
  config.topic = node->getParameter<std::string>(name + ".topic");
  const int alert = node->getParameter<int>(name + ".alert");
  config.description = node->getParameter<std::string>(name + ".description");
  config.seconds = node->getParameter<double>(name + ".seconds");
  config.frequency = node->getParameter<double>(name + ".frequency");

  if (config.topic.empty()) {
    throw std::invalid_argument(name + ".topic is empty");
  }
  if (alert < std::numeric_limits<int8_t>::min() || alert > std::numeric_limits<int8_t>::max()) {
    throw std::invalid_argument(name + ".alert is not an int8 AlertEvent code");
  }
  if (!std::isfinite(config.seconds) || config.seconds < 0.0) {
    throw std::invalid_argument(name + ".seconds must be finite and >= 0");
  }
  if (!std::isfinite(config.frequency) || config.frequency <= 0.0) {
    throw std::invalid_argument(name + ".frequency must be finite and > 0");
  }
  config.alert = static_cast<int8_t>(alert);
  return config;
}

SafetyCheckBase::SafetyCheckBase(
  as2::Node * node, const std::string & name, const AlertPublisher::SharedPtr & alert_pub)
: node_(node), name_(name), config_(readCheckConfig(node, name)), trigger_(config_.seconds),
  alert_pub_(alert_pub)
{
}

void SafetyCheckBase::messageReceived()
{
  last_msg_time_ = node_->now();
}

bool SafetyCheckBase::messageAvailable()
{
  if (!last_msg_time_.has_value()) {
    warnThrottled("No message received on " + config_.topic);
    return false;
  }
  const double age = (node_->now() - last_msg_time_.value()).seconds();
  if (age > kMessageTimeout) {
    char text[64];
    std::snprintf(text, sizeof(text), " for %.1f s, checking the last one", age);
    warnThrottled("No new message on " + config_.topic + text);
  }
  return true;
}

bool SafetyCheckBase::updateTrigger(bool violated)
{
  return trigger_.update(violated, node_->now().seconds());
}

void SafetyCheckBase::publishAlert(const std::string & details)
{
  as2_msgs::msg::AlertEvent msg;
  msg.alert = config_.alert;
  msg.description = config_.description;
  alert_pub_->publish(msg);

  std::string cause = details;
  if (config_.seconds > 0.0) {
    char duration[32];
    std::snprintf(duration, sizeof(duration), " for %.1f s", config_.seconds);
    cause += duration;
  }
  RCLCPP_WARN(
    node_->get_logger(), "Alert %d [%s]: %s (%s)", msg.alert, name_.c_str(),
    msg.description.c_str(), cause.c_str());
}

void SafetyCheckBase::warnThrottled(const std::string & text)
{
  // Per check, since RCLCPP_WARN_THROTTLE keeps one throttle for every check
  const rclcpp::Time now = node_->now();
  if (last_warning_time_.has_value() &&
    (now - last_warning_time_.value()).seconds() < kWarningPeriod)
  {
    return;
  }
  last_warning_time_ = now;
  RCLCPP_WARN(node_->get_logger(), "[%s] %s", name_.c_str(), text.c_str());
}

}  // namespace as2_safety_monitor
