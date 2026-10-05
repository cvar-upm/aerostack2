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
* @file safety_check.hpp
*
* Safety check templates header file.
*
* @author Javilinos
*/

#ifndef AS2_SAFETY_MONITOR__SAFETY_CHECK_HPP_
#define AS2_SAFETY_MONITOR__SAFETY_CHECK_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "as2_core/node.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_safety_monitor
{

/**
 * @brief Raises an alert once per violation episode, after the violation has lasted a time.
 */
class AlertTrigger
{
public:
  /**
   * @brief Construct a new AlertTrigger.
   *
   * @param seconds Time the violation must last before the alert, 0 for instant.
   */
  explicit AlertTrigger(double seconds);

  /**
   * @brief Feed one evaluation, re-arming the trigger when the violation clears.
   *
   * @param violated Whether the condition is violated.
   * @param time Evaluation time, in seconds.
   * @return True only on the evaluation that raises the alert.
   */
  bool update(bool violated, double time);

private:
  double seconds_;
  double violation_start_ = 0.0;
  bool violating_ = false;
  bool fired_ = false;
};

/**
 * @brief Parameters shared by every check, read from "<check>.<field>".
 */
struct CheckConfig
{
  std::string topic;  // Checked topic, relative to the node namespace
  int8_t alert = 0;  // AlertEvent code
  std::string description;  // AlertEvent description
  double seconds = 0.0;  // Time the violation must last before the alert
  double frequency = 0.0;  // Evaluation rate of the last message (Hz)
};

/**
 * @brief Read and validate the parameters shared by every check.
 *
 * @param node Node that owns the parameters.
 * @param name Check name, used as parameter prefix.
 * @return Validated configuration.
 * @throw std::invalid_argument if a value is out of range.
 */
CheckConfig readCheckConfig(as2::Node * node, const std::string & name);

/**
 * @brief Message independent part of a check: configuration, trigger and alert.
 */
class SafetyCheckBase
{
public:
  using AlertPublisher = rclcpp::Publisher<as2_msgs::msg::AlertEvent>;

  SafetyCheckBase(const SafetyCheckBase &) = delete;
  SafetyCheckBase & operator=(const SafetyCheckBase &) = delete;
  virtual ~SafetyCheckBase() = default;

protected:
  /**
   * @brief Construct a new SafetyCheckBase, reading its configuration.
   *
   * @param node Node that owns the check.
   * @param name Check name, used as parameter prefix and in the alert description.
   * @param alert_pub Publisher of the alert events.
   */
  SafetyCheckBase(
    as2::Node * node, const std::string & name, const AlertPublisher::SharedPtr & alert_pub);

  /**
   * @brief Record the arrival of a message on the checked topic.
   */
  void messageReceived();

  /**
   * @brief Whether a message is available, warning when none arrived or the last one is old.
   *
   * @return True if there is a message to evaluate.
   */
  bool messageAvailable();

  /**
   * @brief Feed one evaluation to the trigger.
   *
   * @param violated Whether the condition is violated.
   * @return True when the alert must be raised.
   */
  bool updateTrigger(bool violated);

  /**
   * @brief Publish the alert of this check, logging the state that raised it.
   *
   * @param details State that raised the alert.
   */
  void publishAlert(const std::string & details);

  /**
   * @brief Log a warning of this check, at most once every few seconds.
   *
   * @param text Warning text.
   */
  void warnThrottled(const std::string & text);

  as2::Node * node_;
  std::string name_;
  CheckConfig config_;

private:
  AlertTrigger trigger_;
  AlertPublisher::SharedPtr alert_pub_;
  std::optional<rclcpp::Time> last_msg_time_;
  std::optional<rclcpp::Time> last_warning_time_;
};

/**
 * @brief Check that evaluates the last message of a topic at a fixed frequency.
 *
 * @tparam MessageT Message type of the checked topic.
 */
template<typename MessageT>
class SafetyCheck : public SafetyCheckBase
{
public:
  /**
   * @brief Construct a new SafetyCheck, subscribing to its topic and starting its timer.
   *
   * @param node Node that owns the check.
   * @param name Check name, used as parameter prefix and in the alert description.
   * @param qos Quality of service of the subscription.
   * @param alert_pub Publisher of the alert events.
   */
  SafetyCheck(
    as2::Node * node, const std::string & name, const rclcpp::QoS & qos,
    const AlertPublisher::SharedPtr & alert_pub)
  : SafetyCheckBase(node, name, alert_pub)
  {
    subscription_ = node_->create_subscription<MessageT>(
      node_->generate_global_name(config_.topic), qos,
      [this](typename MessageT::ConstSharedPtr msg) {
        last_msg_ = msg;
        messageReceived();
      });
    timer_ = node_->create_timer(
      std::chrono::duration<double>(1.0 / config_.frequency), [this]() {evaluate();});
  }

protected:
  /**
   * @brief Check a message against the condition.
   *
   * @param msg Last received message.
   * @return Whether the condition is violated, nullopt if the message cannot be evaluated.
   */
  virtual std::optional<bool> isViolated(const MessageT & msg) = 0;

  /**
   * @brief Describe a message that violates the condition.
   *
   * @param msg Last received message.
   * @return State that raised the alert, for the log.
   */
  virtual std::string describe(const MessageT & msg) const = 0;

private:
  void evaluate()
  {
    if (!messageAvailable()) {
      return;
    }
    const std::optional<bool> violated = isViolated(*last_msg_);
    if (violated.has_value() && updateTrigger(violated.value())) {
      publishAlert(describe(*last_msg_));
    }
  }

  typename MessageT::ConstSharedPtr last_msg_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace as2_safety_monitor

#endif  // AS2_SAFETY_MONITOR__SAFETY_CHECK_HPP_
