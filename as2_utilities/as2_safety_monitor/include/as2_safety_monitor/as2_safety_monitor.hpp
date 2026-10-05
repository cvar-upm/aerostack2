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
* @file as2_safety_monitor.hpp
*
* as2_safety_monitor header file.
*
* @author Javilinos
*/

#ifndef AS2_SAFETY_MONITOR__AS2_SAFETY_MONITOR_HPP_
#define AS2_SAFETY_MONITOR__AS2_SAFETY_MONITOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "as2_core/node.hpp"
#include "as2_safety_monitor/safety_check.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_safety_monitor
{

/**
 * @brief Node that runs the enabled safety checks and publishes their alerts.
 */
class SafetyMonitor : public as2::Node
{
public:
  /**
   * @brief Construct a new SafetyMonitor, creating every enabled check.
   *
   * @param options Node options.
   */
  explicit SafetyMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Create a check when its "<name>.enabled" parameter is true.
   *
   * @tparam CheckT Check type.
   * @param name Check name, used as parameter prefix.
   */
  template<typename CheckT>
  void addCheck(const std::string & name)
  {
    if (!this->getParameter<bool>(name + ".enabled", false)) {
      return;
    }
    checks_.push_back(std::make_unique<CheckT>(this, name, alert_pub_));
    RCLCPP_INFO(this->get_logger(), "Check [%s] enabled", name.c_str());
  }

  SafetyCheckBase::AlertPublisher::SharedPtr alert_pub_;
  std::vector<std::unique_ptr<SafetyCheckBase>> checks_;
};

}  // namespace as2_safety_monitor

#endif  // AS2_SAFETY_MONITOR__AS2_SAFETY_MONITOR_HPP_
