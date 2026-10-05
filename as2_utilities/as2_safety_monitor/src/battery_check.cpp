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
* @file battery_check.cpp
*
* Battery check source file.
*
* @author Javilinos
*/

#include "as2_safety_monitor/battery_check.hpp"

#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>

#include "as2_core/names/topics.hpp"

namespace as2_safety_monitor
{

BatteryCheck::BatteryCheck(
  as2::Node * node, const std::string & name, const AlertPublisher::SharedPtr & alert_pub)
: SafetyCheck(node, name, as2_names::topics::sensor_measurements::qos, alert_pub),
  voltage_threshold_(node->getParameter<double>(name + ".voltage_threshold"))
{
  if (!std::isfinite(voltage_threshold_) || voltage_threshold_ <= 0.0) {
    throw std::invalid_argument(name + ".voltage_threshold must be finite and > 0");
  }
}

std::optional<bool> BatteryCheck::isViolated(const sensor_msgs::msg::BatteryState & msg)
{
  // A missing or non positive reading is not evaluated
  if (!std::isfinite(msg.voltage) || msg.voltage <= 0.0f) {
    return std::nullopt;
  }
  return msg.voltage < voltage_threshold_;
}

std::string BatteryCheck::describe(const sensor_msgs::msg::BatteryState & msg) const
{
  char text[64];
  std::snprintf(
    text, sizeof(text), "voltage %.2f V below %.2f V", msg.voltage, voltage_threshold_);
  return text;
}

}  // namespace as2_safety_monitor
