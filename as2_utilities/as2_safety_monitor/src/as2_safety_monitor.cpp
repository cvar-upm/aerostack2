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
* @file as2_safety_monitor.cpp
*
* as2_safety_monitor source file.
*
* @author Javilinos
*/

#include "as2_safety_monitor/as2_safety_monitor.hpp"

#include "as2_core/names/topics.hpp"
#include "as2_safety_monitor/battery_check.hpp"
#include "as2_safety_monitor/geocage_check.hpp"

namespace as2_safety_monitor
{

SafetyMonitor::SafetyMonitor(const rclcpp::NodeOptions & options)
: as2::Node("safety_monitor", options)
{
  alert_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
    this->generate_global_name(as2_names::topics::global::alert_event),
    as2_names::topics::global::qos);

  addCheck<BatteryCheck>("battery");
  addCheck<GeocageCheck>("geocage");

  if (checks_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No safety check enabled");
  }
}

}  // namespace as2_safety_monitor
